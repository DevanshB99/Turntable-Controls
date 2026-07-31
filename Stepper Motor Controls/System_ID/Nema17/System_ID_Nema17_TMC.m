scriptDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(scriptDir, 'Results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

port = "/dev/cu.usbserial-0001";
baud = 115200;
s = serialport(port, baud);
configureTerminator(s,"LF");
flush(s);
pause(2);

Ts = 0.02;
N  = 3000;
amp = 500;
STEPS_PER_REV = 200 * 16;  % 200 full steps x 1/16 microstepping (TMC2209 MS pins tied to 3.3V)

deg2rad = pi/180;

%%GENERATE PRBS
u = idinput(N,'prbs',[0 0.3]) * amp;
y = nan(N,1);

disp('Running PRBS experiment...');
for k = 1:N
    writeline(s, sprintf("V%.2f", u(k)));
    pause(Ts);

    % drain any backlog and keep only the freshest reading, so y(k)
    % stays time-aligned with u(k) instead of drifting behind over the run
    newLine = "";
    while s.NumBytesAvailable > 0
        newLine = readline(s);
    end
    if newLine ~= ""
        y(k) = str2double(newLine);
    end
end

writeline(s,"STOP");
delete(s);

%%DATA PREPROCESSING
valid = ~isnan(y);
u = u(valid);
y = y(valid);

%unwrap position
y_unwrapped = unwrap(y * deg2rad) / deg2rad;
y_unwrapped = y_unwrapped - y_unwrapped(1);

%%VELOCITY-DOMAIN GAIN CHECK
% An open-loop stepper commanded by step rate should behave as a pure
% integrator (theta_dot = K*u, no dynamics). Differentiating position
% and regressing against u directly checks that static gain K,
% independent of any pole-near-the-origin fitting ambiguity.
v = smoothdata(gradient(y_unwrapped, Ts), 'movmean', 5);
K_hat = u \ v;
fitPercent_vel = 100 * (1 - norm(v - K_hat*u) / norm(v - mean(v)));

fprintf('\nVELOCITY-DOMAIN GAIN CHECK\n');
fprintf('Theoretical gain:      %.4f deg/step\n', 360 / STEPS_PER_REV);
fprintf('Velocity-domain gain:  %.4f deg/step (static-gain fit %.2f%%)\n', K_hat, fitPercent_vel);

t_plot = (0:length(u)-1)' * Ts;
figv = figure;
plot(t_plot, v, t_plot, K_hat*u, '--');
xlabel('Time (s)'); ylabel('Angular velocity (deg/s)');
legend('Measured (differentiated)', 'K_{hat} \times u');
title(sprintf('Velocity-Domain Gain Check (K = %.4f deg/step, fit %.2f%%)', K_hat, fitPercent_vel));
grid on;
exportgraphics(figv, fullfile(resultsDir, 'velocity_gain_check.png'));

%simple detrend the output to remove bias
y_clean = detrend(y_unwrapped);

data = iddata(y_clean, u, Ts);

%%SYSTEM IDENTIFICATION
sys1 = tfest(data, 1, 0);
sys2 = tfest(data, 2, 0);
sys3 = tfest(data, 2, 1);
sys4 = tfest(data, 3, 1);

fig1 = figure;
compare(data, sys1, sys2, sys3, sys4);
title('Position Plant Validation');
legend('Measured', 'sys1 (1p)', 'sys2 (2p)', 'sys3 (2p1z)', 'sys4(3p1z)');
grid on;
exportgraphics(fig1, fullfile(resultsDir, 'position_plant_validation.png'));

%select best fit
[~, fit1] = compare(data, sys1);
[~, fit2] = compare(data, sys2);
[~, fit3] = compare(data, sys3);
[~, fit4] = compare(data, sys4);

fprintf('\nFIT PERCENTAGES\n');
fprintf('sys1 (1p):    %.2f%%\n', fit1);
sys1
fprintf('sys2 (2p):    %.2f%%\n', fit2);
sys2
fprintf('sys3 (2p1z):  %.2f%%\n', fit3);
sys3
fprintf('sys4 (3p1z):  %.2f%%\n', fit4);
sys4

if fit1 > 80
    sys_p = sys1;
    fprintf('\nSelected: sys1\n');
elseif fit2 > 80
    sys_p = sys2;
    fprintf('\nSelected: sys2\n');
elseif fit3 > 80
    sys_p = sys3;
    fprintf('\nSelected: sys3\n');
else
    sys_p = sys4;
    fprintf('\nSelected: sys4\n');
end

fprintf('\nIDENTIFIED POSITION PLANT TRANSFER FUNCTION\n');
sys_p


%%STATE-SPACE REALIZATION
sys_ss = ss(tf(sys_p));
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

printMatrix = @(name, M) fprintf('%s (%dx%d):\n%s\n\n', name, size(M,1), size(M,2), mat2str(M, 6));

fprintf('\nSTATE-SPACE MATRICES (%d state%s)\n', size(A,1), repmat('s', 1, size(A,1)~=1));
printMatrix('A', A);
printMatrix('B', B);
printMatrix('C', C);
printMatrix('D', D);

%%DISCRETIZE PLANT
sys_pd = c2d(sys_p, Ts, 'tustin');

%%OPEN-LOOP RESPONSE: CONTINUOUS VS DISCRETE
Tsim = 5;
t_resp = (0:Ts:Tsim)';
ramp_in = t_resp;

fig6 = figure;
subplot(2,3,1); impulse(sys_p, Tsim); grid on; title('Continuous Impulse');
subplot(2,3,2); step(sys_p, Tsim); grid on; title('Continuous Step');
subplot(2,3,3); lsim(sys_p, ramp_in, t_resp); grid on; title('Continuous Ramp');
subplot(2,3,4); impulse(sys_pd, Tsim); grid on; title('Discrete Impulse');
subplot(2,3,5); step(sys_pd, Tsim); grid on; title('Discrete Step');
subplot(2,3,6); lsim(sys_pd, ramp_in, t_resp); grid on; title('Discrete Ramp');
sgtitle('Open-Loop Plant Response: Continuous vs Discrete');
exportgraphics(fig6, fullfile(resultsDir, 'response_comparison.png'));

%%BODE & ROOT LOCUS
fig2 = figure;
bode(sys_p);
grid on;
title('Position Plant Bode');
exportgraphics(fig2, fullfile(resultsDir, 'bode.png'));

fig3 = figure;
rlocus(sys_p);
grid on;
title('Position Plant Root Locus');
exportgraphics(fig3, fullfile(resultsDir, 'root_locus.png'));

%%CONTROLLER DESIGN
C_pid = pidtune(sys_p, 'PID');

fig4 = figure;
step(feedback(C_pid * sys_p, 1), 5);
grid on;
title('Continuous-Time Closed Loop');
exportgraphics(fig4, fullfile(resultsDir, 'closed_loop_continuous.png'));

%%DISCRETE CONTROLLER
C_pid_d = c2d(C_pid, Ts, 'tustin');

T_cl_d = feedback(C_pid_d * sys_pd, 1);

fig5 = figure;
step(T_cl_d, 5);
grid on;
title('Discrete-Time Closed Loop (ESP32)');
exportgraphics(fig5, fullfile(resultsDir, 'closed_loop_discrete.png'));

fprintf('\nSystem ID Complete! Plots saved to %s\n', resultsDir);
