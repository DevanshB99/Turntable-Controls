port = "/dev/cu.usbserial-0001";
baud = 115200;
s = serialport(port, baud);
configureTerminator(s,"LF");
flush(s);
pause(2);

Ts = 0.02;
N  = 3000;
amp = 500;

deg2rad = pi/180;

%%GENERATE PRBS
u = idinput(N,'prbs',[0 0.3]) * amp;
y = zeros(N,1);
t = zeros(N,1);

disp('Running PRBS experiment...');
tic;
k = 1;
while k <= N
    writeline(s, sprintf("V%.2f", u(k)));   
    if s.NumBytesAvailable > 0
        y(k) = str2double(readline(s));      
        t(k) = toc;
        k = k + 1;
    else
        pause(Ts);
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

%simple detrend the output to remove bias
y_clean = detrend(y_unwrapped);

data = iddata(y_clean, u, Ts);

%%SYSTEM IDENTIFICATION
sys1 = tfest(data, 1, 0);
sys2 = tfest(data, 2, 0);
sys3 = tfest(data, 2, 1);
sys4 = tfest(data, 3, 1);

figure;
compare(data, sys1, sys2, sys3, sys4);
title('Position Plant Validation');
legend('Measured', 'sys1 (1p)', 'sys2 (2p)', 'sys3 (2p1z)', 'sys4(3p1z)');
grid on;

%select best fit
[~, fit1] = compare(data, sys1);
[~, fit2] = compare(data, sys2);
[~, fit3] = compare(data, sys3);
[~, fit4] = compare(data, sys4);

fprintf('\nFIT PERCENTAGES\n');
fprintf('sys1 (1p):    %.2f%%\n', fit1);
fprintf('sys2 (2p):    %.2f%%\n', fit2);
fprintf('sys3 (2p1z):  %.2f%%\n', fit3);
fprintf('sys4 (3p1z):  %.2f%%\n', fit4);

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

fprintf('\nIDENTIFIED POSITION PLANT\n');

%%BODE & ROOT LOCUS
figure;
bode(sys_p); 
grid on;
title('Position Plant Bode');

figure;
rlocus(sys_p); 
grid on;
title('Position Plant Root Locus');

%%CONTROLLER DESIGN
C_pid = pidtune(sys_p, 'PID');

figure;
step(feedback(C_pid * sys_p, 1), 5);
grid on;
title('Continuous-Time Closed Loop');

%%DISCRETE CONTROLLER
sys_pd = c2d(sys_p, Ts, 'tustin');
C_pid_d = c2d(C_pid, Ts, 'tustin');

T_cl_d = feedback(C_pid_d * sys_pd, 1);

figure;
step(T_cl_d, 5);
grid on;
title('Discrete-Time Closed Loop (ESP32)');

fprintf('\nSystem ID Complete!\n');