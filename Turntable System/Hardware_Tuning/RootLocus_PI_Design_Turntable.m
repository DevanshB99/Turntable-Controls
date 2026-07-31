%% Root-Locus PID Design for the Identified Turntable Plant (belt-driven, 4:1)
% Loads the plant identified by System_ID_Turntable.m and places the
% closed-loop poles directly - the closed-form equivalent of reading a
% gain/damping point off a root locus plot. Same methodology as the
% bare-motor design script; only the loaded plant differs.
clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(scriptDir, 'Results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

load(fullfile(scriptDir, 'plant_model_turntable.mat'), 'sys_p');
sys_p

%% Step 1: Plant approximation
% Check whether the bare-motor assumption (near-pure integrator) still
% holds, or whether the belt's compliance shows up as a real second pole
% pair (resonance) at some frequency - look at how far the Bode plots
% actually agree before trusting the integrator simplification below.
[num, den] = tfdata(sys_p, 'v');
K = num(end);
p = den(end);
fprintf('Plant gain K = %.5f deg/step, pole p = %.5f rad/s\n', K, p);

sys_p_ideal = tf(K, [1 0]);  % pure-integrator approximation, K/s

fig1 = figure;
bode(sys_p, sys_p_ideal);
legend('Identified plant', 'Ideal integrator approximation');
grid on;
title('Turntable Plant vs. Ideal Integrator Approximation');
exportgraphics(fig1, fullfile(resultsDir, 'rl_bode_comparison.png'));

%% Step 2: Choose the design target
% zeta:  damping ratio (1 = no overshoot, 0.7-0.8 = small overshoot)
% settle_time: desired 2% settling time, for the LINEAR (unsaturated)
%              part of the response near the target.
zeta = 1.0;
settle_time = 1.0;
wn = 4 / (zeta * settle_time);
z  = wn / (2*zeta);

fprintf('\nDesign target: zeta=%.2f, wn=%.2f rad/s, settle~%.2fs, PI zero at s=-%.3f\n', ...
    zeta, wn, settle_time, z);

%% Step 3: Root locus of the PI-compensated plant
C0 = tf([1 z], [1 0]);   % (s+z)/s, unity gain
L0 = C0 * sys_p;

fig2 = figure;
rlocus(L0);
sgrid(min(zeta, 0.999), []);  % sgrid can't draw a grid line at exactly zeta=1 (real, repeated poles)
title(sprintf('Root Locus, PI zero at s=-%.2f (\\zeta=%.2f grid)', z, zeta));
grid on;
exportgraphics(fig2, fullfile(resultsDir, 'rl_root_locus.png'));

%% Step 4: Solve for Kp, Ki at the design point
Kp = 2*zeta*wn / K;
Ki = wn^2 / K;

fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);

cl_poles = pole(feedback(pid(Kp, Ki) * sys_p, 1));

fig3 = figure;
rlocus(L0);
hold on;
plot(real(cl_poles), imag(cl_poles), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
sgrid(min(zeta, 0.999), []);
title('Root Locus with Design Point Marked');
grid on;
exportgraphics(fig3, fullfile(resultsDir, 'rl_root_locus_design_point.png'));

%% Step 5: Verify with the closed-loop step response
C_pi = pid(Kp, Ki);
T_cl = feedback(C_pi * sys_p, 1);

fig4 = figure;
step(T_cl, 3*settle_time);
grid on;
title(sprintf('Closed-Loop Step Response (Kp=%.3f, Ki=%.3f)', Kp, Ki));
exportgraphics(fig4, fullfile(resultsDir, 'rl_pi_step_response.png'));

info = stepinfo(T_cl);
fprintf('\nPredicted overshoot: %.1f%%\n', info.Overshoot);
fprintf('Predicted 2%% settling time: %.2f s\n', info.SettlingTime);

%% Step 6: Save for hardware testing
save(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp', 'Ki', 'zeta', 'wn');
fprintf('\nKp/Ki saved to pid_rootlocus_turntable.mat\n');

%% Step 7: Zeta sweep - actual overshoot vs. design zeta (PI only)
zeta_range = 0.3:0.05:1.0;
overshoot_actual = zeros(size(zeta_range));

for i = 1:length(zeta_range)
    zi  = zeta_range(i);
    wni = 4 / (zi * settle_time);
    Ti  = feedback(pid(2*zi*wni/K, wni^2/K) * sys_p, 1);
    infoi = stepinfo(Ti);
    overshoot_actual(i) = infoi.Overshoot;
end

fig5 = figure;
plot(zeta_range, overshoot_actual, 'o-', 'LineWidth', 1.5);
xlabel('\zeta (design target)');
ylabel('Actual overshoot (%)');
title(sprintf('Actual PI Closed-Loop Overshoot vs. Design \\zeta (settle~%.1fs)', settle_time));
grid on;
exportgraphics(fig5, fullfile(resultsDir, 'rl_zeta_sweep_overshoot.png'));

%% Step 8: PID design - use Kd to counter the PI-zero overshoot
Td_ratio = 0.2;              % Td as a fraction of 1/wn - adjust and re-run
Td = Td_ratio / wn;
Kd = Td * Kp;

Kp_pid = 2*zeta*wn*(1 + K*Kd) / K;
Ki_pid = wn^2*(1 + K*Kd) / K;

C_pid_rl = pid(Kp_pid, Ki_pid, Kd);
T_cl_pid = feedback(C_pid_rl * sys_p, 1);

fig6 = figure;
step(T_cl, T_cl_pid, 3*settle_time);
legend('PI only', 'PID');
grid on;
title('PI vs. PID Closed-Loop Step Response');
exportgraphics(fig6, fullfile(resultsDir, 'rl_pi_vs_pid_step_response.png'));

info_pid = stepinfo(T_cl_pid);
fprintf('\nPID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp_pid, Ki_pid, Kd);
fprintf('PID predicted overshoot: %.1f%%\n', info_pid.Overshoot);
fprintf('PID predicted 2%% settling time: %.2f s\n', info_pid.SettlingTime);

%% Step 8b: Td_ratio sweep - actual overshoot vs. derivative action
Td_ratio_range = 0:0.05:0.6;
overshoot_vs_Td = zeros(size(Td_ratio_range));

for i = 1:length(Td_ratio_range)
    Kdi = (Td_ratio_range(i) / wn) * Kp;
    Kpi = 2*zeta*wn*(1 + K*Kdi) / K;
    Kii = wn^2*(1 + K*Kdi) / K;
    Ti  = feedback(pid(Kpi, Kii, Kdi) * sys_p, 1);
    infoi = stepinfo(Ti);
    overshoot_vs_Td(i) = infoi.Overshoot;
end

fig7 = figure;
plot(Td_ratio_range, overshoot_vs_Td, 'o-', 'LineWidth', 1.5);
xlabel('Td_{ratio}');
ylabel('Actual overshoot (%)');
title(sprintf('Actual PID Overshoot vs. Td_{ratio} (\\zeta=%.2f, \\omega_n=%.2f)', zeta, wn));
grid on;
exportgraphics(fig7, fullfile(resultsDir, 'rl_td_ratio_sweep_overshoot.png'));

save(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp_pid', 'Ki_pid', 'Kd', '-append');
fprintf('PID gains appended to pid_rootlocus_turntable.mat\n');
fprintf('Plots saved to %s\n', resultsDir);

%% Step 8c: Filtered derivative - phase margin check
% Must match Turntable_TMC_Combined.ino's D_FILTER_ALPHA. If the belt
% turns out to have a real resonance (Step 1 flagged it), reconsider Tf
% relative to THAT frequency, not just copying this starting point.
Ts_ctrl = 0.02;
D_FILTER_ALPHA = 0.2;   % must match Turntable_TMC_Combined.ino
Tf = Ts_ctrl * (1 - D_FILTER_ALPHA) / D_FILTER_ALPHA;
fprintf('\nDerivative filter time constant Tf = %.4f s (alpha=%.2f)\n', Tf, D_FILTER_ALPHA);

C_pid_filtered = pid(Kp_pid, Ki_pid, Kd, Tf);

[~, Pm_i, ~, Wcp_i] = margin(C_pid_rl * sys_p);
[~, Pm_f, ~, Wcp_f] = margin(C_pid_filtered * sys_p);

fprintf('Ideal derivative:    phase margin = %.1f deg at %.2f rad/s\n', Pm_i, Wcp_i);
fprintf('Filtered derivative: phase margin = %.1f deg at %.2f rad/s\n', Pm_f, Wcp_f);
fprintf('Phase lag introduced by the filter: %.1f deg\n', Pm_i - Pm_f);

T_cl_filtered = feedback(C_pid_filtered * sys_p, 1);
info_filtered = stepinfo(T_cl_filtered);
fprintf('Filtered-derivative predicted overshoot: %.1f%%\n', info_filtered.Overshoot);
fprintf('Filtered-derivative predicted 2%% settling time: %.2f s\n', info_filtered.SettlingTime);

fig8 = figure;
bode(C_pid_rl*sys_p, C_pid_filtered*sys_p);
legend('Ideal derivative', 'Filtered derivative');
grid on;
title('Open-Loop Bode: Ideal vs. Filtered Derivative');
exportgraphics(fig8, fullfile(resultsDir, 'rl_filtered_derivative_bode.png'));

fig9 = figure;
step(T_cl_pid, T_cl_filtered, 3*settle_time);
legend('Ideal derivative', 'Filtered derivative');
grid on;
title('Closed-Loop Step Response: Ideal vs. Filtered Derivative');
exportgraphics(fig9, fullfile(resultsDir, 'rl_filtered_derivative_step.png'));

%% Step 8d: Mitigate the phase lag if it's significant
if (Pm_i - Pm_f) > 5
    Kd_compensated = Kd * 1.3;
    C_pid_comp = pid(Kp_pid, Ki_pid, Kd_compensated, Tf);
    [~, Pm_comp] = margin(C_pid_comp * sys_p);
    T_cl_comp = feedback(C_pid_comp * sys_p, 1);
    info_comp = stepinfo(T_cl_comp);

    fprintf('\nMitigation: increased Kd from %.4f to %.4f\n', Kd, Kd_compensated);
    fprintf('Recovered phase margin: %.1f deg (was %.1f deg filtered, %.1f deg ideal)\n', ...
        Pm_comp, Pm_f, Pm_i);
    fprintf('Compensated overshoot: %.1f%%, settling: %.2f s\n', info_comp.Overshoot, info_comp.SettlingTime);

    fig10 = figure;
    step(T_cl_pid, T_cl_filtered, T_cl_comp, 3*settle_time);
    legend('Ideal derivative', 'Filtered (uncompensated)', 'Filtered + higher Kd');
    grid on;
    title('Mitigating the Filter''s Phase Lag with Higher Kd');
    exportgraphics(fig10, fullfile(resultsDir, 'rl_filtered_derivative_mitigated.png'));
else
    fprintf('\nPhase margin loss is small (<5 deg) - no compensation needed.\n');
end

%% Step 9: Fine-tune interactively before touching hardware
% Opens the PID Tuner app pre-loaded with the plant and the FILTERED PID
% design (the realistic one, matching what firmware actually runs) as the
% starting point.
pidTuner(sys_p, C_pid_filtered);
