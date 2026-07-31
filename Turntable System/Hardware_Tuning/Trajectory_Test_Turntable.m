clear; clc; close all;

port = "/dev/cu.usbserial-0001";
baud = 115200;
scriptDir = fileparts(mfilename('fullpath'));

TOLERANCE_DEG = 0.2;  % required precision at the end of the move

%% PID GAINS - same convention as Hardware_PID_Tuning_Turntable.m
if exist('C', 'var')
    Kp = C.Kp; Ki = C.Ki; Kd = C.Kd;
    Kp_final = Kp; Ki_final = Ki; Kd_final = Kd;
    save(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp_final', 'Ki_final', 'Kd_final', '-append');
else
    load(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp_final', 'Ki_final', 'Kd_final');
    Kp = Kp_final; Ki = Ki_final; Kd = Kd_final;
end

%% RANDOM TRAJECTORY - similar to a ROS2 JointTrajectoryPoint:
% (position, time_from_start), here a random target angle and a random
% total move duration, in TURNTABLE degrees. Distance/duration ranges
% are carried over from the bare-motor test as a starting point - the
% turntable's actual achievable peak velocity for a given step rate is
% now ~4x lower (belt reduction), and MAX_STEP_RATE hasn't been
% re-validated for this system yet, so re-check these ranges once it has.
rng('shuffle');
start_angle = 0;                                   % we home to 0 first
distance    = 20 + (150 - 20) * rand();             % 20-150 deg
direction   = sign(rand() - 0.5);
target_angle = direction * distance;
traj_duration = 1.5 + (3.5 - 1.5) * rand();         % 1.5-3.5 s
dwell_time    = 4.0;                                % hold at target after the move, to check final precision

fprintf('=== Random Trajectory (Turntable) ===\n');
fprintf('Target: %.2f deg in %.2f s (+ %.1f s dwell)\n', target_angle, traj_duration, dwell_time);

%% SINUSOIDAL-ACCELERATION VELOCITY PROFILE
% Acceleration follows one full sine cycle over the move duration, so
% velocity AND acceleration are both exactly zero at the start and end -
% no jerk discontinuity like a trapezoidal profile has at its corners.
Ts = 0.02;
T  = traj_duration;
D  = target_angle - start_angle;
A  = D * 2*pi / T^2;   % acceleration scale factor

t_move = (0:Ts:T)';
acc_ref_move = A * sin(2*pi*t_move/T);
vel_ref_move = (A*T/(2*pi)) * (1 - cos(2*pi*t_move/T));
pos_ref_move = start_angle + (A*T/(2*pi))*t_move - (A*T^2/(4*pi^2))*sin(2*pi*t_move/T);

t_dwell = (Ts:Ts:dwell_time)';
pos_ref_dwell = target_angle * ones(size(t_dwell));
vel_ref_dwell = zeros(size(t_dwell));

t_ref   = [t_move; t_move(end) + t_dwell];
pos_ref = [pos_ref_move; pos_ref_dwell];
vel_ref = [vel_ref_move; vel_ref_dwell];

Vmax = max(abs(vel_ref_move));
Amax = max(abs(acc_ref_move));
fprintf('Peak velocity: %.1f deg/s, peak acceleration: %.1f deg/s^2\n\n', Vmax, Amax);

%% Connect
s = serialport(port, baud);
configureTerminator(s, "LF");
pause(2);
flush(s);

%% HOME TO ZERO
fprintf('Homing to 0 degrees...\n');
writeline(s, "HOME");
homed = false;
while ~homed
    if s.NumBytesAvailable > 0
        response = readline(s);
        if startsWith(response, "HOMED:")
            homed = true;
        end
    end
    pause(0.1);
end
pause(1);

%% UPLOAD GAINS
writeline(s, sprintf("KP%.4f", Kp));  pause(0.1); readline(s);
writeline(s, sprintf("KI%.4f", Ki));  pause(0.1); readline(s);
writeline(s, sprintf("KD%.4f", Kd));  pause(0.1); readline(s);

%% RUN TRAJECTORY
writeline(s, sprintf("TARGET%.2f", pos_ref(1)));
pause(0.1);
readline(s);

writeline(s, "START");
pause(0.1);
readline(s);

% Self-clocked to real elapsed time (toc), not loop iteration count -
% writeline/readline round-trip latency means iterations don't take
% exactly Ts, so the reference sent must be computed from actual elapsed
% time, and the loop must run until real time reaches Tend, not for a
% fixed number of iterations.
Tend = t_ref(end);
maxSamples = round(Tend / Ts) + 500;  % generous headroom
t = zeros(maxSamples, 1);
angle = zeros(maxSamples, 1);
error_vals = zeros(maxSamples, 1);
command = zeros(maxSamples, 1);
cmd_sent = zeros(maxSamples, 1);  % reference actually in effect at each sample

fprintf('Running trajectory...\n');
tic;
k = 0;
lastSend = -Inf;
ref_now = pos_ref(1);
while toc < Tend
    t_now = toc;

    if t_now - lastSend >= Ts
        if t_now <= T
            ref_now = start_angle + (A*T/(2*pi))*t_now - (A*T^2/(4*pi^2))*sin(2*pi*t_now/T);
        else
            ref_now = target_angle;
        end
        writeline(s, sprintf("TARGET%.2f", ref_now));
        lastSend = t_now;
    end

    if s.NumBytesAvailable > 0
        line = readline(s);
        data = sscanf(line, "A%f,E%f,C%f");
        if length(data) == 3
            k = k + 1;
            t(k) = t_now;
            angle(k) = data(1);
            error_vals(k) = data(2);
            command(k) = data(3);
            cmd_sent(k) = ref_now;
        end
    end
end

writeline(s, "STOP");
delete(s);

%% ANALYSIS
N_actual = k;
t = t(1:N_actual);
angle = angle(1:N_actual);
error_vals = error_vals(1:N_actual);
command = command(1:N_actual);
cmd_sent = cmd_sent(1:N_actual);

% Tracking error IS the firmware's own wrapped (target - actual) at each
% instant - already correct regardless of the 0/360 wrap seam.
tracking_error = error_vals;

% Wrap-immune actual position for plotting, reconstructed from whatever
% reference was actually in effect at each sample - the raw 0-360
% encoder reading falsely looks like a full-circle jump whenever it's
% near the wrap seam, which is exactly where every trajectory here starts.
pos_actual = cmd_sent - tracking_error;

max_tracking_error = max(abs(tracking_error));
dwell_mask = t > (t_move(end));
final_error = mean(abs(tracking_error(dwell_mask)));

% Split the dwell into an "early" window (matching the old 1.5s dwell)
% and everything after, to see directly whether error is still
% converging with more time or has already leveled off at a floor.
early_dwell_mask = t > t_move(end) & t <= (t_move(end) + 1.5);
late_dwell_mask  = t > (t_move(end) + 1.5);
early_dwell_error = mean(abs(tracking_error(early_dwell_mask)));
late_dwell_error  = mean(abs(tracking_error(late_dwell_mask)));

% Your spec is a PEAK deviation ("+/-0.2 deg at the maximum"), not an
% average - check both, since a small persistent oscillation can look
% fine on the mean while still occasionally exceeding the peak requirement.
max_dwell_error       = max(abs(tracking_error(dwell_mask)));
max_late_dwell_error  = max(abs(tracking_error(late_dwell_mask)));

fprintf('\n=== Results ===\n');
fprintf('Max tracking error during move: %.3f deg\n', max_tracking_error);
fprintf('Mean |error| during dwell (first 1.5s): %.3f deg\n', early_dwell_error);
fprintf('Mean |error| during dwell (after 1.5s):  %.3f deg\n', late_dwell_error);
fprintf('Mean |error| across full dwell: %.3f deg\n', final_error);
fprintf('Max |error| across full dwell:  %.3f deg\n', max_dwell_error);
fprintf('Max |error| during dwell (after 1.5s): %.3f deg\n', max_late_dwell_error);
if max_dwell_error <= TOLERANCE_DEG
    fprintf('MEETS +/-%.1f deg PEAK requirement at final dwell.\n', TOLERANCE_DEG);
else
    fprintf('DOES NOT meet +/-%.1f deg PEAK requirement at final dwell.\n', TOLERANCE_DEG);
end

%% PLOTS
resultsDir = fullfile(scriptDir, 'Results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

fig = figure('Position', [100, 100, 1200, 1100]);

subplot(4,1,1);
plot(t_ref, pos_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, pos_actual, 'b', 'LineWidth', 1.5);
xline(t_move(end), 'k:');
legend('Reference', 'Actual', 'End of move');
ylabel('Angle (deg)');
title('Turntable Trajectory Tracking (sinusoidal-acceleration profile)');
grid on;

subplot(4,1,2);
plot(t_ref, vel_ref, 'LineWidth', 1.5);
xline(t_move(end), 'k:');
ylabel('Velocity (deg/s)');
title('Desired Velocity Profile');
grid on;

subplot(4,1,3);
plot(t, tracking_error, 'LineWidth', 1.5); hold on;
yline(TOLERANCE_DEG, 'r--');
yline(-TOLERANCE_DEG, 'r--');
xline(t_move(end), 'k:');
ylabel('Tracking Error (deg)');
title(sprintf('Tracking Error (+/-%.1f deg tolerance)', TOLERANCE_DEG));
grid on;

subplot(4,1,4);
plot(t, command, 'LineWidth', 1.5);
xline(t_move(end), 'k:');
ylabel('Command (microsteps/s)');
xlabel('Time (s)');
title('Commanded Step Rate');
grid on;

exportgraphics(fig, fullfile(resultsDir, 'trajectory_test.png'));
fprintf('\nPlot saved to %s\n', fullfile(resultsDir, 'trajectory_test.png'));
