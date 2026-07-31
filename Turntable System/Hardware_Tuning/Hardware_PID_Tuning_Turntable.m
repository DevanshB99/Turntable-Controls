% Do NOT clear here - if you just exported a controller from pidTuner as
% "C", clearing would destroy it before the lines below ever get to use it.

port = "/dev/cu.usbserial-0001";
baud = 115200;

scriptDir = '/Users/devanshbajwala/Documents/PhD/Visual Inspection/Turntable_Project/Turntable-Controls/Turntable System/Hardware_Tuning';

% PID GAINS - use the pidTuner-exported "C" directly if it's still in the
% workspace (also persists it to disk as Kp_final/Ki_final/Kd_final so a
% later run still has it even after C is gone). Otherwise fall back to
% whatever was last persisted that way.
if exist('C', 'var')
    Kp = C.Kp;
    Ki = C.Ki;
    Kd = C.Kd;
    Kp_final = Kp; Ki_final = Ki; Kd_final = Kd;
    save(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp_final', 'Ki_final', 'Kd_final', '-append');
else
    load(fullfile(scriptDir, 'pid_rootlocus_turntable.mat'), 'Kp_final', 'Ki_final', 'Kd_final');
    Kp = Kp_final;
    Ki = Ki_final;
    Kd = Kd_final;
end

% Test parameters
target_angle = 90;   % Target position (deg) - turntable frame
test_duration = 5;   % Test duration (s)
Ts = 0.02;           % Sampling time (s)

%% Connect
s = serialport(port, baud);
configureTerminator(s, "LF");
pause(2);
flush(s);

fprintf('=== Hardware PID Test (Turntable) ===\n');
fprintf('Kp = %.3f, Ki = %.3f, Kd = %.3f\n', Kp, Ki, Kd);
fprintf('Target = %.0f deg\n\n', target_angle);

%% HOME TO ZERO
fprintf('Step 1: Homing to 0 degrees...\n');
writeline(s, "HOME");

homed = false;
while ~homed
    if s.NumBytesAvailable > 0
        response = readline(s);
        if startsWith(response, "HOMED:")
            final_pos = str2double(extractAfter(response, "HOMED:"));
            fprintf('  Homed to: %.2f deg\n', final_pos);
            homed = true;
        elseif response == "HOMING"
            fprintf('  Homing in progress...\n');
        end
    end
    pause(0.1);
end

pause(1);  % Let motor settle

%% VERIFY STARTING POSITION
writeline(s, "READ");
pause(0.1);
if s.NumBytesAvailable > 0
    start_pos = str2double(readline(s));
    fprintf('  Verified start position: %.2f deg\n\n', start_pos);
end

%% SEND PID GAINS
fprintf('Step 2: Uploading PID gains...\n');
writeline(s, sprintf("KP%.4f", Kp));
pause(0.1);
readline(s);

writeline(s, sprintf("KI%.4f", Ki));
pause(0.1);
readline(s);

writeline(s, sprintf("KD%.4f", Kd));
pause(0.1);
readline(s);

writeline(s, sprintf("TARGET%.1f", target_angle));
pause(0.1);
readline(s);

fprintf('  Gains uploaded\n\n');

%% RUN TEST
fprintf('Step 3: Running PID test...\n');
N = round(test_duration / Ts);
t = zeros(N, 1);
angle = zeros(N, 1);
error_vals = zeros(N, 1);
command = zeros(N, 1);

writeline(s, "START");
pause(0.1);
readline(s);

fig = figure('Position', [100, 100, 1200, 800]);

subplot(3,1,1);
h1 = animatedline('Color', 'b', 'LineWidth', 2);
hold on;
yline(target_angle, 'r--', 'LineWidth', 1.5);
ylabel('Angle (deg)');
title(sprintf('Live Response | Kp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));
grid on;

subplot(3,1,2);
h2 = animatedline('Color', 'r', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Error (deg)');
grid on;

subplot(3,1,3);
h3 = animatedline('Color', 'g', 'LineWidth', 1.5);
ylabel('Command (microsteps/s)');
xlabel('Time (s)');
grid on;

tic;
for k = 1:N
    if s.NumBytesAvailable > 0
        line = readline(s);
        data = sscanf(line, "A%f,E%f,C%f");

        if length(data) == 3
            t(k) = toc;
            angle(k) = data(1);
            error_vals(k) = data(2);
            command(k) = data(3);

            addpoints(h1, t(k), angle(k));
            addpoints(h2, t(k), error_vals(k));
            addpoints(h3, t(k), command(k));

            if mod(k, 5) == 0
                drawnow;
            end
        end
    else
        pause(Ts);
    end
end

writeline(s, "STOP");
delete(s);

%% ANALYSIS
fprintf('\n=== Test Results ===\n');

valid = t > 0;
t = t(valid);
angle = angle(valid);
error_vals = error_vals(valid);
command = command(valid);

% Continuous position relative to target (target - wrapped error), immune
% to the raw 0/360 encoder seam that corrupted these metrics whenever a
% test starts right after homing, i.e. right at the wrap boundary
pos = target_angle - error_vals;
initial_error = target_angle - pos(1);

final_value = mean(pos(end-20:end));
steady_state_error = abs(target_angle - final_value);

delta = pos - target_angle;
crossed = find(sign(delta) ~= sign(delta(1)), 1);
if ~isempty(crossed)
    overshoot = max(abs(delta(crossed:end)));
else
    overshoot = 0;
end

idx_10 = find(abs(pos - pos(1)) > 0.1*abs(initial_error), 1);
idx_90 = find(abs(pos - pos(1)) > 0.9*abs(initial_error), 1);
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time = t(idx_90) - t(idx_10);
    fprintf('Rise Time: %.0f ms\n', rise_time*1000);
end

settled_idx = find(abs(pos - target_angle) < 0.02*abs(target_angle), 1);
if ~isempty(settled_idx)
    settling_time = t(settled_idx);
    fprintf('Settling Time: %.0f ms\n', settling_time*1000);
end

fprintf('Overshoot: %.1f deg (%.1f%%)\n', overshoot, 100*overshoot/target_angle);
fprintf('Final Position: %.1f deg\n', final_value);
fprintf('Steady-State Error: %.1f deg (%.1f%%)\n', steady_state_error, 100*steady_state_error/target_angle);
fprintf('Max Command: %.0f microsteps/s\n', max(abs(command)));

%%RECOMMENDATIONS
fprintf('\n=== Tuning Recommendations ===\n');

if overshoot > 0.15 * target_angle
    fprintf('Overshoot too high (%.1f%%) -> Reduce Kp to %.2f\n', 100*overshoot/target_angle, Kp*0.7);
elseif overshoot > 0.05 * target_angle
    fprintf('Moderate overshoot (%.1f%%) -> Reduce Kp slightly to %.2f\n', 100*overshoot/target_angle, Kp*0.85);
else
    fprintf('Overshoot acceptable (%.1f%%)\n', 100*overshoot/target_angle);
end

if steady_state_error > 2
    fprintf('Steady-state error too high (%.1f deg) -> Increase Ki to %.2f\n', steady_state_error, Ki*1.5);
elseif steady_state_error > 0.5
    fprintf('Small steady-state error (%.1f deg) -> Increase Ki to %.2f\n', steady_state_error, Ki*1.2);
else
    fprintf('Steady-state error acceptable (%.1f deg)\n', steady_state_error);
end

fprintf('\n');
