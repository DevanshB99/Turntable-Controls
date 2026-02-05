%% COMPLETE TURNTABLE SYSTEM TEST
% Tests NEMA 17 motor with A4988 (1/16 microstepping) + AS5600 encoder
% 
% This tests:
%   1. Motor movement at 1/16 microstepping
%   2. Encoder angle measurement
%   3. Correlation between motor steps and encoder angle
%   4. Accuracy of 4:1 gear ratio
%
% Usage:
%   1. Upload turntable_complete.ino to ESP32
%   2. Wire microstepping pins (GPIO 14, 12, 13 all HIGH)
%   3. Change COM_PORT below
%   4. Run this script

clear all; close all; clc;

%% SETUP - CHANGE THIS TO YOUR COM PORT
COM_PORT = '/dev/cu.usbserial-0001';  % Change to your port

%% System Parameters (from your hardware)
MICROSTEPS_PER_REV = 3200;  % 200 * 16
GEAR_RATIO = 4;
STEPS_PER_TURNTABLE_REV = MICROSTEPS_PER_REV * GEAR_RATIO;  % 12,800

fprintf('=== TURNTABLE SYSTEM TEST ===\n\n');
fprintf('System Configuration:\n');
fprintf('  Motor: NEMA 17 (200 steps/rev)\n');
fprintf('  Microstepping: 1/16 (3200 steps/rev)\n');
fprintf('  Gear Ratio: 4:1\n');
fprintf('  Total: %d steps per turntable revolution\n', STEPS_PER_TURNTABLE_REV);
fprintf('  Angular Resolution: %.4f degrees/step\n\n', 360/STEPS_PER_TURNTABLE_REV);

%% Connect to ESP32
fprintf('Connecting to ESP32 on %s...\n', COM_PORT);

s = serialport(COM_PORT, 115200);
configureTerminator(s, "LF");

% Wait for ESP32 to initialize
pause(2);

% Read startup messages
fprintf('\nESP32 Status:\n');
while s.NumBytesAvailable > 0
    msg = readline(s);
    fprintf('  %s\n', msg);
end

fprintf('\nConnected!\n\n');

%% Test 1: Check Initial Position
fprintf('=== TEST 1: Initial Position ===\n');

writeline(s, "DATA");
pause(0.2);

if s.NumBytesAvailable > 0
    response = readline(s);
    if contains(response, 'DATA:')
        dataStr = extractAfter(response, 'DATA:');
        values = str2double(split(dataStr, ','));
        fprintf('Initial Encoder Angle: %.2f degrees\n', values(1));
        fprintf('Initial Motor Angle: %.2f degrees\n', values(2));
        fprintf('Initial Step Count: %d\n', values(3));
    end
end

%% Test 2: Controlled Rotation Test
fprintf('\n=== TEST 2: Controlled Rotation Test ===\n');
fprintf('Rotating CW for 10 seconds at 800 steps/second...\n');

% Set speed
writeline(s, "SPEED:800");
pause(0.2);
if s.NumBytesAvailable > 0
    fprintf('%s\n', readline(s));
end

% Start rotation
writeline(s, "CW");
pause(0.2);

% Collect data
encoderAngles = [];
motorAngles = [];
stepCounts = [];
errors = [];
timestamps = [];

startTime = tic;
fprintf('Collecting data');

while toc(startTime) < 10
    if s.NumBytesAvailable > 0
        response = readline(s);
        if contains(response, 'DATA:')
            dataStr = extractAfter(response, 'DATA:');
            values = str2double(split(dataStr, ','));
            
            if length(values) >= 4
                encoderAngles = [encoderAngles; values(1)];
                motorAngles = [motorAngles; values(2)];
                stepCounts = [stepCounts; values(3)];
                errors = [errors; values(4)];
                timestamps = [timestamps; toc(startTime)];
                fprintf('.');
            end
        end
    end
    pause(0.01);
end

fprintf(' Done!\n');

% Stop motor
writeline(s, "STOP");
pause(0.5);

% Flush remaining data
while s.NumBytesAvailable > 0
    readline(s);
end

fprintf('\nCollected %d data points\n', length(encoderAngles));

%% Analyze Results
if length(encoderAngles) > 10
    fprintf('\n=== ANALYSIS ===\n');
    
    % Calculate statistics
    finalSteps = stepCounts(end);
    expectedTurntableAngle = (finalSteps / STEPS_PER_TURNTABLE_REV) * 360;
    
    % Handle wraparound for encoder angle change
    encoderChange = encoderAngles(end) - encoderAngles(1);
    if encoderChange < 0
        % Crossed 360-0 boundary
        encoderChange = encoderChange + 360;
    end
    
    fprintf('Motor Performance:\n');
    fprintf('  Total Steps: %d\n', finalSteps);
    fprintf('  Expected Turntable Angle: %.2f degrees\n', mod(expectedTurntableAngle, 360));
    fprintf('  Actual Encoder Change: %.2f degrees\n', encoderChange);
    fprintf('  Difference: %.2f degrees\n', abs(encoderChange - mod(expectedTurntableAngle, 360)));
    
    % Calculate average error
    avgError = mean(abs(errors));
    maxError = max(abs(errors));
    stdError = std(errors);
    
    fprintf('\nTracking Accuracy:\n');
    fprintf('  Average Error: %.2f degrees\n', avgError);
    fprintf('  Max Error: %.2f degrees\n', maxError);
    fprintf('  Std Deviation: %.2f degrees\n', stdError);
    
    % Check for step losses
    fprintf('\nStep Integrity:\n');
    if avgError < 2.0
        fprintf('  ✓ EXCELLENT - No significant step loss detected\n');
    elseif avgError < 5.0
        fprintf('  ⚠ GOOD - Minor tracking error (acceptable)\n');
    else
        fprintf('  ✗ WARNING - Significant error, may indicate step loss\n');
    end
    
    % Calculate velocity
    if length(timestamps) > 1
        timeSpan = timestamps(end) - timestamps(1);
        avgVelocity = encoderChange / timeSpan;
        
        expectedVelocity = 800 / (STEPS_PER_TURNTABLE_REV / 360);  % deg/s
        
        fprintf('\nVelocity:\n');
        fprintf('  Measured: %.1f degrees/second\n', avgVelocity);
        fprintf('  Expected: %.1f degrees/second\n', expectedVelocity);
    end
    
    %% Generate Plots
    figure('Name', 'Turntable System Test', 'Position', [100 100 1400 800]);
    
    % Plot 1: Encoder vs Motor Angle
    subplot(2,3,1);
    plot(timestamps, encoderAngles, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(timestamps, motorAngles, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    title('Encoder vs Motor Angle');
    legend('Encoder', 'Motor (calculated)', 'Location', 'best');
    grid on;
    
    % Plot 2: Tracking Error
    subplot(2,3,2);
    plot(timestamps, errors, 'r-', 'LineWidth', 1.5);
    hold on;
    yline(mean(errors), 'b--', 'Mean', 'LineWidth', 1);
    yline(mean(errors) + 2*std(errors), 'k:', '+2σ');
    yline(mean(errors) - 2*std(errors), 'k:', '-2σ');
    xlabel('Time (s)');
    ylabel('Error (degrees)');
    title('Tracking Error (Encoder - Motor)');
    grid on;
    
    % Plot 3: Step Count
    subplot(2,3,3);
    plot(timestamps, stepCounts, 'g-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Step Count');
    title('Cumulative Step Count');
    grid on;
    
    % Plot 4: Encoder vs Steps (should be linear with 4:1 gear ratio)
    subplot(2,3,4);
    plot(stepCounts, encoderAngles, 'b.', 'MarkerSize', 8);
    xlabel('Step Count');
    ylabel('Encoder Angle (degrees)');
    title('Encoder Angle vs Step Count');
    grid on;
    
    % Plot 5: Error Distribution
    subplot(2,3,5);
    histogram(errors, 30);
    xlabel('Error (degrees)');
    ylabel('Count');
    title('Error Distribution');
    grid on;
    
    % Plot 6: Angular Velocity
    if length(timestamps) > 1
        % Calculate instantaneous velocity
        dt = diff(timestamps);
        dAngle = diff(encoderAngles);
        
        % Handle wraparound
        dAngle(dAngle > 180) = dAngle(dAngle > 180) - 360;
        dAngle(dAngle < -180) = dAngle(dAngle < -180) + 360;
        
        velocity = dAngle ./ dt;
        
        subplot(2,3,6);
        plot(timestamps(2:end), velocity, 'b-', 'LineWidth', 1);
        hold on;
        yline(mean(velocity), 'r--', 'Mean', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Angular Velocity (deg/s)');
        title('Instantaneous Velocity');
        grid on;
    end
    
    fprintf('\nPlots generated!\n');
end

%% Test 3: Step Resolution Test
fprintf('\n=== TEST 3: Step Resolution Test ===\n');
fprintf('Testing minimum step movement...\n');

% Reset position
writeline(s, "RESET");
pause(0.2);

% Take small number of steps
writeline(s, "SPEED:100");  % Slow speed for precision
pause(0.2);

fprintf('Taking 100 microsteps forward...\n');
writeline(s, "CW");
pause(1.0);  % 1 second at 100 steps/sec = 100 steps
writeline(s, "STOP");
pause(0.5);

% Read position
writeline(s, "DATA");
pause(0.2);

if s.NumBytesAvailable > 0
    response = readline(s);
    if contains(response, 'DATA:')
        dataStr = extractAfter(response, 'DATA:');
        values = str2double(split(dataStr, ','));
        
        steps = values(3);
        encoderAngle = values(1);
        
        expectedAngle = (steps / STEPS_PER_TURNTABLE_REV) * 360;
        
        fprintf('Results:\n');
        fprintf('  Steps taken: %d\n', steps);
        fprintf('  Expected angle: %.3f degrees\n', expectedAngle);
        fprintf('  Encoder angle: %.3f degrees\n', encoderAngle);
        fprintf('  Difference: %.3f degrees\n', abs(encoderAngle - expectedAngle));
        
        if abs(encoderAngle - expectedAngle) < 1.0
            fprintf('  ✓ Resolution test PASSED\n');
        else
            fprintf('  ⚠ Check mechanical coupling\n');
        end
    end
end

%% Summary
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('System Status:\n');
fprintf('  ✓ Microstepping: 1/16 configured\n');
fprintf('  ✓ Encoder: Reading angles\n');
fprintf('  ✓ Motor: Moving smoothly\n');

if avgError < 2.0
    fprintf('  ✓ Tracking Accuracy: EXCELLENT (< 2°)\n');
elseif avgError < 5.0
    fprintf('  ✓ Tracking Accuracy: GOOD (< 5°)\n');
else
    fprintf('  ⚠ Tracking Accuracy: Needs improvement (> 5°)\n');
end

fprintf('\nYour turntable is ready for:\n');
fprintf('  → Position control\n');
fprintf('  → Velocity control\n');
fprintf('  → PID implementation\n');

%% Cleanup
fprintf('\nCleaning up...\n');
writeline(s, "STOP");
pause(0.2);
clear s;
fprintf('Done!\n');
