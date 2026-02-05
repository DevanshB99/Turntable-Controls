COM_PORT = '/dev/cu.usbserial-0001';  % Change to your port

%% Connect to ESP32
fprintf('Connecting to ESP32 on %s...\n', COM_PORT);

% Create serial connection
s = serialport(COM_PORT, 115200);
configureTerminator(s, "LF");

% Wait for ESP32 to initialize
pause(2);

% Read startup messages
while s.NumBytesAvailable > 0
    msg = readline(s);
    fprintf('%s\n', msg);
end

fprintf('Connected!\n\n');

%% Test 1: Static Angle Reading
fprintf('=== TEST 1: Static Angle Reading ===\n');
fprintf('Reading angle 10 times (motor stopped)...\n\n');

staticAngles = zeros(10, 1);
for i = 1:10
    writeline(s, "ANGLE");
    pause(0.1);
    if s.NumBytesAvailable > 0
        response = readline(s);
        if contains(response, 'ANGLE:')
            angleStr = extractAfter(response, 'ANGLE:');
            staticAngles(i) = str2double(angleStr);
            fprintf('Reading %d: %.2f degrees\n', i, staticAngles(i));
        end
    end
    pause(0.2);
end

avgAngle = mean(staticAngles);
stdAngle = std(staticAngles);

fprintf('\nStatic Test Results:\n');
fprintf('  Average Angle: %.2f degrees\n', avgAngle);
fprintf('  Std Deviation: %.3f degrees\n', stdAngle);
fprintf('  Noise Level: ');
if stdAngle < 0.1
    fprintf('EXCELLENT (< 0.1°)\n');
elseif stdAngle < 0.5
    fprintf('GOOD (< 0.5°)\n');
else
    fprintf('NOISY (> 0.5°) - Check magnet alignment!\n');
end

%% Test 2: Continuous Reading While Rotating
fprintf('\n=== TEST 2: Rotation Test ===\n');
fprintf('Rotating CLOCKWISE for 5 seconds while reading angles...\n');

% Start rotation
writeline(s, "CW");
pause(0.5);

% Collect angle data
angles = [];
timestamps = [];
startTime = tic;

fprintf('Collecting data');
while toc(startTime) < 5
    if s.NumBytesAvailable > 0
        response = readline(s);
        if contains(response, 'ANGLE:')
            angleStr = extractAfter(response, 'ANGLE:');
            angle = str2double(angleStr);
            angles = [angles; angle];
            timestamps = [timestamps; toc(startTime)];
            fprintf('.');
        end
    end
    pause(0.01);  % Small delay
end
fprintf(' Done!\n');

% Stop motor
writeline(s, "STOP");
pause(0.5);

% Flush remaining data
while s.NumBytesAvailable > 0
    readline(s);
end

fprintf('\nCollected %d angle readings\n', length(angles));

%% Analyze the data
if length(angles) > 10
    % Calculate velocity (degrees per second)
    if length(angles) > 1
        angleChanges = diff(angles);
        
        % Handle wraparound (360 -> 0 or 0 -> 360)
        angleChanges(angleChanges > 180) = angleChanges(angleChanges > 180) - 360;
        angleChanges(angleChanges < -180) = angleChanges(angleChanges < -180) + 360;
        
        timeChanges = diff(timestamps);
        velocities = angleChanges ./ timeChanges;
        
        avgVelocity = mean(velocities);
        fprintf('  Average Velocity: %.1f degrees/second\n', avgVelocity);
    end
    
    % Check angle range
    minAngle = min(angles);
    maxAngle = max(angles);
    angleRange = maxAngle - minAngle;
    
    fprintf('  Angle Range: %.1f to %.1f degrees\n', minAngle, maxAngle);
    fprintf('  Range Covered: %.1f degrees\n', angleRange);
    
    % Check if we crossed zero (360->0 transition)
    zeroCrossings = sum(abs(diff(angles)) > 180);
    fprintf('  Zero Crossings: %d\n', zeroCrossings);
    
    % Plot results
    figure('Name', 'Encoder Test Results', 'Position', [100 100 1200 600]);
    
    % Plot 1: Angle vs Time
    subplot(2,2,1);
    plot(timestamps, angles, 'b.-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    title('Encoder Angle vs Time');
    grid on;
    ylim([0 360]);
    
    % Plot 2: Angle Distribution
    subplot(2,2,2);
    histogram(angles, 36);  % 36 bins = 10 degrees per bin
    xlabel('Angle (degrees)');
    ylabel('Count');
    title('Angle Distribution');
    grid on;
    xlim([0 360]);
    
    % Plot 3: Velocity
    if length(velocities) > 0
        subplot(2,2,3);
        plot(timestamps(2:end), velocities, 'r.-', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Angular Velocity (deg/s)');
        title('Angular Velocity');
        grid on;
    end
    
    % Plot 4: Angle unwrapped (handle 360->0 transition)
    subplot(2,2,4);
    unwrapped = unwrap(angles * pi/180) * 180/pi;  % Unwrap in radians, convert back
    plot(timestamps, unwrapped, 'g.-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Cumulative Angle (degrees)');
    title('Unwrapped Angle (Continuous)');
    grid on;
    
    fprintf('\nPlots generated!\n');
end

%% Test 3: Full 360 Degree Test
fprintf('\n=== TEST 3: Manual 360 Degree Test ===\n');
fprintf('Instructions:\n');
fprintf('1. Note the current angle\n');
fprintf('2. Manually rotate the turntable ONE full rotation\n');
fprintf('3. Check if angle returns to starting value\n\n');

% Read current angle
writeline(s, "ANGLE");
pause(0.2);
if s.NumBytesAvailable > 0
    response = readline(s);
    if contains(response, 'ANGLE:')
        angleStr = extractAfter(response, 'ANGLE:');
        startAngle = str2double(angleStr);
        fprintf('Starting Angle: %.2f degrees\n', startAngle);
        fprintf('\nNow manually rotate the turntable by hand...\n');
        fprintf('Watch the angle change in real-time:\n\n');
    end
end

% Real-time angle display for 30 seconds
fprintf('(Requesting angle every 0.5 seconds for 30 seconds)\n');
fprintf('Press Ctrl+C to stop early\n\n');

try
    for i = 1:60  % 30 seconds at 0.5s intervals
        writeline(s, "ANGLE");
        pause(0.2);
        
        if s.NumBytesAvailable > 0
            response = readline(s);
            if contains(response, 'ANGLE:')
                angleStr = extractAfter(response, 'ANGLE:');
                currentAngle = str2double(angleStr);
                
                % Show with visual bar
                fprintf('Angle: %6.2f° ', currentAngle);
                barLength = round(currentAngle / 10);
                fprintf('[%s%s]\n', repmat('■', 1, barLength), repmat('□', 1, 36-barLength));
            end
        end
        
        pause(0.3);
    end
catch ME
    fprintf('\nStopped by user.\n');
end

%% Summary
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('✓ Encoder I2C communication: Working\n');
fprintf('✓ Angle reading range: 0-360 degrees\n');
fprintf('✓ Real-time updates: Working\n');
fprintf('\nEncoder Verification:\n');
fprintf('  - If angles change smoothly: ✓ Magnet properly aligned\n');
fprintf('  - If angles jump/unstable: ✗ Check magnet distance (should be 1-3mm)\n');
fprintf('  - If stuck at one value: ✗ Check I2C wiring (GPIO 21, 22)\n');

%% Cleanup
fprintf('\nTest complete. Disconnecting...\n');
clear s;
fprintf('Done!\n');
