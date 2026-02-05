%% AS5600 Encoder Multi-Turn Tracker (handles 4:1 belt rollover)
clear; clc; close all;

% Raspberry Pi connection
if ~exist('r','var') || ~isvalid(r)
    r = raspi('192.168.1.123', 'macs', 'macsmacs');
    disp('Connected to Raspberry Pi');
else
    disp('Reusing existing Raspberry Pi connection');
end

% Setup I2C for AS5600
i2cBus = i2cdev(r, 'i2c-1', 0x36);
r.I2CBusSpeed
% Setup multi-turn tracker
last_angle = readAS5600_angle(i2cBus);
encoder_turn_count = 0;

% Calculate expected rollover: 360° / 4 = 90°
rollover_threshold = 90;

% Create figure window
hFig = figure('Name', 'AS5600 Multi-Turn Monitor (4:1 Fixed)', ...
    'CloseRequestFcn', @closeFig);
disp('Manually rotate the turntable. Close window to exit.');

% Main loop → poll encoder every 200 ms
while ishandle(hFig)
    current_angle = readAS5600_angle(i2cBus);
    
    % Calculate angle difference
    diff = current_angle - last_angle;
    
    % Detect CW rollover (near 90° → 0°)
    if diff < -rollover_threshold / 2
        encoder_turn_count = encoder_turn_count + 1;
    % Detect CCW rollover (near 0° → 90°)
    elseif diff > rollover_threshold / 2
        encoder_turn_count = encoder_turn_count - 1;
    end
    
    % Calculate continuous encoder angle
    absolute_encoder_angle = encoder_turn_count * 360 + current_angle;
    
    % Scale down to turntable angle
    turntable_angle = absolute_encoder_angle / 4;
    
    % Print result
    fprintf('Turntable angle: %.2f° (raw: %.2f°, encoder turns: %d)\n', ...
        turntable_angle, current_angle, encoder_turn_count);
    
    last_angle = current_angle;
    pause(0.2); % 5 Hz read rate
end

% Cleanup on window close
function closeFig(src, ~)
    disp('Exiting multi-turn encoder monitor...');
    delete(src);
end

% AS5600 angle register read function
function angle = readAS5600_angle(i2cBus)
    try
        pause(0.005);
        high = readRegister(i2cBus,14,'uint8');  % 0x0E
        pause(0.005);
        low  = readRegister(i2cBus,15,'uint8');  % 0x0F
        %raw = bitshift(bitand(high,15),8) + low;
        raw = bitor(bitshift((high),8),low);
        angle = mod(double(raw)/4096*360,360);
    catch
        angle = NaN;
        warning('AS5600 I²C read error');
    end
end