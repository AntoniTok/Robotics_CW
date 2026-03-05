% test_rotate.m
% Setup Dynamixel, then call rotate_cube_in_place

clear; clc; close all;

%% Dynamixel Setup (same as your main script)
lib_name = '';
if strcmp(computer, 'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [~, ~] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
        'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

IDs        = [11, 12, 13, 14, 15];
BAUDRATE   = 1000000;
DEVICENAME = 'COM8';  % Adjust if needed
PROTOCOL_VERSION = 2.0;

port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num),   error('Failed to open port'); end
if ~setBaudRate(port_num, BAUDRATE), error('Failed to set baudrate'); end
fprintf('Port open!\n');
pause(0.5);

% Enable torque + set profiles
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), 64, 1);   % torque on
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), 108, 30);  % accel
    if IDs(k) == 15
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), 112, 200); % gripper speed
    else
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), 112, 150); % arm speed
    end
end
fprintf('Robot ready.\n');

%% ---- Call the rotation function ----
%  Adjust grid position and number of rotations here:
cube_gx = 16;
cube_gy = 10;
k = 1;  % number of 90-degree rotations

rotate_cube_in_place(k, cube_gx, cube_gy, port_num, PROTOCOL_VERSION, IDs);

%% Wait before shutting down
fprintf('\nRotation complete. Press ENTER in the command window to disable torque and shut down...\n');
input('', 's');

%% Cleanup
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), 64, 0);  % torque off
end
closePort(port_num);
unloadlibrary(lib_name);
fprintf('Done. Robot shut down.\n');