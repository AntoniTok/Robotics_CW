clc;
clear all;

lib_name = '';
if strcmp(computer, 'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h',    ...
        'addheader', 'packet_handler.h',  ...
        'addheader', 'group_sync_write.h',...
        'addheader', 'group_sync_read.h');
end

%% ---- Addresses & Lengths ---- %%
ADDR_PRO_TORQUE_ENABLE    = 64;
ADDR_PRO_GOAL_POSITION    = 116;
ADDR_PRO_PRESENT_POSITION = 132;
ADDR_HARDWARE_ERROR       = 70;
PROFILE_VELOCITY          = 112;

LEN_GOAL_POSITION    = 4;
LEN_PRESENT_POSITION = 4;

%% ---- Settings ---- %%
PROTOCOL_VERSION = 2.0;
ID_BASE     = 11;
ID_SHOULDER = 12;
ID_ELBOW    = 13;
ID_WRIST    = 14;
ID_GRIPPER  = 15;
IDs         = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER];

BAUDRATE   = 1000000;
DEVICENAME = 'COM7';

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;
COMM_SUCCESS   = 0;

%% ---- Velocity & Acceleration Limits (used for wait time estimate only) ---- %%
max_joint_vel = [
    deg2rad(90);
    deg2rad(60);
    deg2rad(60);
    deg2rad(90);
    deg2rad(120)
];

max_joint_accel = [
    deg2rad(180);
    deg2rad(120);
    deg2rad(120);
    deg2rad(180);
    deg2rad(240)
];

%% ---- Home Position ---- %%
home_rad = deg2rad([180; 180; 180; 180; 180]);

%% ---- Open Port ---- %%
port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    unloadlibrary(lib_name);
    fprintf('Failed to open port\n');
    return;
end
if ~setBaudRate(port_num, BAUDRATE)
    unloadlibrary(lib_name);
    fprintf('Failed to set baudrate\n');
    return;
end
fprintf('Port open\n');

%% ---- Reboot All Motors to Clear Any Latched Hardware Errors ---- %%
fprintf('\nRebooting all motors to clear hardware errors...\n');
for k = 1:length(IDs)
    reboot(port_num, PROTOCOL_VERSION, IDs(k));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('  ID %d reboot failed: %s\n', IDs(k), getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    else
        fprintf('  ID %d rebooted OK\n', IDs(k));
    end
end
pause(0.5);

%% ---- Check Hardware Errors Post-Reboot ---- %%
fprintf('\nChecking hardware error registers...\n');
all_clear = true;
for k = 1:length(IDs)
    err = read1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_HARDWARE_ERROR);
    if err ~= 0
        fprintf('  ID %d HARDWARE ERROR 0x%02X: ', IDs(k), err);
        if bitand(err, 1),  fprintf('InputVoltage ');  end
        if bitand(err, 4),  fprintf('Overheat ');      end
        if bitand(err, 8),  fprintf('MotorEncoder ');  end
        if bitand(err, 16), fprintf('ElecShock ');     end
        if bitand(err, 32), fprintf('Overload ');      end
        fprintf('\n');
        all_clear = false;
    else
        fprintf('  ID %d: OK\n', IDs(k));
    end
end

if ~all_clear
    fprintf('\nWARNING: Hardware errors still present after reboot.\n');
    fprintf('Check for mechanical binding or wiring issues.\n');
    fprintf('Press Enter to attempt motion anyway, or Ctrl+C to abort...\n');
    input('');
end

%% ---- Enable Torque ---- %%
fprintf('\nEnabling torque...\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error       = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('  ID %d torque enable failed: %s\n', IDs(k), getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('  ID %d torque error: %s\n', IDs(k), getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('  ID %d torque ON\n', IDs(k));
    end
end

%% ---- Read Current Positions via Sync Read ---- %%
fprintf('\nReading current joint positions...\n');

groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);

for k = 1:length(IDs)
    result = groupSyncReadAddParam(groupread_num, IDs(k));
    if ~result
        fprintf('groupSyncRead addparam failed for ID %d\n', IDs(k));
        return;
    end
end

groupSyncReadTxRxPacket(groupread_num);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('Sync read failed: %s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end

start_rad = zeros(5, 1);
for k = 1:length(IDs)
    available = groupSyncReadIsAvailable(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if ~available
        fprintf('Sync read data not available for ID %d\n', IDs(k));
        return;
    end
    raw          = groupSyncReadGetData(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
    deg          = tick2deg(typecast(uint32(raw), 'int32'));
    start_rad(k) = deg2rad(deg);
    fprintf('  Joint %d (ID %d): %.1f deg\n', k, IDs(k), deg);
end

%% ---- Compute Shortest-Path Target and Estimated Move Duration ---- %%
% wrap_to_pi ensures each joint takes the shortest angular path to home.
home_rad_adj = start_rad + wrap_to_pi(home_rad - start_rad);

% segment_time gives us how long the move should take under our vel/accel
% limits. We use this purely to know how long to wait before checking done.
T_seg = segment_time(start_rad, home_rad_adj, max_joint_vel, max_joint_accel);
fprintf('\nMoving to home [180,180,180,180,180] deg\n');
fprintf('Estimated duration: %.2f s\n\n', T_seg);

%% ---- Set Profile Velocity Once ---- %%
% 1 tick = 0.229 RPM. 100 ticks ~ 23 RPM ~ 138 deg/s. Increase once stable.
SAFE_PROFILE_VEL = 100;

fprintf('Setting profile velocity...\n');
for k = 1:length(IDs)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), PROFILE_VELOCITY, SAFE_PROFILE_VEL);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('  ID %d profile vel failed: %s\n', IDs(k), getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    else
        fprintf('  ID %d profile vel set to %d ticks\n', IDs(k), SAFE_PROFILE_VEL);
    end
end

%% ---- Send Goal Positions (single sync write) ---- %%
% The motor's onboard profile generator handles smooth motion from here.
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);

for k = 1:length(IDs)
    pos_tick = deg2tick(rad2deg(home_rad_adj(k)));
    result_p = groupSyncWriteAddParam(groupwrite_pos, IDs(k), typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
    if ~result_p
        fprintf('groupSyncWriteAddParam failed for ID %d\n', IDs(k));
    end
end

groupSyncWriteTxPacket(groupwrite_pos);
fprintf('Goal positions sent. Waiting for motion to complete...\n');

%% ---- Wait for Motion to Complete ---- %%
% Poll present positions every 0.1s until all joints are within 1 deg of
% target, or until T_seg * 1.5 seconds have elapsed (timeout).
POSITION_TOLERANCE = deg2rad(1.0);
POLL_INTERVAL      = 0.1;
timeout            = T_seg * 1.5;
t_start            = tic;

while toc(t_start) < timeout
    pause(POLL_INTERVAL);

    % Read all present positions
    groupSyncReadTxRxPacket(groupread_num);
    all_arrived = true;
    for k = 1:length(IDs)
        available = groupSyncReadIsAvailable(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if ~available, all_arrived = false; break; end
        raw     = groupSyncReadGetData(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
        cur_rad = deg2rad(tick2deg(typecast(uint32(raw), 'int32')));
        if abs(wrap_to_pi(cur_rad - home_rad_adj(k))) > POSITION_TOLERANCE
            all_arrived = false;
            break;
        end
    end

    if all_arrived
        fprintf('All joints at home position.\n');
        break;
    end
end

if toc(t_start) >= timeout
    fprintf('WARNING: Motion timed out. Check for mechanical issues.\n');
end

fprintf('\nPress Enter to release torque and shut down...\n');
input('');

%% ---- Disable Torque and Close ---- %%
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque disabled\n');

closePort(port_num);
fprintf('Port closed\n');
unloadlibrary(lib_name);


%% ---- Helper Functions ---- %%

function ticks = deg2tick(degrees)
    ticks = round(degrees * (4096 / 360));
    ticks = max(0, min(4095, ticks));
end

function degrees = tick2deg(ticks)
    degrees = double(ticks) * (360 / 4096);
end