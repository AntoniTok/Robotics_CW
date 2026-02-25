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

%% ---- Velocity & Acceleration Limits ---- %%
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

%% ---- Joint Limits (radians) ---- %%
% Adjust these to match your physical robot's safe range.
% Motors use 0–360° (0–4095 ticks), neutral at 180° (tick 2048).
% These limits are in IK-frame (centred at 0 = neutral).
joint_limits = [
    deg2rad(-150),  deg2rad(150);   % theta1: base
    deg2rad(-90),   deg2rad(90);    % theta2: shoulder
    deg2rad(-150),  deg2rad(150);   % theta3: elbow
    deg2rad(-100),  deg2rad(100);   % theta4: wrist
    0,              0               % theta5: gripper (fixed)
];

%% ---- Get Target End-Effector Position from User ---- %%
fprintf('Enter target end-effector position (metres):\n');
X = input('  X: ');
Y = input('  Y: ');
Z = input('  Z: ');
pitch_desired = input('  Desired end-effector pitch (rad) [default -pi/2]: ');
if isempty(pitch_desired)
    pitch_desired = -pi/2;
end

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

%% ---- Reboot All Motors ---- %%
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

start_rad_motor = zeros(5, 1);   % raw motor angles (0–2pi, 180deg = neutral)
start_rad_ik    = zeros(5, 1);   % IK-frame angles (centred at 0)

for k = 1:length(IDs)
    available = groupSyncReadIsAvailable(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if ~available
        fprintf('Sync read data not available for ID %d\n', IDs(k));
        return;
    end
    raw                  = groupSyncReadGetData(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
    deg_motor            = tick2deg(typecast(uint32(raw), 'int32'));
    start_rad_motor(k)   = deg2rad(deg_motor);
    start_rad_ik(k)      = deg2rad(deg_motor - 180);   % shift: motor 180° → IK 0°
    fprintf('  Joint %d (ID %d): %.1f deg (motor)  /  %.1f deg (IK frame)\n', ...
            k, IDs(k), deg_motor, deg_motor - 180);
end

%% ---- Run Inverse Kinematics ---- %%
fprintf('\nRunning IK for target (%.4f, %.4f, %.4f) m, pitch=%.3f rad...\n', X, Y, Z, pitch_desired);

try
    [t1, t2, t3, t4, t5] = inverse_kinematics_continuous( ...
        X, Y, Z, pitch_desired, start_rad_ik', joint_limits);
catch ME
    fprintf('IK failed: %s\n', ME.message);
    % Clean up before aborting
    for k = 1:length(IDs)
        write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    end
    closePort(port_num);
    unloadlibrary(lib_name);
    return;
end

% IK angles in IK frame → motor frame (add 180°)
target_rad_ik    = [t1; t2; t3; t4; t5];
target_rad_motor = target_rad_ik + deg2rad(180);

fprintf('\nTarget joint angles:\n');
for k = 1:5
    fprintf('  Joint %d: %.1f deg (IK)  →  %.1f deg (motor)\n', ...
            k, rad2deg(target_rad_ik(k)), rad2deg(target_rad_motor(k)));
end

%% ---- Compute Shortest-Path Target and Estimated Duration ---- %%
% Use motor-frame angles for the motion planner.
target_rad_adj = start_rad_motor + wrap_to_pi(target_rad_motor - start_rad_motor);

T_seg = segment_time(start_rad_motor, target_rad_adj, max_joint_vel, max_joint_accel);
fprintf('\nEstimated move duration: %.2f s\n', T_seg);

%% ---- Set Profile Velocity ---- %%
SAFE_PROFILE_VEL = 50;
fprintf('\nSetting profile velocity...\n');
for k = 1:length(IDs)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), PROFILE_VELOCITY, SAFE_PROFILE_VEL);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('  ID %d profile vel failed: %s\n', IDs(k), getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    else
        fprintf('  ID %d profile vel = %d ticks\n', IDs(k), SAFE_PROFILE_VEL);
    end
end

%% ---- Send Goal Positions ---- %%
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);

for k = 1:length(IDs)
    pos_tick = deg2tick(rad2deg(target_rad_adj(k)));
    result_p = groupSyncWriteAddParam(groupwrite_pos, IDs(k), typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
    if ~result_p
        fprintf('groupSyncWriteAddParam failed for ID %d\n', IDs(k));
    end
end

groupSyncWriteTxPacket(groupwrite_pos);
fprintf('Goal positions sent. Waiting for motion to complete...\n');

%% ---- Wait for Motion to Complete ---- %%
POSITION_TOLERANCE = deg2rad(1.0);
POLL_INTERVAL      = 0.1;
timeout            = T_seg * 1.5;
t_start            = tic;

while toc(t_start) < timeout
    pause(POLL_INTERVAL);

    groupSyncReadTxRxPacket(groupread_num);
    all_arrived = true;
    for k = 1:length(IDs)
        available = groupSyncReadIsAvailable(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if ~available, all_arrived = false; break; end
        raw     = groupSyncReadGetData(groupread_num, IDs(k), ADDR_PRO_PRESENT_POSITION, LEN_PRESENT_POSITION);
        cur_rad = deg2rad(tick2deg(typecast(uint32(raw), 'int32')));
        if abs(wrap_to_pi(cur_rad - target_rad_adj(k))) > POSITION_TOLERANCE
            all_arrived = false;
            break;
        end
    end

    if all_arrived
        fprintf('All joints at target position.\n');
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