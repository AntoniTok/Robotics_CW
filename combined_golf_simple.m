% task2a_swing.m
% Swing Motion: Neutral → Close Gripper → 45° Pitch → Position → Swing Base 100°
% Phases:
%   1. Move to neutral/home position
%   2. Close gripper fully and hold
%   3. Tilt end-effector to -45° pitch (downward)
%   4. Move to target position (x=0, y=0.075, z=0.12) at -45° pitch
%   5. Swing base (theta1) +100° as fast as possible (vel=900, accel=80)

clear; clc; close all;

%% =========================================================
%% 1. DYNAMIXEL SETUP
%% =========================================================
fprintf('Initializing Dynamixel SDK...\n');
lib_name = '';
if strcmp(computer, 'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
        'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

% Control table addresses
ADDR_PRO_TORQUE_ENABLE        = 64;
ADDR_PRO_PROFILE_ACCELERATION = 108;
ADDR_PRO_PROFILE_VELOCITY     = 112;
ADDR_PRO_GOAL_POSITION        = 116;
LEN_GOAL_POSITION             = 4;
PROTOCOL_VERSION              = 2.0;

% Motor IDs: Base(11), Shoulder(12), Elbow(13), Wrist(14), Gripper(15)
IDs        = [11, 12, 13, 14, 15];
ID_GRIPPER = 15;
ID_BASE    = 11;
BAUDRATE   = 1000000;
DEVICENAME = 'COM7';   % <<< ADJUST PORT IF NEEDED

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

% Normal motion profile (arm movement phases 1–4)
NORMAL_VEL   = 150;
NORMAL_ACCEL = 30;

% Gripper profile (snappy)
GRIPPER_VEL   = 200;
GRIPPER_ACCEL = 30;

% Swing profile (phase 5 — base motor only)
SWING_VEL   = 900;
SWING_ACCEL = 80;

% Open port
port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    fprintf('Failed to open port %s.\n', DEVICENAME);
    unloadlibrary(lib_name); return;
end
if ~setBaudRate(port_num, BAUDRATE)
    fprintf('Failed to set baudrate.\n');
    closePort(port_num); unloadlibrary(lib_name); return;
end
fprintf('Port open!\n');
pause(0.5);

% Enable torque and set normal profiles on all motors
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, NORMAL_ACCEL);
    if IDs(k) == ID_GRIPPER
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, GRIPPER_VEL);
    else
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, NORMAL_VEL);
    end
end
fprintf('Torque & profiles set (Normal mode).\n');

%% =========================================================
%% 2. ROBOT PARAMETERS (from original script)
%% =========================================================
d1    = 0.077;
a2    = sqrt(0.128^2 + 0.024^2);
delta = atan2(0.024, 0.128);
a3    = 0.124;
a4    = 0.126;
L_finger    = 0.02;
L_tip_total = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + shift_q2, deg2rad(90) + shift_q2;
    deg2rad(-75)  + shift_q3, deg2rad(85) + shift_q3;
    deg2rad(-135),            deg2rad(135)
    ];

MOTOR_11_OFFSET = deg2rad(2.2);

%% =========================================================
%% 3. GRIPPER SETTINGS
%% =========================================================
GRIPPER_OPEN       = deg2rad(-45);  % Open
GRIPPER_CLOSE_FULL = deg2rad(45);   % Fully closed (adjust if hardware limit differs)

current_gripper = GRIPPER_OPEN;

%% =========================================================
%% 4. VISUALIZATION SETUP
%% =========================================================
fig = figure('Name','Swing Motion','Color','w','Position',[100 100 900 700]);
view(45, 30); axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis([-0.4 0.4 -0.4 0.4 0 0.5]);

%% =========================================================
%% PHASE 1: MOVE TO NEUTRAL HOME POSITION
%% =========================================================
fprintf('\n--- PHASE 1: Moving to Neutral Home ---\n');

home_x = 0.20; home_y = 0.0; home_z = 0.20; home_pitch = 0;
[q1h, q2h, q3h, q4h, valid] = inverse_kinematics( ...
    home_x, home_y, home_z, home_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if ~valid
    error('Home position unreachable via IK!');
end

current_q = [q1h, q2h, q3h, q4h];
phys = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys);

plot_robot_simple(current_q, d1, a2, a3, L_tip_total, delta);
drawnow;
fprintf('  Waiting to reach home...\n');
pause(2.5);

%% =========================================================
%% PHASE 2: CLOSE GRIPPER FULLY (hold for rest of motion)
%% =========================================================
fprintf('\n--- PHASE 2: Closing Gripper Fully ---\n');
current_gripper = GRIPPER_CLOSE_FULL;
phys = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys);
fprintf('  Gripper closing (%.1f deg)...\n', rad2deg(current_gripper));
pause(1.0);  % Let gripper fully close before moving

%% =========================================================
%% PHASE 3: TRANSITION TO -45° PITCH (at home XY, same Z)
%% =========================================================
fprintf('\n--- PHASE 3: Tilting to -45 deg pitch ---\n');

target_pitch = deg2rad(-45);
num_steps    = 30;

% Interpolate only the pitch from 0 → -45°, keeping XY/Z at home
start_pos   = forward_kinematics(current_q, d1, a2, a3, L_tip_total, delta);
start_pitch = current_q(2) + delta + current_q(3) + current_q(4);

traj_pitch = linspace(start_pitch, target_pitch, num_steps);

for t = 1:num_steps
    [q1t, q2t, q3t, q4t, vt] = inverse_kinematics( ...
        start_pos(1), start_pos(2), start_pos(3), traj_pitch(t), ...
        d1, a2, a3, L_tip_total, delta, joint_limits);
    if vt
        current_q = [q1t, q2t, q3t, q4t];
        phys = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys);
        plot_robot_simple(current_q, d1, a2, a3, L_tip_total, delta);
        drawnow; pause(0.01);
    end
end
fprintf('  Pitch at -45 deg. Settling...\n');
pause(0.5);

%% =========================================================
%% PHASE 4: MOVE TO SWING POSITION (x=0, y=0.075, z=0.12)
%% =========================================================
fprintf('\n--- PHASE 4: Moving to Swing Position ---\n');

% Target in world frame
swing_x = 0.000;
swing_y = 0.075;
swing_z = 0.120;

% NOTE: theta1 here = atan2(0.075, 0) = 90 deg, so arm points in +Y
[q1_sw, q2_sw, q3_sw, q4_sw, valid_sw] = inverse_kinematics( ...
    swing_x, swing_y, swing_z, target_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if ~valid_sw
    fprintf('  WARNING: Swing position unreachable — check IK. Trying relaxed pitch...\n');
    % Try slightly different pitches as fallback
    for fallback_pitch = deg2rad(-50:-5:-70)
        [q1_sw, q2_sw, q3_sw, q4_sw, valid_sw] = inverse_kinematics( ...
            swing_x, swing_y, swing_z, fallback_pitch, ...
            d1, a2, a3, L_tip_total, delta, joint_limits);
        if valid_sw
            target_pitch = fallback_pitch;
            fprintf('  Using fallback pitch: %.1f deg\n', rad2deg(target_pitch));
            break;
        end
    end
    if ~valid_sw
        error('Swing position completely unreachable!');
    end
end

% Linear trajectory from current pos → swing position
cur_pos    = forward_kinematics(current_q, d1, a2, a3, L_tip_total, delta);
cur_pitch  = current_q(2) + delta + current_q(3) + current_q(4);
num_steps  = 40;

traj_x     = linspace(cur_pos(1), swing_x, num_steps);
traj_y     = linspace(cur_pos(2), swing_y, num_steps);
traj_z     = linspace(cur_pos(3), swing_z, num_steps);
traj_pitch = linspace(cur_pitch,  target_pitch, num_steps);

for t = 1:num_steps
    [q1t, q2t, q3t, q4t, vt] = inverse_kinematics( ...
        traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
        d1, a2, a3, L_tip_total, delta, joint_limits);
    if vt
        current_q = [q1t, q2t, q3t, q4t];
        phys = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys);
        plot_robot_simple(current_q, d1, a2, a3, L_tip_total, delta);
        drawnow; pause(0.01);
    end
end
fprintf('  At swing position. Settling...\n');
pause(0.8);

%% =========================================================
%% PHASE 5: SWING — BASE ROTATES +100° AT MAXIMUM SPEED
%% =========================================================
fprintf('\n--- PHASE 5: SWING — Base +100 deg at MAX speed ---\n');

% Boost ONLY the base motor (ID 11) to swing profile
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_ACCELERATION, SWING_ACCEL);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_VELOCITY,     SWING_VEL);
fprintf('  Base motor: vel=%d, accel=%d\n', SWING_VEL, SWING_ACCEL);

% Compute target theta1 (add 100 degrees to current base angle)
% +100 deg = counterclockwise from above (adjust sign if needed)
swing_delta_deg = 100;  % <<< Change sign to -100 to swing in opposite direction
q1_swing_target = current_q(1) + deg2rad(swing_delta_deg);

% Clamp to joint limits
q1_swing_target = max(joint_limits(1,1), min(joint_limits(1,2), q1_swing_target));

fprintf('  Theta1: %.1f deg → %.1f deg (swing of %.1f deg)\n', ...
    rad2deg(current_q(1)), rad2deg(q1_swing_target), swing_delta_deg);

% Build the swing target joint angles (q2,q3,q4 stay fixed)
q_swing = [q1_swing_target, current_q(2), current_q(3), current_q(4)];

% Send single command — Dynamixel profiles handle the acceleration ramp
phys_swing = sim_to_phys_angles(q_swing, current_gripper, delta, offset_classmate);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_swing);

% Animate the swing visually while motor moves
num_vis_steps = 50;
for t = 1:num_vis_steps
    q1_vis = current_q(1) + (q1_swing_target - current_q(1)) * (t / num_vis_steps);
    q_vis  = [q1_vis, current_q(2), current_q(3), current_q(4)];
    plot_robot_simple(q_vis, d1, a2, a3, L_tip_total, delta);
    drawnow; pause(0.02);
end

current_q = q_swing;
fprintf('  Swing complete. Waiting for arm to settle...\n');
pause(1.5);

%% =========================================================
%% PHASE 6: RETURN TO END POSITION (from original script)
%% =========================================================
fprintf('\n--- PHASE 6: Returning to End Position ---\n');

% Restore normal velocity/acceleration on base motor before return move
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_ACCELERATION, NORMAL_ACCEL);
write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_VELOCITY,     NORMAL_VEL);
fprintf('  Base motor restored to normal profile (vel=%d, accel=%d).\n', NORMAL_VEL, NORMAL_ACCEL);

end_x = 0.15; end_y = 0.0; end_z = 0.15; end_pitch = 0;
[q1e, q2e, q3e, q4e, valid_end] = inverse_kinematics( ...
    end_x, end_y, end_z, end_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if ~valid_end
    fprintf('  WARNING: End position unreachable — skipping return.\n');
else
    % Linear trajectory from post-swing position back to end position
    cur_pos   = forward_kinematics(current_q, d1, a2, a3, L_tip_total, delta);
    cur_pitch = current_q(2) + delta + current_q(3) + current_q(4);
    num_steps = 40;

    traj_x     = linspace(cur_pos(1), end_x,      num_steps);
    traj_y     = linspace(cur_pos(2), end_y,       num_steps);
    traj_z     = linspace(cur_pos(3), end_z,       num_steps);
    traj_pitch = linspace(cur_pitch,  end_pitch,   num_steps);

    for t = 1:num_steps
        [q1t, q2t, q3t, q4t, vt] = inverse_kinematics( ...
            traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
            d1, a2, a3, L_tip_total, delta, joint_limits);
        if vt
            current_q = [q1t, q2t, q3t, q4t];
            phys = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys);
            plot_robot_simple(current_q, d1, a2, a3, L_tip_total, delta);
            drawnow; pause(0.01);
        end
    end
    fprintf('  At end position. Waiting to settle...\n');
    pause(2.0);
end

%% =========================================================
%% CLEANUP
%% =========================================================
fprintf('\n--- Shutting Down ---\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque DISABLED.\n');
closePort(port_num);
fprintf('Port Closed.\n');
unloadlibrary(lib_name);


%% =========================================================
%% HELPER FUNCTIONS
%% =========================================================

function phys_angles = sim_to_phys_angles(sim_q, gripper_q, delta, offset_classmate)
% Convert DH simulation angles to physical Dynamixel motor frame
q1 = sim_q(1);
q2 = -(sim_q(2) + delta - offset_classmate);
q3 = -(sim_q(3) + offset_classmate);
q4 = -sim_q(4);
q5 = gripper_q;
phys_angles = [q1; q2; q3; q4; q5] + deg2rad(180);
end

function send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles)
ADDR_PRO_GOAL_POSITION = 116;
LEN_GOAL_POSITION      = 4;
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ...
    ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);
for k = 1:5
    deg_val  = rad2deg(phys_angles(k));
    pos_tick = round(deg_val * (4096 / 360));
    pos_tick = max(0, min(4095, pos_tick));
    groupSyncWriteAddParam(groupwrite_pos, IDs(k), ...
        typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
end
groupSyncWriteTxPacket(groupwrite_pos);
end

function pos = forward_kinematics(q, d1, a2, a3, L4, delta)
T01 = dh_matrix(q(1),          d1, 0,  pi/2);
T12 = dh_matrix(q(2) + delta,   0, a2, 0);
T23 = dh_matrix(q(3),           0, a3, 0);
T34 = dh_matrix(q(4),           0, L4, 0);
pos = (T01*T12*T23*T34) * [0;0;0;1];
pos = pos(1:3);
end

function [t1,t2,t3,t4,isValid] = inverse_kinematics(x,y,z,phi,d1,a2,a3,L4,delta,limits)
isValid = true;
t1      = atan2(y, x);
r_target = sqrt(x^2 + y^2);
z_target = z - d1;
r_w      = r_target - L4*cos(phi);
z_w      = z_target - L4*sin(phi);
D_sq     = r_w^2 + z_w^2;
cos_t3   = (D_sq - a2^2 - a3^2) / (2*a2*a3);
if abs(cos_t3) > 1
    isValid = false; t1=0;t2=0;t3=0;t4=0; return;
end
sin_t3 = -sqrt(1 - cos_t3^2);
t3     = atan2(sin_t3, cos_t3);
alpha  = atan2(z_w, r_w);
cos_b  = (a2^2 + D_sq - a3^2) / (2*a2*sqrt(D_sq));
if abs(cos_b) > 1, cos_b = sign(cos_b); end
beta   = acos(cos_b);
angle_link2 = alpha + beta;
t2     = angle_link2 - delta;
t4     = phi - (angle_link2 + t3);
if nargin >= 10 && ~isempty(limits)
    if t1 < limits(1,1) || t1 > limits(1,2) || ...
       t2 < limits(2,1) || t2 > limits(2,2) || ...
       t3 < limits(3,1) || t3 > limits(3,2) || ...
       t4 < limits(4,1) || t4 > limits(4,2)
        isValid = false;
    end
end
end

function plot_robot_simple(q, d1, a2, a3, L4, delta)
cla; hold on; grid on; axis equal;
axis([-0.4 0.4 -0.4 0.4 0 0.5]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
T01 = dh_matrix(q(1),          d1, 0,  pi/2);
T12 = dh_matrix(q(2) + delta,   0, a2, 0);
T23 = dh_matrix(q(3),           0, a3, 0);
T34 = dh_matrix(q(4),           0, L4, 0);
T02 = T01*T12; T03 = T02*T23; T04 = T03*T34;
pts = [[0;0;0], T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4)];
plot3(pts(1,:), pts(2,:), pts(3,:), '-k', 'LineWidth', 3, ...
    'Marker','o', 'MarkerFaceColor','y', 'MarkerSize', 7);
end

function T = dh_matrix(theta, d, a, alpha)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,           sin(alpha),             cos(alpha),            d;
     0,           0,                      0,                     1];
end