% task2c.m
% Task 2c: Move the tool through the gates
% Based on task2a_working.m

clear; clc; close all;

%% 1. SETTINGS & DYNAMIXEL SETUP
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

ADDR_PRO_TORQUE_ENABLE    = 64;
ADDR_PRO_PROFILE_ACCELERATION = 108;
ADDR_PRO_PROFILE_VELOCITY     = 112;
ADDR_PRO_GOAL_POSITION    = 116;

LEN_GOAL_POSITION    = 4;
PROTOCOL_VERSION = 2.0;

ID_GRIPPER = 15;
IDs        = [11, 12, 13, 14, 15]; % Base, Shoulder, Elbow, Wrist, Gripper
BAUDRATE   = 1000000;
DEVICENAME = 'COM7'; % Adjust if needed

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;
SAFE_PROFILE_VEL   = 120; % slightly slower for precise gate navigation
GRIPPER_PROFILE_VEL = 200;
SAFE_PROFILE_ACCEL = 30;
global MOTOR_11_OFFSET;
MOTOR_11_OFFSET    = deg2rad(2);

port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    fprintf('Failed to open port %s.\n', DEVICENAME); unloadlibrary(lib_name); return;
end
if ~setBaudRate(port_num, BAUDRATE)
    fprintf('Failed to set baudrate.\n'); closePort(port_num); unloadlibrary(lib_name); return;
end

fprintf('Port open!\n');
pause(0.5);

for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, SAFE_PROFILE_ACCEL);

    if IDs(k) == ID_GRIPPER
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, GRIPPER_PROFILE_VEL);
    else
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, SAFE_PROFILE_VEL);
    end
end
fprintf('Torque, Acceleration & Velocity Profiles ENABLED.\n');

%% 2. GRIPPER SETTINGS
GRIPPER_OPEN  = deg2rad(-45);
GRIPPER_CLOSE = deg2rad(20);
current_gripper = GRIPPER_CLOSE; % Keep closed for passing through gate

%% 3. GRID & SCENE CONFIGURATION
GRID_UNIT = 0.025;
GRID_W    = 17;
GRID_H    = 12;

ROBOT_GX  = 9;
ROBOT_GY  = 3;

% Define the gates
% gates format: [grid_x, grid_y, orientation (1 for X-aligned passage, 2 for Y-aligned passage)]
% Since gates can appear anywhere, these are placeholders for Demo Day.
gates = [
    5, 7, 1; % passing along X axis
    12, 10, 2; % passing along Y axis
    15, 6, 1
    ];

% (No cubes needed for this specific gate traversal task unless stated)
target_cubes = [];
cubes_start = [];
holders = [];

%% 4. ROBOT PARAMETERS
d1 = 0.077;
a2 = sqrt(0.128^2 + 0.024^2);
delta = atan2(0.024, 0.128);
a3 = 0.124;
a4 = 0.126;
L_finger    = 0.025;
L_tip_total = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + shift_q2,  deg2rad(90) + shift_q2;
    deg2rad(-75)  + shift_q3,  deg2rad(85) + shift_q3;
    deg2rad(-135),             deg2rad(135)
    ];

fig = figure('Name','Task 2c: Passing Through Gates','Color','w','Position',[100 100 1200 800]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.3 0.4 -0.4 0.4 0 0.6]);

%% 5. INITIALIZATION
home_x = 0.25; home_y = 0; home_z = 0.20; home_pitch = 0;
[q1,q2,q3,q4,valid] = inverse_kinematics(home_x,home_y,home_z,home_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);

if ~valid
    error('Mathematical home position unreachable!');
end

current_q = [q1, q2, q3, q4];

% Move physical robot to Home initially
phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

plot_scene_gates(current_q, gates, d1,a2,a3,L_tip_total,delta, GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(2); % Wait to reach home physically

%% 6. MAIN GATE PASSAGE LOOP
try
    for i = 1:size(gates, 1)

        g_x = gates(i, 1);
        g_y = gates(i, 2);
        g_ori = gates(i, 3);

        [wx, wy, ~]  = grid_to_world(g_x, g_y, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

        % Must be below line on gate, and not touch gates.
        % Typically, setting Z slightly above 0 with a flat pitch is best.
        z_pass = 0.02; % 2 cm above ground
        hover_z = 0.06; % Hover height for transit between gates

        % Determine approach and exit offsets based on orientation
        pass_dist = 0.08; % distance to start before gate and end after gate
        if g_ori == 1
            % Pass along X axis
            wpt_pre_x = wx - pass_dist; wpt_pre_y = wy;
            wpt_post_x = wx + pass_dist; wpt_post_y = wy;
        else
            % Pass along Y axis
            wpt_pre_x = wx; wpt_pre_y = wy - pass_dist;
            wpt_post_x = wx; wpt_post_y = wy + pass_dist;
        end

        fprintf('Passing Gate %d at Grid(%d,%d)\n', i, g_x, g_y);

        % Try a straight flat pitch first to easily pass under the line
        best_pitch = 0;

        % Waypoints: [x, y, z, pitch]
        waypoints = [
            wpt_pre_x, wpt_pre_y, hover_z, best_pitch; % hover before gate
            wpt_pre_x, wpt_pre_y, z_pass,  best_pitch; % lower down
            wx,        wy,        z_pass,  best_pitch; % drive through center
            wpt_post_x, wpt_post_y, z_pass,  best_pitch; % exit on other side
            wpt_post_x, wpt_post_y, hover_z, best_pitch; % lift back up
            ];

        for wp_idx = 1:size(waypoints, 1)
            target    = waypoints(wp_idx, :);
            goal_x    = target(1); goal_y = target(2); goal_z = target(3);
            goal_pitch = target(4);

            current_pos         = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
            current_pitch_val   = current_q(2) + delta + current_q(3) + current_q(4);

            num_steps = 15; % Smooth steps
            traj_x     = linspace(current_pos(1), goal_x,     num_steps);
            traj_y     = linspace(current_pos(2), goal_y,     num_steps);
            traj_z     = linspace(current_pos(3), goal_z,     num_steps);
            traj_pitch = linspace(current_pitch_val, goal_pitch, num_steps);

            for t = 1:num_steps
                [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics( ...
                    traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
                    d1,a2,a3,L_tip_total,delta,joint_limits);

                if valid_t
                    current_q = [q1_t, q2_t, q3_t, q4_t];

                    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
                    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

                    % Visual Simulation Sync
                    plot_scene_gates(current_q, gates, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
                    drawnow;

                    pause(0.015);
                else
                    warning('Trajectory point unreachable for Gate %d!', i);
                end
            end
        end

    end
catch ME
    fprintf('Program interrupted: %s\n', ME.message);
end

%% 7. END POSITION
end_x = 0.175; end_y = 0; end_z = 0.15; end_pitch = 0;
[q1,q2,q3,q4,valid] = inverse_kinematics(end_x,end_y,end_z,end_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);

if valid
    current_q = [q1, q2, q3, q4];
    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
    plot_scene_gates(current_q, gates, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    pause(2);
end

%% 8. CLEANUP
fprintf('\n--- Shutting Down ---\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque DISABLED.\n');
closePort(port_num);
fprintf('Port Closed.\n');
unloadlibrary(lib_name);


%% --- HELPER FUNCTIONS ---

function phys_angles = sim_to_phys_angles(sim_q, gripper_q, delta, offset_classmate, is_placing)
global MOTOR_11_OFFSET;
if is_placing
    q1 = sim_q(1) + MOTOR_11_OFFSET;
else
    q1 = sim_q(1);
end
q2 = -(sim_q(2) + delta - offset_classmate);
q3 = -(sim_q(3) + offset_classmate);
q4 = -sim_q(4);
q5 = gripper_q;

phys_angles = [q1; q2; q3; q4; q5] + deg2rad(180);
end

function send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles)
ADDR_PRO_GOAL_POSITION = 116;
LEN_GOAL_POSITION = 4;
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);

for k = 1:5
    deg_val = rad2deg(phys_angles(k));
    pos_tick = round(deg_val * (4096 / 360));
    pos_tick = max(0, min(4095, pos_tick));
    groupSyncWriteAddParam(groupwrite_pos, IDs(k), typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
end
groupSyncWriteTxPacket(groupwrite_pos);
end

function [wx, wy, wz] = grid_to_world(gx, gy, gz_scale, unit, r_gx, r_gy)
wx = (gy  - r_gy) * unit;
wy = (r_gx - gx)  * unit;
wz =  gz_scale     * unit;
end

function plot_scene_gates(q, gates, d1,a2,a3,L4,delta,unit,rx,ry)
cla; hold on; grid on; axis equal;
axis([-0.3 0.4 -0.4 0.4 0 0.6]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

% Plot gates as simple 3D arches
for k = 1:size(gates, 1)
    [gx, gy, ~] = grid_to_world(gates(k,1), gates(k,2), 0, unit, rx, ry);

    g_width = 0.10;
    g_height = 0.08;
    g_depth = 0.02;

    ori = gates(k,3);

    if ori == 1
        % Passing X axis, gate faces X, meaning posts span across Y
        p1 = [gx, gy - g_width/2, 0];
        p2 = [gx, gy + g_width/2, 0];
    else
        % Passing Y axis, gate faces Y, meaning posts span across X
        p1 = [gx - g_width/2, gy, 0];
        p2 = [gx + g_width/2, gy, 0];
    end

    % Draw two vertical posts and one horizontal bar
    plot3([p1(1) p1(1)], [p1(2) p1(2)], [0 g_height], 'k', 'LineWidth', 4);
    plot3([p2(1) p2(1)], [p2(2) p2(2)], [0 g_height], 'k', 'LineWidth', 4);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [g_height g_height], 'k', 'LineWidth', 4);
end

P_tip = plot_robot(q, d1,a2,a3,L4,delta, 0.04);
end

function pos = forward_kinematics(q, d1,a2,a3,L4,delta)
t1=q(1); t2=q(2); t3=q(3); t4=q(4);
T01 = dh_matrix(t1,    d1, 0,  pi/2);
T12 = dh_matrix(t2+delta, 0, a2, 0);
T23 = dh_matrix(t3,    0,  a3, 0);
T34 = dh_matrix(t4,    0,  L4, 0);
pos = (T01*T12*T23*T34) * [0;0;0;1];
pos = pos(1:3);
end

function [theta1,theta2,theta3,theta4,isValid] = inverse_kinematics(x,y,z,phi,d1,a2,a3,L4,delta,limits)
isValid  = true;
theta1   = atan2(y, x);
r_target = sqrt(x^2 + y^2);
z_target = z - d1;
r_w      = r_target - L4*cos(phi);
z_w      = z_target - L4*sin(phi);
D_sq     = r_w^2 + z_w^2;
cos_t3   = (D_sq - a2^2 - a3^2) / (2*a2*a3);

if abs(cos_t3) > 1
    isValid = false; theta1=0;theta2=0;theta3=0;theta4=0; return;
end

sin_t3 = -sqrt(1 - cos_t3^2);
theta3 = atan2(sin_t3, cos_t3);

alpha  = atan2(z_w, r_w);
cos_b  = (a2^2 + D_sq - a3^2) / (2*a2*sqrt(D_sq));
if abs(cos_b) > 1, cos_b = sign(cos_b); end
beta   = acos(cos_b);

angle_link2 = alpha + beta;
theta2      = angle_link2 - delta;
theta4      = phi - (angle_link2 + theta3);

if nargin >= 10 && ~isempty(limits)
    if theta1 < limits(1,1) || theta1 > limits(1,2) || ...
            theta2 < limits(2,1) || theta2 > limits(2,2) || ...
            theta3 < limits(3,1) || theta3 > limits(3,2) || ...
            theta4 < limits(4,1) || theta4 > limits(4,2)
        isValid = false;
    end
end
end

function P_tip = plot_robot(q, d1,a2,a3,L4,delta,s)
t1=q(1); t2=q(2); t3=q(3); t4=q(4);
T01 = dh_matrix(t1,       d1, 0,  pi/2);
T12 = dh_matrix(t2+delta,  0, a2, 0);
T23 = dh_matrix(t3,        0, a3, 0);
T34 = dh_matrix(t4,        0, L4, 0);
T02 = T01*T12; T03 = T02*T23; T04 = T03*T34;

pts = [[0;0;0], T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4)];
plot3(pts(1,:), pts(2,:), pts(3,:), '-k', 'LineWidth', 3, ...
    'Marker','o','MarkerFaceColor','y','MarkerSize',6);

plot_frame(eye(4), s);
plot_frame(T01, s); plot_frame(T02, s);
plot_frame(T03, s); plot_frame(T04, s);

P_tip = T04(1:3,4);
end

function T = dh_matrix(theta, d, a, alpha)
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1];
end

function plot_frame(T, s)
p = T(1:3,4); R = T(1:3,1:3);
line([p(1) p(1)+R(1,1)*s],[p(2) p(2)+R(2,1)*s],[p(3) p(3)+R(3,1)*s],'Color','r','LineWidth',2);
line([p(1) p(1)+R(1,2)*s],[p(2) p(2)+R(2,2)*s],[p(3) p(3)+R(3,2)*s],'Color','g','LineWidth',2);
line([p(1) p(1)+R(1,3)*s],[p(2) p(2)+R(2,3)*s],[p(3) p(3)+R(3,3)*s],'Color','b','LineWidth',2);
end
