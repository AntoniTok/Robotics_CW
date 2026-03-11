% task_golf.m
% Script for robot arm to pick up a golf club and hit a ball
% Requires pre-defined grid functions and kinematic functions from previous tasks

clear; clc; close all;

%% 1. Initialization
fprintf('Starting robot control for Golf...\n');
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
DEVICENAME = 'COM7';

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;
SAFE_PROFILE_VEL   = 150;
SWING_PROFILE_VEL  = 400; % Much faster velocity for hitting
GRIPPER_PROFILE_VEL = 200;
SAFE_PROFILE_ACCEL = 30;
SWING_PROFILE_ACCEL = 100; % Faster acceleration for hitting
global MOTOR_11_OFFSET;
MOTOR_11_OFFSET    = deg2rad(1);

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
%% 2. Gripper Setup
GRIPPER_OPEN  = deg2rad(-45);
GRIPPER_CLOSE = deg2rad(23);
current_gripper = GRIPPER_OPEN;

%% 3. Scene Configuration
GRID_UNIT = 0.025;
GRID_W    = 17;
GRID_H    = 12;

ROBOT_GX  = 9;
ROBOT_GY  = 3;

% Customizable target locations
club_start = [17.1, 0.9]; % Grid position of the golf club
ball_pos   = [9, 12]; % Target ball grid position

%% 4. Robot Kinematic Parameters
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
    deg2rad(-185), deg2rad(185);
    deg2rad(-95)  + shift_q2,  deg2rad(95) + shift_q2;
    deg2rad(-80)  + shift_q3,  deg2rad(90) + shift_q3;
    deg2rad(-140),             deg2rad(140)
    ];

fig = figure('Name','Task Golf','Color','w','Position',[100 100 1200 800]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.4 0.4 -0.4 0.4 0 0.6]);

%% 5. Home Positioning
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

plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta, GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(2);

%% 6. Club Collection
try
    cx = club_start(1);
    cy = club_start(2);
    [cwx, cwy, ~]  = grid_to_world(cx, cy, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    hover_z = 0.09;
    grip_z  = 0.05;
    lift_height = 0.10; % Lift higher to clear ground with club
    z_pass_initial  = grip_z + lift_height;
    pick_pitch = -pi/2; % Hanging straight down

    fprintf('Picking up the golf club at Grid(%d,%d)\n', cx, cy);

    waypoints_pick = [
        cwx, cwy, hover_z, pick_pitch, 0;
        cwx, cwy, grip_z,  pick_pitch, 1;
        cwx, cwy, z_pass_initial,  pick_pitch, 2;
        ];

    for wp_idx = 1:size(waypoints_pick, 1)
        target    = waypoints_pick(wp_idx, :);
        goal_x    = target(1); goal_y = target(2); goal_z = target(3);
        goal_pitch = target(4); action = target(5);

        current_pos         = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
        current_pitch_val   = current_q(2) + delta + current_q(3) + current_q(4);

        num_steps = 15;
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

                if t == num_steps
                    if action == 1 % CLOSE GRIPPER
                        current_gripper = GRIPPER_CLOSE;
                        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, true);
                        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                        pause(0.5);
                    end
                end

                phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, current_gripper == GRIPPER_CLOSE);
                send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

                plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
                drawnow;
                pause(0.010);
            else
                warning('Trajectory point unreachable during Pick sequence!');
            end
        end
    end

    % --- UPDATE FINGER LENGTH ---
    fprintf('Club picked up. Increasing finger length (L_tip_total) by 50mm.\n');
    L_tip_total = L_tip_total + 0.050; % Add 50mm to account for club length

catch ME
    fprintf('Program interrupted during pick: %s\n', ME.message);
end

%% 7. Full Golf Swing (Backswing, Fast Hit, Follow-through)
try
    bx = ball_pos(1);
    by = ball_pos(2);
    [bwx, bwy, ~]  = grid_to_world(bx, by, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    r_hit = sqrt(bwx^2 + bwy^2);
    theta_ball = atan2(bwy, bwx);
    hit_z = 0.015;
    hit_pitch = -pi/4; % 摆出45度击球

    % Fast impact zone: +/- 50mm around ball. 0.05 / radius = angular spread
    delta_theta = atan2(0.05, r_hit);
    theta_pre  = theta_ball + delta_theta;
    theta_post = theta_ball - delta_theta;

    % Backswing peak (起杆顶点，位于轨迹左侧高处)
    theta_back = theta_ball + deg2rad(90);
    r_back = r_hit - 0.06; % 贴近身体
    z_back = 0.15; % 高高举起
    pitch_back = -pi/8;

    % Follow-through peak (收杆顶点，位于轨迹右侧高处)
    theta_fwd = theta_ball - deg2rad(90);
    r_fwd = r_hit - 0.06;
    z_fwd = 0.15;
    pitch_fwd = -pi/8;

    % --- Phase 1: Move to Backswing Peak ---
    fprintf('Phase 1: Lifting and moving to Backswing Peak (起杆)...\n');
    current_pos       = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
    current_pitch_val = current_q(2) + delta + current_q(3) + current_q(4);

    % Safe lift first
    safe_z = 0.25;
    [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics( ...
        current_pos(1), current_pos(2), safe_z, current_pitch_val, ...
        d1,a2,a3,L_tip_total,delta,joint_limits);
    if valid_t
        current_q = [q1_t, q2_t, q3_t, q4_t];
        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, current_gripper == GRIPPER_CLOSE);
        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
        plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
        pause(0.5);
    end

    % Sweep to the peak point
    num_steps = 25;
    back_x = r_back * cos(theta_back);
    back_y = r_back * sin(theta_back);

    traj_x     = linspace(current_pos(1), back_x, num_steps);
    traj_y     = linspace(current_pos(2), back_y, num_steps);
    traj_z     = linspace(safe_z, z_back, num_steps);
    traj_pitch = linspace(current_pitch_val, pitch_back, num_steps);

    for t = 1:num_steps
        [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics( ...
            traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
            d1,a2,a3,L_tip_total,delta,joint_limits);
        if valid_t
            current_q = [q1_t, q2_t, q3_t, q4_t];
            phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, current_gripper == GRIPPER_CLOSE);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
            plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
            drawnow;
            pause(0.010);
        end
    end

    % --- Phase 2, 3, 4: The Unified FAST Swing ---
    fprintf('Executing full continuous fast swing (Downswing -> Impact -> Follow-through)...\n');

    % Set fast velocity profile for ALL moving motors
    for k = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, SWING_PROFILE_ACCEL);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, SWING_PROFILE_VEL);
    end

    % Trajectory 1: Downswing (Phase 2)
    num_steps_down = 15;
    traj_theta_2 = linspace(theta_back, theta_pre, num_steps_down);
    traj_r_2     = linspace(r_back, r_hit, num_steps_down);
    traj_z_2     = linspace(z_back, hit_z, num_steps_down);
    traj_pitch_2 = linspace(pitch_back, hit_pitch, num_steps_down);

    % Trajectory 2: Impact Zone (Phase 3)
    num_steps_hit = 10;
    traj_theta_3 = linspace(theta_pre, theta_post, num_steps_hit);
    traj_r_3     = linspace(r_hit, r_hit, num_steps_hit);
    traj_z_3     = linspace(hit_z, hit_z, num_steps_hit);
    traj_pitch_3 = linspace(hit_pitch, hit_pitch, num_steps_hit);

    % Trajectory 3: Follow-through (Phase 4)
    num_steps_up = 15;
    traj_theta_4 = linspace(theta_post, theta_fwd, num_steps_up);
    traj_r_4     = linspace(r_hit, r_fwd, num_steps_up);
    traj_z_4     = linspace(hit_z, z_fwd, num_steps_up);
    traj_pitch_4 = linspace(hit_pitch, pitch_fwd, num_steps_up);

    % Combine them
    full_theta = [traj_theta_2, traj_theta_3(2:end), traj_theta_4(2:end)];
    full_r     = [traj_r_2,     traj_r_3(2:end),     traj_r_4(2:end)];
    full_z     = [traj_z_2,     traj_z_3(2:end),     traj_z_4(2:end)];
    full_pitch = [traj_pitch_2, traj_pitch_3(2:end), traj_pitch_4(2:end)];

    for t = 1:length(full_theta)
        tx = full_r(t) * cos(full_theta(t));
        ty = full_r(t) * sin(full_theta(t));

        [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics( ...
            tx, ty, full_z(t), full_pitch(t), ...
            d1,a2,a3,L_tip_total,delta,joint_limits);
        if valid_t
            current_q = [q1_t, q2_t, q3_t, q4_t];
            phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, current_gripper == GRIPPER_CLOSE);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

            % Update visual output every other frame to keep physical speed high
            if mod(t, 2) == 0
                plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
                drawnow;
            end
            pause(0.002);
        end
    end

    pause(0.5);

    % Restore safe velocity profile for all motors
    for k = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, SAFE_PROFILE_ACCEL);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, SAFE_PROFILE_VEL);
    end

    fprintf('Swing complete.\n');

catch ME
    fprintf('Program interrupted during golf swing: %s\n', ME.message);
end

%% 8. Recovery Position
end_x = 0.175; end_y = 0; end_z = 0.15; end_pitch = 0;
[q1,q2,q3,q4,valid] = inverse_kinematics(end_x,end_y,end_z,end_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);

if valid
    current_q = [q1, q2, q3, q4];
    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
    plot_scene_golf(current_q, club_start, ball_pos, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    pause(2);
end

%% 9. Shutdown
fprintf('Shutting down...\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque DISABLED.\n');
closePort(port_num);
unloadlibrary(lib_name);


%% --- HELPER FUNCTIONS ---

function phys_angles = sim_to_phys_angles(sim_q, gripper_q, delta, offset_classmate, is_placing)
global MOTOR_11_OFFSET;
if is_placing
    if sim_q(1) > 0
        q1 = sim_q(1) - MOTOR_11_OFFSET;
    else
        q1 = sim_q(1);
    end
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

function plot_scene_golf(q, club_start, ball_pos, d1,a2,a3,L4,delta,unit,rx,ry)
cla; hold on; grid on; axis equal;
axis([-0.4 0.4 -0.4 0.4 0 0.6]);
view(45, 30);
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');

% Plot Club
[cwx, cwy, ~] = grid_to_world(club_start(1), club_start(2), 0, unit, rx, ry);
plot3(cwx, cwy, 0.025, 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% Plot Ball
[bwx, bwy, ~] = grid_to_world(ball_pos(1), ball_pos(2), 0, unit, rx, ry);
plot3(bwx, bwy, 0.015, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

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
