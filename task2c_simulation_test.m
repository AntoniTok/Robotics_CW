% task2c_simulation_test.m
% Pure simulation script for initial testing of gate traversal logic.
% No hardware communication is required.

clear; clc; close all;

%% 1. Simulation Mode
INITIAL_TEST_MODE = false;  % true: faster initial check, false: full simulation path

%% 2. Scene Configuration
GRID_UNIT = 0.025;
ROBOT_GX = 9;
ROBOT_GY = 3;

% Gate definitions: {gx, gy, orientation, pass_height}
gates = {
    5,  6,  'x', 0.11;
    7, 11, 'x', 0.08;
    11, 8,  'y', 0.11;
    15, 4,  'y', 0.08;
    };

if INITIAL_TEST_MODE
    gates = gates(1:2, :);
end

stick_start = [17, 1];

%% 3. Robot Kinematic Parameters
% Consistent with task2c.m

d1 = 0.077;
a2 = sqrt(0.128^2 + 0.024^2);
delta = atan2(0.024, 0.128);
a3 = 0.124;
a4 = 0.126;
L_finger = 0.025;
L_tip_total = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-185), deg2rad(185);
    deg2rad(-95) + shift_q2, deg2rad(95) + shift_q2;
    deg2rad(-80) + shift_q3, deg2rad(90) + shift_q3;
    deg2rad(-140), deg2rad(140)
    ];

%% 4. Visualization Setup
figure('Name', 'Task 2c Simulation Test', 'Color', 'w', 'Position', [100 100 1200 800]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.3 0.4 -0.4 0.4 0 0.6]);

%% 5. Home Position
home_x = 0.25;
home_y = 0;
home_z = 0.20;
home_pitch = 0;

[q1, q2, q3, q4, valid] = inverse_kinematics(home_x, home_y, home_z, home_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if ~valid
    error('Home position unreachable in simulation.');
end

current_q = [q1, q2, q3, q4];
plot_scene_gates(current_q, gates, d1, a2, a3, L_tip_total, delta, GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(0.8);

%% 6. Simulated Stick Pick
sx = stick_start(1);
sy = stick_start(2);
[swx, swy, ~] = grid_to_world(sx, sy, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

hover_z = 0.09;
grip_z = 0.05;
z_pass_initial = grip_z + 0.03;
pick_pitch = -pi/3;

fprintf('Sim pick at Grid(%d,%d)\n', sx, sy);

waypoints_pick = [
    swx, swy, hover_z, pick_pitch;
    swx, swy, grip_z, pick_pitch;
    swx, swy, z_pass_initial, pick_pitch;
    ];

current_q = follow_waypoints(current_q, waypoints_pick, d1, a2, a3, L_tip_total, delta, joint_limits, ...
    gates, GRID_UNIT, ROBOT_GX, ROBOT_GY, INITIAL_TEST_MODE);

%% 7. Gate Traversal (Simulation Only)
fprintf('Start passing %d gate(s) in simulation\n', size(gates, 1));

for i = 1:size(gates, 1)
    g_x = gates{i, 1};
    g_y = gates{i, 2};
    g_ori = gates{i, 3};
    g_z_pass = gates{i, 4};

    [wx, wy, ~] = grid_to_world(g_x, g_y, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    pass_dist = 0.025;
    if strcmpi(g_ori, 'x')
        wpt_pre_x = wx;
        wpt_pre_y = wy + pass_dist;
        wpt_post_x = wx;
        wpt_post_y = wy - pass_dist;
    else
        wpt_pre_x = wx - pass_dist;
        wpt_pre_y = wy;
        wpt_post_x = wx + pass_dist;
        wpt_post_y = wy;
    end

    if i == 1
        waypoints_gate = [
            wpt_pre_x,  wpt_pre_y,  g_z_pass, pick_pitch;
            wx,         wy,         g_z_pass, pick_pitch;
            wpt_post_x, wpt_post_y, g_z_pass, pick_pitch;
            ];
    else
        waypoints_gate = [
            wpt_pre_x,  prev_wpt_post_y, prev_z_pass, pick_pitch;
            wpt_pre_x,  prev_wpt_post_y, g_z_pass,    pick_pitch;
            wpt_pre_x,  wpt_pre_y,       g_z_pass,    pick_pitch;
            wx,         wy,              g_z_pass,    pick_pitch;
            wpt_post_x, wpt_post_y,      g_z_pass,    pick_pitch;
            ];
    end

    fprintf('Passing gate %d at Grid(%d,%d), z=%.3f\n', i, g_x, g_y, g_z_pass);

    current_q = follow_waypoints(current_q, waypoints_gate, d1, a2, a3, L_tip_total, delta, joint_limits, ...
        gates, GRID_UNIT, ROBOT_GX, ROBOT_GY, INITIAL_TEST_MODE);

    prev_wpt_post_y = wpt_post_y;
    prev_z_pass = g_z_pass;
end

%% 8. Return to End Pose
end_x = 0.175;
end_y = 0;
end_z = 0.15;
end_pitch = 0;

[q1, q2, q3, q4, valid] = inverse_kinematics(end_x, end_y, end_z, end_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if valid
    current_q = [q1, q2, q3, q4];
    plot_scene_gates(current_q, gates, d1, a2, a3, L_tip_total, delta, GRID_UNIT, ROBOT_GX, ROBOT_GY);
end

fprintf('Simulation test completed.\n');


%% --- Helper Functions ---
function q_out = follow_waypoints(q_in, waypoints, d1, a2, a3, L4, delta, limits, gates, unit, rx, ry, quick_mode)
q_out = q_in;

for wp_idx = 1:size(waypoints, 1)
    goal_x = waypoints(wp_idx, 1);
    goal_y = waypoints(wp_idx, 2);
    goal_z = waypoints(wp_idx, 3);
    goal_pitch = waypoints(wp_idx, 4);

    current_pos = forward_kinematics(q_out, d1, a2, a3, L4, delta);
    current_pitch = q_out(2) + delta + q_out(3) + q_out(4);

    dist = sqrt((goal_x - current_pos(1))^2 + (goal_y - current_pos(2))^2 + (goal_z - current_pos(3))^2);

    if quick_mode
        num_steps = max(8, round(dist * 60));
        dt = 0.005;
    else
        num_steps = max(12, round(dist * 120));
        dt = 0.01;
    end

    traj_x = linspace(current_pos(1), goal_x, num_steps);
    traj_y = linspace(current_pos(2), goal_y, num_steps);
    traj_z = linspace(current_pos(3), goal_z, num_steps);
    traj_p = linspace(current_pitch, goal_pitch, num_steps);

    for t = 1:num_steps
        [q1, q2, q3, q4, valid] = inverse_kinematics(traj_x(t), traj_y(t), traj_z(t), traj_p(t), ...
            d1, a2, a3, L4, delta, limits);

        if valid
            q_out = [q1, q2, q3, q4];
            plot_scene_gates(q_out, gates, d1, a2, a3, L4, delta, unit, rx, ry);
            drawnow;
            pause(dt);
        end
    end
end
end

function [wx, wy, wz] = grid_to_world(gx, gy, gz_scale, unit, r_gx, r_gy)
wx = (gy - r_gy) * unit;
wy = (r_gx - gx) * unit;
wz = gz_scale * unit;
end

function plot_scene_gates(q, gates, d1, a2, a3, L4, delta, unit, rx, ry)
cla; hold on; grid on; axis equal;
axis([-0.3 0.4 -0.4 0.4 0 0.6]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

for k = 1:size(gates, 1)
    [gx, gy, ~] = grid_to_world(gates{k,1}, gates{k,2}, 0, unit, rx, ry);
    g_width = 0.10;
    g_height = 0.08;

    ori = gates{k,3};

    if strcmpi(ori, 'x')
        p1 = [gx - g_width/2, gy, 0];
        p2 = [gx + g_width/2, gy, 0];
    else
        p1 = [gx, gy - g_width/2, 0];
        p2 = [gx, gy + g_width/2, 0];
    end

    plot3([p1(1) p1(1)], [p1(2) p1(2)], [0 g_height], 'k', 'LineWidth', 4);
    plot3([p2(1) p2(1)], [p2(2) p2(2)], [0 g_height], 'k', 'LineWidth', 4);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [g_height g_height], 'k', 'LineWidth', 4);
end

plot_robot(q, d1, a2, a3, L4, delta, 0.04);
end

function pos = forward_kinematics(q, d1, a2, a3, L4, delta)
T01 = dh_matrix(q(1), d1, 0, pi/2);
T12 = dh_matrix(q(2) + delta, 0, a2, 0);
T23 = dh_matrix(q(3), 0, a3, 0);
T34 = dh_matrix(q(4), 0, L4, 0);

T = T01 * T12 * T23 * T34;
pos = T(1:3, 4);
end

function [theta1, theta2, theta3, theta4, isValid] = inverse_kinematics(x, y, z, phi, d1, a2, a3, L4, delta, limits)
isValid = true;
theta1 = atan2(y, x);
r_target = sqrt(x^2 + y^2);
z_target = z - d1;
r_w = r_target - L4 * cos(phi);
z_w = z_target - L4 * sin(phi);
D_sq = r_w^2 + z_w^2;
cos_t3 = (D_sq - a2^2 - a3^2) / (2 * a2 * a3);

if abs(cos_t3) > 1
    isValid = false;
    theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;
    return;
end

sin_t3 = -sqrt(1 - cos_t3^2);
theta3 = atan2(sin_t3, cos_t3);

alpha = atan2(z_w, r_w);
cos_b = (a2^2 + D_sq - a3^2) / (2 * a2 * sqrt(D_sq));
if abs(cos_b) > 1
    cos_b = sign(cos_b);
end
beta = acos(cos_b);

angle_link2 = alpha + beta;
theta2 = angle_link2 - delta;
theta4 = phi - (angle_link2 + theta3);

if theta1 < limits(1,1) || theta1 > limits(1,2) || ...
        theta2 < limits(2,1) || theta2 > limits(2,2) || ...
        theta3 < limits(3,1) || theta3 > limits(3,2) || ...
        theta4 < limits(4,1) || theta4 > limits(4,2)
    isValid = false;
end
end

function P_tip = plot_robot(q, d1, a2, a3, L4, delta, s)
T01 = dh_matrix(q(1), d1, 0, pi/2);
T12 = dh_matrix(q(2) + delta, 0, a2, 0);
T23 = dh_matrix(q(3), 0, a3, 0);
T34 = dh_matrix(q(4), 0, L4, 0);

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;

pts = [[0;0;0], T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4)];
plot3(pts(1,:), pts(2,:), pts(3,:), '-k', 'LineWidth', 3, ...
    'Marker', 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 6);

plot_frame(eye(4), s);
plot_frame(T01, s);
plot_frame(T02, s);
plot_frame(T03, s);
plot_frame(T04, s);

P_tip = T04(1:3,4);
end

function T = dh_matrix(theta, d, a, alpha)
T = [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta);
    sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
    0,           sin(alpha),               cos(alpha),              d;
    0,           0,                        0,                       1];
end

function plot_frame(T, s)
p = T(1:3,4);
R = T(1:3,1:3);
line([p(1) p(1) + R(1,1) * s], [p(2) p(2) + R(2,1) * s], [p(3) p(3) + R(3,1) * s], 'Color', 'r', 'LineWidth', 2);
line([p(1) p(1) + R(1,2) * s], [p(2) p(2) + R(2,2) * s], [p(3) p(3) + R(3,2) * s], 'Color', 'g', 'LineWidth', 2);
line([p(1) p(1) + R(1,3) * s], [p(2) p(2) + R(2,3) * s], [p(3) p(3) + R(3,3) * s], 'Color', 'b', 'LineWidth', 2);
end
