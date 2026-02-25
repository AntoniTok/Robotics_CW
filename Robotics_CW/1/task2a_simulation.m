% Task 2a: Pick and Place Simulation (Grid System)
% Robot Base: Grid (9,3), Grid Size: 17x12, Unit: 0.025 m

clear; clc; close all;

%% 1. GRID CONFIGURATION  <-- CHANGE grid size / robot position here
GRID_UNIT = 0.025;  % metres per cell
GRID_W    = 17;     % number of columns (X)
GRID_H    = 12;     % number of rows    (Y)

ROBOT_GX  = 9;      % robot base column (1-based)
ROBOT_GY  = 3;      % robot base row    (1-based)
% World frame: X = forward (+gy), Y = left (+gx decreasing)

%% 2. OBJECT DEFINITIONS  <-- CHANGE cube positions / targets here
% [GridX, GridY, ColorID]   ColorID: 1=Red, 2=Green, 3=Blue
target_cubes = [
    1,  1, 1;   % Cube 1 (Red)
    2,  8, 2;   % Cube 2 (Green)
    17, 12, 3    % Cube 3 (Blue)
    ];
cubes_start = target_cubes;

% [GridX, GridY] destination holders
holders = [
    6,  6;
    9,  6;
    12, 6
    ];

cube_colors  = {'r', 'g', 'b'};
cube_height  = 0.025;  % metres  <-- CHANGE if cube size differs

%% 3. ROBOT PARAMETERS  <-- CHANGE to match your Dofbot link lengths
d1 = 0.077;
a2 = sqrt(0.128^2 + 0.024^2);  % ~0.1302 m
delta = atan2(0.024, 0.128);    % link-2 angle offset (~0.185 rad)
a3 = 0.124;
a4 = 0.126;
L_finger    = 0.08;
L_tip_total = a4 + L_finger;

% Joint limits (derived from classmate offset convention)
offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + shift_q2,  deg2rad(90) + shift_q2;
    deg2rad(-75)  + shift_q3,  deg2rad(85) + shift_q3;
    deg2rad(-135),             deg2rad(135)
    ];

%% 4. SIMULATION SETUP
fig = figure('Name','Task 2a: Pick and Place','Color','w','Position',[100 100 1200 800]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.3 0.4 -0.4 0.4 0 0.6]);  % <-- CHANGE axis limits if robot/scene is larger

%% 5. MAIN LOOP
% Safe home position -- robot starts here before each move
home_x = 0.15; home_y = 0; home_z = 0.20; home_pitch = 0;
[q1,q2,q3,q4,~] = inverse_kinematics(home_x,home_y,home_z,home_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);
current_q = [q1, q2, q3, q4];

gripper_state    = 0;  % 0=Open, 1=Closed
attached_cube_idx = 0; % index of cube currently held (0 = none)

plot_scene(current_q, cubes_start, holders, 0, d1,a2,a3,L_tip_total,delta, ...
    GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(1);

for i = 1:size(cubes_start, 1)

    % World-space positions for cube i and its holder
    [cx, cy, ~]  = grid_to_world(cubes_start(i,1), cubes_start(i,2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
    cz_pick      = cube_height / 2;

    [hx, hy, ~]  = grid_to_world(holders(i,1), holders(i,2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
    cz_place     = cube_height / 2;

    hover_z = 0.03;  % metres above target before descending  <-- CHANGE hover height here

    fprintf('Moving Cube %d: Grid(%d,%d) -> Grid(%d,%d)\n', ...
        i, cubes_start(i,1), cubes_start(i,2), holders(i,1), holders(i,2));

    % --- Pitch selection: try vertical (-90 deg) first, relax to -45 deg ---
    % CHANGE search range below if different grip angles are needed
    best_pitch  = -pi/2;
    test_angles = deg2rad(-90 : 5 : -45);  % steepest first

    for angle = test_angles
        [~,~,~,~, vp]  = inverse_kinematics(cx, cy, cz_pick,          angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, vph] = inverse_kinematics(cx, cy, cz_pick + hover_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, vd]  = inverse_kinematics(hx, hy, cz_place,          angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, vdh] = inverse_kinematics(hx, hy, cz_place + hover_z,angle, d1,a2,a3,L_tip_total,delta,joint_limits);

        if vp && vph && vd && vdh
            best_pitch = angle;
            fprintf('  Valid pitch: %.1f deg\n', rad2deg(best_pitch));
            break;
        end
    end

    if abs(best_pitch - (-pi/2)) > 0.01
        fprintf('  Vertical grip unreachable. Using %.1f deg.\n', rad2deg(best_pitch));
    end

    % Waypoints: [x, y, z, pitch, action]
    % action: 0=Stay Open, 1=Close (Pick), 2=Stay Closed, 3=Open (Place)
    waypoints = [
        cx, cy, cz_pick  + hover_z, best_pitch, 0;  % Approach pick (hover)
        cx, cy, cz_pick,            best_pitch, 1;  % Pick (grip)
        cx, cy, cz_pick  + hover_z, best_pitch, 2;  % Lift
        hx, hy, cz_place + hover_z, best_pitch, 2;  % Approach place (hover)
        hx, hy, cz_place,           best_pitch, 3;  % Place (release)
        hx, hy, cz_place + hover_z, best_pitch, 0;  % Retract
        ];

    for wp_idx = 1:size(waypoints, 1)
        target    = waypoints(wp_idx, :);
        goal_x    = target(1); goal_y = target(2); goal_z = target(3);
        goal_pitch = target(4); action  = target(5);

        current_pos         = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
        current_pitch_val   = current_q(2) + delta + current_q(3) + current_q(4);

        num_steps = 30;  % <-- CHANGE for smoother / faster motion

        traj_x     = linspace(current_pos(1), goal_x,     num_steps);
        traj_y     = linspace(current_pos(2), goal_y,     num_steps);
        traj_z     = linspace(current_pos(3), goal_z,     num_steps);
        traj_pitch = linspace(current_pitch_val, goal_pitch, num_steps);

        for t = 1:num_steps
            [q1,q2,q3,q4,valid] = inverse_kinematics( ...
                traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
                d1,a2,a3,L_tip_total,delta,joint_limits);

            if valid
                current_q = [q1, q2, q3, q4];

                if attached_cube_idx > 0
                    cubes_start(attached_cube_idx, 1:2) = [-999, -999]; % hide; drawn at tip
                end

                if t == num_steps
                    if action == 1      % PICK
                        attached_cube_idx = i;
                    elseif action == 3  % PLACE
                        cubes_start(i, 1:2) = holders(i, :);
                        attached_cube_idx   = 0;
                    end
                end

                plot_scene(current_q, cubes_start, holders, attached_cube_idx, ...
                    d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
                drawnow;
            end
        end
    end

end


%% --- HELPER FUNCTIONS ---

function [wx, wy, wz] = grid_to_world(gx, gy, gz_scale, unit, r_gx, r_gy)
% Converts grid indices to world coordinates.
% wx (forward) increases with gy; wy (left) increases as gx decreases.
wx = (gy  - r_gy) * unit;
wy = (r_gx - gx)  * unit;
wz =  gz_scale     * unit;
end

function plot_scene(q, cubes, holders, attached_idx, d1,a2,a3,L4,delta,unit,rx,ry)
cla; hold on; grid on; axis equal;
axis([-0.3 0.4 -0.4 0.4 0 0.6]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

% Draw holders (black squares)
for k = 1:size(holders, 1)
    [hx, hy, ~] = grid_to_world(holders(k,1), holders(k,2), 0, unit, rx, ry);
    plot3(hx, hy, 0, 'ks', 'MarkerSize', 10, 'LineWidth', 2);
end

% Draw robot and get tip position
P_tip = plot_robot(q, d1,a2,a3,L4,delta, 0.04);

% Draw cubes
cols = {'r','g','b'};
for k = 1:size(cubes, 1)
    if k == attached_idx
        plot3(P_tip(1), P_tip(2), P_tip(3), 's', 'MarkerSize', 12, 'MarkerFaceColor', cols{cubes(k,3)});
    else
        [cx, cy, ~] = grid_to_world(cubes(k,1), cubes(k,2), 0, unit, rx, ry);
        plot3(cx, cy, 0.0125, 's', 'MarkerSize', 12, 'MarkerFaceColor', cols{cubes(k,3)});
    end
end
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

sin_t3 = -sqrt(1 - cos_t3^2);  % elbow-up; change sign for elbow-down
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
