%task2b_stack.m
% Merges Task 2a Simulation (path planning) with Real Robot Control

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
DEVICENAME = 'COM8'; % Adjust if needed

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;
SAFE_PROFILE_VEL   = 150;
GRIPPER_PROFILE_VEL = 200; % Much faster for snappy grip
SAFE_PROFILE_ACCEL = 30;
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
fprintf('Torque, Acceleration & Velocity Profiles ENABLED (Gripper boosted).\n');

%% 2. GRIPPER SETTINGS (Adjust as needed)
% Define the offset in radians from 180 degrees (0 in IK frame means 180 in physical)
GRIPPER_OPEN  = deg2rad(-55); % Open position
GRIPPER_CLOSE = deg2rad(5);  % Tightened: Increased from 0 to 40 to grip harder
current_gripper = GRIPPER_OPEN;

%% 3. GRID & SCENE CONFIGURATION
GRID_UNIT = 0.025;
GRID_W    = 17;
GRID_H    = 12;

ROBOT_GX  = 9;
ROBOT_GY  = 3;

target_cubes = [
    9,  12,  1;
    16,  10,  2;
    17, 7, 3
    ];
cubes_start = target_cubes;

bridge_clearance_z = 0.03; % example: must be BELOW bridge, tune this

holders = [
    3,  3;
    3,  3;
    3,  3
    ];

cube_colors  = {'r', 'g', 'b'};
cube_height  = 0.025;

%% 4. ROBOT PARAMETERS
d1 = 0.077;
a2 = sqrt(0.128^2 + 0.024^2);
delta = atan2(0.024, 0.128);
a3 = 0.124;
a4 = 0.126;
L_finger    = 0.025; % Task2a includes finger length for pick and place
L_tip_total = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + shift_q2,  deg2rad(100) + shift_q2;
    deg2rad(-75)  + shift_q3,  deg2rad(105) + shift_q3;
    deg2rad(-135),             deg2rad(135)
    ];

fig = figure('Name','Task 2a: Real Pick and Place','Color','w','Position',[100 100 1200 800]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.3 0.4 -0.4 0.4 0 0.6]);

%% 5. INITIALIZATION
home_x = 0.3; home_y = 0; home_z = 0.20; home_pitch = 0;
[q1,q2,q3,q4,valid] = inverse_kinematics(home_x,home_y,home_z,home_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);

if ~valid
    error('Mathematical home position unreachable!');
end

current_q = [q1, q2, q3, q4];

% Move physical robot to Home initially
phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

attached_cube_idx = 0;
plot_scene(current_q, cubes_start, holders, 0, d1,a2,a3,L_tip_total,delta, ...
    GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(2); % Wait to reach home physically

%% 6. MAIN PICK AND PLACE LOOP
try
    for i = 1:size(cubes_start, 1)

        [cx, cy, ~]  = grid_to_world(cubes_start(i,1), cubes_start(i,2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
        cz_pick      = cube_height/2+0.015;

        [hx, hy, ~]  = grid_to_world(holders(i,1), holders(i,2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
        cz_place = (cube_height/2 + 0.015) + (i-1)*0.025;   % 2.5 cm step

        hover_z = 0.06; % Increased hover to prevent physical collision

        fprintf('Moving Cube %d: Grid(%d,%d) -> Grid(%d,%d)\n', ...
            i, cubes_start(i,1), cubes_start(i,2), holders(i,1), holders(i,2));

        % Pitch selection
        best_pitch  = -pi/2;
        test_angles = deg2rad(-90 : 5 : 90);
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

        % Waypoints: [x, y, z, pitch, action]
        % action: 0=Stay Open, 1=Close (Pick), 2=Stay Closed, 3=Open (Place)
        under_bridge = (i == 1);   % cube 1 special-case

        if under_bridge
            pick_pitch = -pi/6;        % parallel to table (horizontal)
        else
            pick_pitch = best_pitch;  % your normal pitch result
        end
        if under_bridge
            SAFE_Z_ABOVE = 0.10;     % above 6cm bridge
            z_corridor   = 0.04;     % under bridge (< 0.06) but above table
            cz_pick      = cube_height/2 + 0.025;

            approach_dx = -0.03;     % entry offset (near cube)
            approach_dy =  0.00;
            GRIP_DY = 0.01;   % 2 cm extra in +Y (tune)
            gx = cx+ GRIP_DY;
            gy = cy ;

            EXIT_DIST = 0.09;        % slide-out distance (>= 0.06)
            entry_x = cx + approach_dx;
            entry_y = cy + approach_dy;
            exit_x  = entry_x - EXIT_DIST;
            exit_y  = entry_y;

            % after you define exit_x, exit_y etc...
            retreat_x = entry_x-0.1;   % or exit_x, depending how far you want to retreat
            retreat_y = entry_y;   % keep same y if you want straight back
            retreat_z = cz_pick;  % stay low while retreating

            waypoints = [
                exit_x,  exit_y,  SAFE_Z_ABOVE, pick_pitch, 0;

                entry_x, entry_y, z_corridor,   pick_pitch, 0;

                gx,      gy,      z_corridor,       pick_pitch, 0; % slide IN corridor
                gx,      gy,      cz_pick-0.002,      pick_pitch, 0; % down
                gx,      gy,      cz_pick-0.002,      pick_pitch, 1; % CLOSE (arm still)

                exit_x,  exit_y,  z_corridor+0.01,   pick_pitch, 2;
                exit_x,  exit_y,  SAFE_Z_ABOVE,      pick_pitch, 2;
                hx,      hy,      cz_place+hover_z,  pick_pitch, 2;
                hx+0.002,      hy-0.006,      cz_place+0.02,     pick_pitch-(pi/2), 2;
                hx+0.002,      hy-0.006,      cz_place+0.01,     pick_pitch-(pi/2), 3;
                hx,      hy,      cz_place+hover_z,  best_pitch, 0;

            ];
        else

            % your normal top-down waypoints
            waypoints = [
                cx, cy, cz_pick  + hover_z, best_pitch, 0;
                cx, cy, cz_pick,            best_pitch, 1;
                cx, cy, cz_pick  + hover_z, best_pitch, 2;
                hx, hy, cz_place + hover_z, best_pitch, 2;
                hx, hy, cz_place,           best_pitch, 3;
                hx, hy, cz_place + hover_z, best_pitch, 0;
            ];
        end

        for wp_idx = 1:size(waypoints, 1)
            target    = waypoints(wp_idx, :);
            goal_x    = target(1); goal_y = target(2); goal_z = target(3);
            goal_pitch = target(4); action  = target(5);

            current_pos         = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
            current_pitch_val   = current_q(2) + delta + current_q(3) + current_q(4);

            num_steps = 12;     % 10–15 is good for real robot
            pause(0.03);        % 0.02–0.05 (lower jerk)
            traj_x     = linspace(current_pos(1), goal_x,     num_steps);
            traj_y     = linspace(current_pos(2), goal_y,     num_steps);
            traj_z     = linspace(current_pos(3), goal_z,     num_steps);
            traj_pitch = linspace(current_pitch_val, goal_pitch, num_steps);
            last_valid_q = current_q;

            for t = 1:num_steps
                [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics( ...
                    traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
                    d1,a2,a3,L_tip_total,delta,joint_limits);

                if valid_t
                    current_q = [q1_t, q2_t, q3_t, q4_t];
                    last_valid_q = current_q;

                    if t == num_steps
                        % --- Perform gripper action at the waypoint, even if last IK step failed ---
                        if action == 1      % PICK close
                            attached_cube_idx = i;
                            current_gripper = GRIPPER_CLOSE;
                            phys_angles = sim_to_phys_angles(last_valid_q, current_gripper, delta, offset_classmate, attached_cube_idx > 0);
                            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                            pause(0.3);

                        elseif action == 3  % PLACE open
                            cubes_start(i, 1:2) = holders(i, :);
                            attached_cube_idx   = 0;
                            current_gripper = GRIPPER_OPEN;
                            phys_angles = sim_to_phys_angles(last_valid_q, current_gripper, delta, offset_classmate, false);
                            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                            pause(0.3);
                        end
                    end

                    % Send regular trajectory step
                    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_cube_idx > 0);
                    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

                    if attached_cube_idx > 0
                        cubes_start(attached_cube_idx, 1:2) = [-999, -999]; % hide; drawn at tip
                    end

                    % Visual Simulation Sync
                    plot_scene(current_q, cubes_start, holders, attached_cube_idx, ...
                        d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
                    drawnow;

                    % Extra pause for grab/release sending
                    if t == num_steps && (action == 1 || action == 3)
                        % Re-send to make sure gripper moves while arm stationary
                        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_cube_idx > 0);
                        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                        pause(0.5);
                    end

                    % Wait slightly to make simulation pace match Dynamixel
                    pause(0.01);
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

if ~valid
    error('Mathematical end position unreachable!');
end

current_q = [q1, q2, q3, q4];

% Move physical robot to Home initially
phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

attached_cube_idx = 0;
plot_scene(current_q, cubes_start, holders, 0, d1,a2,a3,L_tip_total,delta, ...
    GRID_UNIT, ROBOT_GX, ROBOT_GY);
pause(2); % Wait to reach home physically
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
% Converts mathematical DH angles to the expected Physical Motor frame mapping
% Dofbot neutral is 180 degrees (2048 ticks) for all motors.
global MOTOR_11_OFFSET;
if is_placing
    q1 = sim_q(1) + MOTOR_11_OFFSET;
else
    q1 = sim_q(1);
end

% Fix from Robot_Control_Single_225 (reverse physical mapping direction for motors)
q2 = -(sim_q(2) + delta - offset_classmate);
q3 = -(sim_q(3) + offset_classmate);
q4 = -sim_q(4);
q5 = gripper_q;

phys_angles = [q1; q2; q3; q4] + deg2rad(180);
phys_angles = [phys_angles; q5  + deg2rad(210)];
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

function plot_scene(q, cubes, holders, attached_idx, d1,a2,a3,L4,delta,unit,rx,ry)
cla; hold on; grid on; axis equal;
axis([-0.3 0.4 -0.4 0.4 0 0.6]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

for k = 1:size(holders, 1)
    [hx, hy, ~] = grid_to_world(holders(k,1), holders(k,2), 0, unit, rx, ry);
    plot3(hx, hy, 0, 'ks', 'MarkerSize', 10, 'LineWidth', 2);
end

P_tip = plot_robot(q, d1,a2,a3,L4,delta, 0.04);

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

sin_t3 = -sqrt(1 - cos_t3^2);
theta3 = atan2(sin_t3, cos_t3);

alpha  = atan2(z_w, r_w);
cos_b  = (a2^2 + D_sq - a3^2) / (2*a2*sqrt(D_sq));
if abs(cos_b) > 1, cos_b = sign(cos_b); end
beta   = acos(cos_b);

angle_link2 = alpha + beta;
theta2      = angle_link2 - delta;
theta4      = phi - (angle_link2 + theta3);
fprintf("theta2=%.1f deg, lim=[%.1f, %.1f]\n", rad2deg(theta2), rad2deg(limits(2,1)), rad2deg(limits(2,2)));
fprintf("theta3=%.1f deg, lim=[%.1f, %.1f]\n", rad2deg(theta3), rad2deg(limits(3,1)), rad2deg(limits(3,2)));

if  ~isempty(limits)
    if theta1 < limits(1,1) || theta1 > limits(1,2)
        disp("limit fail: theta1"); isValid = false;
    elseif theta2 < limits(2,1) || theta2 > limits(2,2)
        disp("limit fail: theta2"); isValid = false;
    elseif theta3 < limits(3,1) || theta3 > limits(3,2)
        disp("limit fail: theta3"); isValid = false;
    elseif theta4 < limits(4,1) || theta4 > limits(4,2)
        disp("limit fail: theta4"); isValid = false;
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
