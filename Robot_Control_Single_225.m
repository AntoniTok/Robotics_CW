% Robot_Control_Single_225.m
% Interactive Simulation + Real Robot Synchronous Control
% Uses Dynamixel SDK to physicalize the standalone IK logic.

clear; clc; close all;

%% ---- 1. SETUP DYNAMIXEL SDK ---- %%
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
ADDR_PRO_PRESENT_POSITION = 132;

LEN_GOAL_POSITION    = 4;
LEN_PRESENT_POSITION = 4;
PROTOCOL_VERSION = 2.0;

IDs        = [11, 12, 13, 14, 15]; % Base, Shoulder, Elbow, Wrist, Gripper
BAUDRATE   = 1000000;
DEVICENAME = 'COM7'; % Adjust if needed

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;
COMM_SUCCESS   = 0;
SAFE_PROFILE_VEL   = 30;
SAFE_PROFILE_ACCEL = 10;

%% ---- 2. OPEN PORT & ENABLE TORQUE ---- %%
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
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, SAFE_PROFILE_VEL);
end
fprintf('Torque, Acceleration & Velocity Profiles ENABLED.\n');

%% ---- 3. SIMULATION & ROBOT PARAMETERS ---- %%
d1 = 0.077;
a2 = sqrt(0.128^2 + 0.024^2);  % ~0.1302 m
delta = atan2(0.024, 0.128);   % link-2 angle offset (~10.64 deg)
a3 = 0.124;
a4 = 0.126;
L_finger    = 0.0;
L_tip_total = a4 + L_finger;

% Offset mappings for physical hardware (DOFBOT specific)
offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;

% Simulation joint limits
joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + shift_q2,  deg2rad(90) + shift_q2;
    deg2rad(-75)  + shift_q3,  deg2rad(85) + shift_q3;
    deg2rad(-135),             deg2rad(135)
    ];

fig = figure('Name','Standalone Interactive & Real Robot Control','Color','w','Position',[100 100 800 600]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');
axis([-0.3 0.4 -0.4 0.4 0 0.6]);

% Safe mathematical home position
home_x = 0.3; home_y = 0.0; home_z = 0.20; home_pitch = 0;
[q1,q2,q3,q4,valid] = sim_inverse_kinematics(home_x,home_y,home_z,home_pitch, d1,a2,a3,L_tip_total,delta,joint_limits);

if ~valid
    error('Mathematical home position is unreachable! Check parameters.');
end

current_q = [q1, q2, q3, q4];

% Move physical robot to Home initially
phys_angles = sim_to_phys_angles(current_q, delta, offset_classmate);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

% Initial Plot
cla; hold on; grid on; axis equal; axis([-0.3 0.4 -0.4 0.4 0 0.6]); view(45, 30);
plot_robot(current_q, d1,a2,a3,L_tip_total,delta, 0.04);
drawnow;

fprintf('====================================================\n');
fprintf('Real & Simulated Control Started.\n');
fprintf('Type Ctrl+C or enter empty values to exit.\n');
fprintf('====================================================\n');

%% ---- 4. INTERACTIVE MAIN LOOP ---- %%
try
    while true
        fprintf('\n-- Enter New Target --\n');
        X_in = input('Target X (m) [e.g. 0.20]: ');
        if isempty(X_in), break; end

        Y_in = input('Target Y (m) [e.g. 0.10]: ');
        if isempty(Y_in), break; end

        Z_in = input('Target Z (m) [e.g. 0.05]: ');
        if isempty(Z_in), break; end

        pitch_in = input('Desired Pitch (deg) [e.g. -90 for vertical down]: ');
        if isempty(pitch_in), break; end

        goal_pitch = deg2rad(pitch_in);

        % Check if target is valid
        [q1,q2,q3,q4,valid_target] = sim_inverse_kinematics(X_in, Y_in, Z_in, goal_pitch, d1,a2,a3,L_tip_total,delta,joint_limits);

        if ~valid_target
            fprintf('WARNING: Target is unreachable or violates joint limits!\n');
            continue;
        end

        % Target is valid, sync physical robot directly to target
        % (Physical robot handles own interpolation using PROFILE_VELOCITY)
        target_q = [q1, q2, q3, q4];
        phys_angles = sim_to_phys_angles(target_q, delta, offset_classmate);
        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

        % Draw Simulation Smoothly
        current_pos = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
        current_pitch_val = current_q(2) + delta + current_q(3) + current_q(4);
        num_steps = 30;

        traj_x = linspace(current_pos(1), X_in, num_steps);
        traj_y = linspace(current_pos(2), Y_in, num_steps);
        traj_z = linspace(current_pos(3), Z_in, num_steps);
        traj_pitch = linspace(current_pitch_val, goal_pitch, num_steps);

        fprintf('Moving arm...\n');
        for t = 1:num_steps
            [q1_t,q2_t,q3_t,q4_t,valid_t] = sim_inverse_kinematics( ...
                traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
                d1,a2,a3,L_tip_total,delta,joint_limits);

            if valid_t
                current_q = [q1_t, q2_t, q3_t, q4_t];

                % Update plot
                cla; hold on; grid on; axis equal; axis([-0.3 0.4 -0.4 0.4 0 0.6]); view(45, 30);
                plot3(X_in, Y_in, Z_in, 'r*', 'MarkerSize', 10, 'LineWidth', 2);
                plot_robot(current_q, d1,a2,a3,L_tip_total,delta, 0.04);
                drawnow;
            end
            % We don't pause much here so simulation runs concurrently with real motion
            pause(0.02);
        end

        % Snap to final state to correct slight interpolation errors
        current_q = target_q;
        fprintf('Target reached.\n');
    end
catch ME
    fprintf('Program interrupted: %s\n', ME.message);
end

%% ---- 5. CLEANUP & SHUTDOWN ---- %%
fprintf('\n--- Shutting Down ---\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque DISABLED.\n');
closePort(port_num);
fprintf('Port Closed.\n');
unloadlibrary(lib_name);
fprintf('Simulation Exited gracefully.\n');


%% ---- HELPER FUNCTIONS ---- %%

function phys_angles = sim_to_phys_angles(sim_q, delta, offset_classmate)
% Converts pure mathematical DH angles to the expected Physical Motor frame mapping
% Dofbot neutral (standing up) is 180 degrees (2048 ticks) for all motors.
q1 = sim_q(1);

% If the first motor ABOVE the base (Shoulder / ID 12) is rotating backwards,
% we flip the sign of its DH angle application:
q2 = -(sim_q(2) + delta - offset_classmate);

q3 = -(sim_q(3) + offset_classmate);
q4 = -sim_q(4);

% Pack and shift 180 degrees to center physical motors
phys_angles = [q1; q2; q3; q4; 0] + deg2rad(180);
end

function send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles)
% Sends array of radians to the real robot
ADDR_PRO_GOAL_POSITION = 116;
LEN_GOAL_POSITION = 4;
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);

for k = 1:5
    % Clip to [0, 2pi] roughly
    deg_val = rad2deg(phys_angles(k));
    pos_tick = round(deg_val * (4096 / 360));
    pos_tick = max(0, min(4095, pos_tick));

    groupSyncWriteAddParam(groupwrite_pos, IDs(k), typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
end
groupSyncWriteTxPacket(groupwrite_pos);
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

function [theta1,theta2,theta3,theta4,isValid] = sim_inverse_kinematics(x,y,z,phi,d1,a2,a3,L4,delta,limits)
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
