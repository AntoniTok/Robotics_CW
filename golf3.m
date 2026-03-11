% task2a_golf_swing.m
% Executes a parabolic golf-swing motion on the Dofbot arm.
%
% Trajectory (all in metres):
%   Start  : x=0.05, y=0.05, z=0.28  | pitch =  0 deg
%   Bottom : x=0.09, y=0.10, z=0.12  | pitch = -45 deg  (impact point)
%   Finish : x=0.25, y=0.05, z=0.28  | pitch = -45 deg  (follow-through)
%
% A quadratic Bezier is used so the arc passes exactly through all three
% poses.  Pitch runs 0 -> -45 deg and STAYS down through follow-through,
% which keeps theta2 smaller / theta3 larger (less forward reach) and
% corrects the too-far-forward bias.  A boosted velocity profile is applied
% for the swing phase.

clear; clc; close all;

%% ── 1. DYNAMIXEL SETUP ──────────────────────────────────────────────────
fprintf('Initialising Dynamixel SDK...\n');
lib_name = '';
if strcmp(computer,'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer,'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer,'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer,'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer,'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader','port_handler.h', ...
        'addheader','packet_handler.h', ...
        'addheader','group_sync_write.h', ...
        'addheader','group_sync_read.h');
end

% Control table addresses
ADDR_TORQUE_ENABLE      = 64;
ADDR_PROFILE_ACCEL      = 108;
ADDR_PROFILE_VEL        = 112;
ADDR_GOAL_POSITION      = 116;
LEN_GOAL_POSITION       = 4;
PROTOCOL_VERSION        = 2.0;

IDs      = [11, 12, 13, 14, 15];   % Base, Shoulder, Elbow, Wrist, Gripper
ID_GRIPPER = 15;
BAUDRATE = 1000000;
DEVICENAME = 'COM7';               % ← adjust if needed

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

% ── Velocity / acceleration profiles ────────────────────────────────────
NORMAL_VEL   = 150;   % ticks/s – used while moving to start & returning home
SWING_VEL    = 900;   % ticks/s – boosted for the swing arc  (max ≈ 1023)
NORMAL_ACCEL = 30;
SWING_ACCEL  = 80;    % higher accel so the fast swing ramps up quickly

% Pause between swing keyframes (seconds).
% Gives motors time to begin moving before the next goal is sent.
% Tune this: shorter = faster swing but risks skipped waypoints.
SWING_KEYFRAME_PAUSE = 0.06;

global MOTOR_11_OFFSET;
MOTOR_11_OFFSET = deg2rad(2);

% ── Open port ────────────────────────────────────────────────────────────
port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    error('Failed to open port %s.', DEVICENAME);
end
if ~setBaudRate(port_num, BAUDRATE)
    closePort(port_num); error('Failed to set baudrate.');
end
fprintf('Port open.\n');  pause(0.3);

% Enable torque + set normal profiles on all joints
set_profiles(port_num, PROTOCOL_VERSION, IDs, NORMAL_ACCEL, NORMAL_VEL);
fprintf('Torque & normal profiles enabled.\n');

%% ── 2. ROBOT KINEMATIC PARAMETERS ──────────────────────────────────────
d1       = 0.077;
a2       = sqrt(0.128^2 + 0.024^2);
delta    = atan2(0.024, 0.128);
a3       = 0.124;
a4       = 0.126;
L_finger = 0.025;
L_tip    = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));
shift_q2 = offset_classmate - delta;
shift_q3 = -offset_classmate;
joint_limits = [
    deg2rad(-180),              deg2rad(180);
    deg2rad(-90) + shift_q2,    deg2rad(90)  + shift_q2;
    deg2rad(-75) + shift_q3,    deg2rad(85)  + shift_q3;
    deg2rad(-135),              deg2rad(135)
];

GRIPPER_OPEN  = deg2rad(-45);  % used only for home / return
GRIPPER_CLOSE = deg2rad(20);   % closed for the entire swing

%% ── 3. SWING TRAJECTORY DEFINITION ─────────────────────────────────────
P0 = [0.08, -0.14,  0.26];   pitch0 =  0;       % backswing
Pm = [0.15,  0.00,  0.06];   pitchM = -pi/4;    % impact
P1 = [0.08,  0.18,  0.26];   pitch1 =  0;       % follow-through

A = 2*(P0 + P1) - 4*Pm;
B = (P1 - P0) - A;
C = P0;

ap = 2*(pitch0 + pitch1) - 4*pitchM;
bp = (pitch1 - pitch0) - ap;
cp = pitch0;

%% ── 4. PRE-FLIGHT IK CHECK ──────────────────────────────────────────────
% Verify every point along the trajectory is reachable BEFORE moving.
% Any failures are printed with their xyz/pitch so you can adjust the Ps.
fprintf('\n--- Pre-flight IK check (20 samples) ---\n');
t_check = linspace(0, 1, 20);
any_fail = false;
for idx = 1:length(t_check)
    t  = t_check(idx);
    tx = A(1)*t^2 + B(1)*t + C(1);
    ty = A(2)*t^2 + B(2)*t + C(2);
    tz = A(3)*t^2 + B(3)*t + C(3);
    tp = ap*t^2   + bp*t   + cp;
    [~,~,~,~,ok] = inverse_kinematics(tx,ty,tz,tp, ...
                       d1,a2,a3,L_tip,delta,joint_limits);
    if ~ok
        fprintf('  IK FAIL  t=%.2f  xyz=[%.3f  %.3f  %.3f]  pitch=%.1f deg\n', ...
            t, tx, ty, tz, rad2deg(tp));
        any_fail = true;
    end
end
if any_fail
    error(['Pre-flight check failed – adjust P0/Pm/P1 before running on hardware.\n' ...
           'See printed failures above.']);
else
    fprintf('  All 20 samples reachable.\n');
end
fprintf('--- End pre-flight check ---\n\n');

%% ── 5. VISUALISATION SETUP ──────────────────────────────────────────────
fig = figure('Name','Golf Swing','Color','w','Position',[100 100 900 650]);
view(20, 25); axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis([-0.05 0.42 -0.35 0.35 0 0.42]);
title('Parabolic Golf Swing');

t_ref = linspace(0,1,200);
ref_pts = (A' * t_ref.^2) + (B' * t_ref) + C';
plot3(ref_pts(1,:), ref_pts(2,:), ref_pts(3,:), '--', ...
    'Color',[0.6 0.6 0.6], 'LineWidth',1.5, 'DisplayName','Ideal arc');
plot3([P0(1) Pm(1) P1(1)], [P0(2) Pm(2) P1(2)], [P0(3) Pm(3) P1(3)], ...
    'ko', 'MarkerSize',10, 'MarkerFaceColor','y', 'DisplayName','Key poses');
legend('Location','northeast');
drawnow;

%% ── 6. MOVE TO HOME THEN SWING START ────────────────────────────────────
HOME = [0.20, 0.00, 0.25];  home_pitch = 0;

[q1,q2,q3,q4,ok] = inverse_kinematics(HOME(1),HOME(2),HOME(3),home_pitch,...
    d1,a2,a3,L_tip,delta,joint_limits);
if ~ok, error('Home position unreachable!'); end
current_q = [q1,q2,q3,q4];

fprintf('Moving to home...\n');
phys = sim_to_phys(current_q, GRIPPER_CLOSE, delta, offset_classmate);
send_angles(port_num, PROTOCOL_VERSION, IDs, phys);
plot_robot_vis(current_q, d1,a2,a3,L_tip,delta);
drawnow;  pause(2.5);

fprintf('Moving to swing start position...\n');
NUM_APPROACH = 20;
home_pos = forward_kinematics(current_q, d1,a2,a3,L_tip,delta);
home_pitch_val = sum([current_q(2)+delta, current_q(3), current_q(4)]);

for s = 1:NUM_APPROACH
    frac = s / NUM_APPROACH;
    tx = home_pos(1)*(1-frac) + P0(1)*frac;
    ty = home_pos(2)*(1-frac) + P0(2)*frac;
    tz = home_pos(3)*(1-frac) + P0(3)*frac;
    tp = home_pitch_val*(1-frac) + pitch0*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,...
        d1,a2,a3,L_tip,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys = sim_to_phys(current_q, GRIPPER_CLOSE, delta, offset_classmate);
    send_angles(port_num, PROTOCOL_VERSION, IDs, phys);
    plot_robot_vis(current_q, d1,a2,a3,L_tip,delta);
    drawnow;  pause(0.05);
end
fprintf('At swing start.  Boosting velocity to %d...\n', SWING_VEL);

%% ── 7. GOLF SWING ───────────────────────────────────────────────────────
set_profiles(port_num, PROTOCOL_VERSION, IDs, SWING_ACCEL, SWING_VEL);
pause(0.2);

SWING_KEYFRAMES = 5;
t_swing = linspace(0, 1, SWING_KEYFRAMES);

fprintf('Executing swing (%d keyframes at vel=%d, pause=%.3fs between)...\n', ...
    SWING_KEYFRAMES, SWING_VEL, SWING_KEYFRAME_PAUSE);

for idx = 1:SWING_KEYFRAMES
    t  = t_swing(idx);

    tx = A(1)*t^2 + B(1)*t + C(1);
    ty = A(2)*t^2 + B(2)*t + C(2);
    tz = A(3)*t^2 + B(3)*t + C(3);
    tp = ap*t^2   + bp*t   + cp;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,...
        d1,a2,a3,L_tip,delta,joint_limits);
    if ~ok
        fprintf('  IK invalid at t=%.2f – skipping\n', t);
        continue;
    end
    current_q = [q1,q2,q3,q4];

    gripper_angle = GRIPPER_CLOSE * (1 - t) + deg2rad(0) * t;
    phys = sim_to_phys(current_q, gripper_angle, delta, offset_classmate);
    send_angles(port_num, PROTOCOL_VERSION, IDs, phys);

    % Short pause so each goal position is registered before the next is sent.
    % Without this, motors receive all goals in milliseconds and skip waypoints.
    pause(SWING_KEYFRAME_PAUSE);

    plot_robot_vis(current_q, d1,a2,a3,L_tip,delta);
    drawnow limitrate;
end

pause(1.0);
fprintf('Swing complete.\n');

%% ── 8. RETURN TO NEUTRAL ─────────────────────────────────────────────────
fprintf('Returning to neutral (normal speed)...\n');
set_profiles(port_num, PROTOCOL_VERSION, IDs, NORMAL_ACCEL, NORMAL_VEL);
pause(0.2);

HOME_HIGH = HOME;
REST      = [0.15, 0.00, 0.08];
rest_pitch = -pi/4;

[q1,q2,q3,q4,ok] = inverse_kinematics(HOME_HIGH(1),HOME_HIGH(2),HOME_HIGH(3),home_pitch,...
    d1,a2,a3,L_tip,delta,joint_limits);
if ~ok, error('Neutral position unreachable!'); end

follow_pos   = forward_kinematics(current_q, d1,a2,a3,L_tip,delta);
follow_pitch = current_q(2)+delta + current_q(3) + current_q(4);

% ── Phase A: follow-through → home ───────────────────────────────────────
NUM_RETURN = 25;
for s = 1:NUM_RETURN
    frac = s / NUM_RETURN;
    tx = follow_pos(1)*(1-frac) + HOME_HIGH(1)*frac;
    ty = follow_pos(2)*(1-frac) + HOME_HIGH(2)*frac;
    tz = follow_pos(3)*(1-frac) + HOME_HIGH(3)*frac;
    tp = follow_pitch*(1-frac)  + home_pitch*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,...
        d1,a2,a3,L_tip,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys = sim_to_phys(current_q, GRIPPER_OPEN, delta, offset_classmate);
    send_angles(port_num, PROTOCOL_VERSION, IDs, phys);
    plot_robot_vis(current_q, d1,a2,a3,L_tip,delta);
    drawnow;  pause(0.05);
end
pause(0.5);

% ── Phase B: home → low rest pose ────────────────────────────────────────
fprintf('Lowering to rest position...\n');
home_pos_now = forward_kinematics(current_q, d1,a2,a3,L_tip,delta);
home_pitch_now = current_q(2)+delta + current_q(3) + current_q(4);

NUM_LOWER = 20;
for s = 1:NUM_LOWER
    frac = s / NUM_LOWER;
    tx = home_pos_now(1)*(1-frac) + REST(1)*frac;
    ty = home_pos_now(2)*(1-frac) + REST(2)*frac;
    tz = home_pos_now(3)*(1-frac) + REST(3)*frac;
    tp = home_pitch_now*(1-frac)  + rest_pitch*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,...
        d1,a2,a3,L_tip,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys = sim_to_phys(current_q, GRIPPER_OPEN, delta, offset_classmate);
    send_angles(port_num, PROTOCOL_VERSION, IDs, phys);
    plot_robot_vis(current_q, d1,a2,a3,L_tip,delta);
    drawnow;  pause(0.05);
end
pause(1.5);

%% ── 9. SHUTDOWN ──────────────────────────────────────────────────────────
fprintf('\n--- Shutting down ---\n');
for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
end
fprintf('Torque DISABLED.\n');
closePort(port_num);
fprintf('Port closed.\n');
unloadlibrary(lib_name);


%% ════════════════════════════════════════════════════════════════════════
%%  HELPER FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

function set_profiles(port_num, PROTO, IDs, accel, vel)
    ADDR_TORQUE_ENABLE = 64;
    ADDR_PROFILE_ACCEL = 108;
    ADDR_PROFILE_VEL   = 112;
    TORQUE_ENABLE      = 1;
    for k = 1:length(IDs)
        write1ByteTxRx(port_num, PROTO, IDs(k), ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
        write4ByteTxRx(port_num, PROTO, IDs(k), ADDR_PROFILE_ACCEL, accel);
        write4ByteTxRx(port_num, PROTO, IDs(k), ADDR_PROFILE_VEL,   vel);
    end
end

function phys = sim_to_phys(sim_q, gripper_q, delta, offset_classmate)
    global MOTOR_11_OFFSET;
    q1 = sim_q(1) + MOTOR_11_OFFSET;
    q2 = -(sim_q(2) + delta - offset_classmate);
    q3 = -(sim_q(3) + offset_classmate);
    q4 = -sim_q(4);
    q5 = gripper_q;
    phys = [q1; q2; q3; q4; q5] + deg2rad(180);
end

function send_angles(port_num, PROTOCOL_VERSION, IDs, phys_angles)
    ADDR_GOAL_POSITION = 116;
    LEN_GOAL_POSITION  = 4;
    gw = groupSyncWrite(port_num, PROTOCOL_VERSION, ...
                        ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    for k = 1:5
        ticks = round(rad2deg(phys_angles(k)) * (4096/360));
        ticks = max(0, min(4095, ticks));
        groupSyncWriteAddParam(gw, IDs(k), ...
            typecast(int32(ticks),'uint32'), LEN_GOAL_POSITION);
    end
    groupSyncWriteTxPacket(gw);
end

function pos = forward_kinematics(q, d1,a2,a3,L4,delta)
    T = dh_matrix(q(1), d1, 0, pi/2) ...
      * dh_matrix(q(2)+delta, 0, a2, 0) ...
      * dh_matrix(q(3), 0, a3, 0) ...
      * dh_matrix(q(4), 0, L4, 0);
    pos = T(1:3,4);
end

function [t1,t2,t3,t4,ok] = inverse_kinematics(x,y,z,phi,...
                                                 d1,a2,a3,L4,delta,lims)
    ok = true;
    t1 = atan2(y, x);
    r  = sqrt(x^2 + y^2) - L4*cos(phi);
    zw = z - d1       - L4*sin(phi);
    D2 = r^2 + zw^2;
    c3 = (D2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(c3) > 1,  ok=false; t1=0;t2=0;t3=0;t4=0; return; end
    t3 = atan2(-sqrt(1-c3^2), c3);
    cb = (a2^2 + D2 - a3^2) / (2*a2*sqrt(D2));
    if abs(cb)>1, cb=sign(cb); end
    angle2 = atan2(zw,r) + acos(cb);
    t2 = angle2 - delta;
    t4 = phi - (angle2 + t3);
    if nargin >= 10 && ~isempty(lims)
        if t1<lims(1,1)||t1>lims(1,2)|| ...
           t2<lims(2,1)||t2>lims(2,2)|| ...
           t3<lims(3,1)||t3>lims(3,2)|| ...
           t4<lims(4,1)||t4>lims(4,2)
            ok = false;
        end
    end
end

function plot_robot_vis(q, d1,a2,a3,L4,delta)
    cla; hold on; grid on; axis equal;
    axis([-0.05 0.42 -0.35 0.35 0 0.42]);
    view(20,25);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Parabolic Golf Swing');

    T01 = dh_matrix(q(1),       d1, 0,  pi/2);
    T12 = dh_matrix(q(2)+delta,  0, a2, 0);
    T23 = dh_matrix(q(3),        0, a3, 0);
    T34 = dh_matrix(q(4),        0, L4, 0);
    T02 = T01*T12; T03 = T02*T23; T04 = T03*T34;

    pts = [zeros(3,1), T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4)];
    plot3(pts(1,:), pts(2,:), pts(3,:), '-k', 'LineWidth',3, ...
        'Marker','o','MarkerFaceColor','y','MarkerSize',7);

    plot3(T04(1,4), T04(2,4), T04(3,4), 'rp', ...
        'MarkerSize',14, 'MarkerFaceColor','r');
end

function T = dh_matrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end