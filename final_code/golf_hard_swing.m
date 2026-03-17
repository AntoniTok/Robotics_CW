lib_name = '';
if strcmp(computer, 'PCWIN'),       lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86'),  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'),  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [~, ~] = loadlibrary(lib_name, 'dynamixel_sdk.h','addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

ADDR_PRO_TORQUE_ENABLE         = 64;
ADDR_PRO_PROFILE_ACCELERATION  = 108;
ADDR_PRO_PROFILE_VELOCITY      = 112;
ADDR_PRO_GOAL_POSITION         = 116;

LEN_GOAL_POSITION = 4;
PROTOCOL_VERSION  = 2.0;

ID_GRIPPER = 15;
IDs        = [11, 12, 13, 14, 15];
BAUDRATE   = 1000000;
DEVICENAME = 'COM7'; 

TORQUE_ENABLE      = 1;
TORQUE_DISABLE     = 0;
NORMAL_VEL         = 150;
NORMAL_ACCEL       = 50;
GRIPPER_PROFILE_VEL = 200;
SWING_VEL            = 800;
SWING_ACCEL          = 100;
SWING_KEYFRAME_PAUSE = 0.06;

global MOTOR_11_OFFSET;
MOTOR_11_OFFSET = deg2rad(2);

port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    fprintf('Failed to open port %s.\n', DEVICENAME);
    unloadlibrary(lib_name);
    return;
end

fprintf('Port open\n');
pause(0.5);

for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, NORMAL_ACCEL);

    if IDs(k) == ID_GRIPPER
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, GRIPPER_PROFILE_VEL);
    else
        write4ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_PROFILE_VELOCITY, NORMAL_VEL);
    end
end

GRIPPER_OPEN       = deg2rad(-45);
GRIPPER_CLOSE_BALL = deg2rad(45);
GRIPPER_CLOSE_CLUB = deg2rad(30);

current_gripper = GRIPPER_OPEN;

GRID_UNIT = 0.025;
GRID_W    = 17;
GRID_H    = 12;

ROBOT_GX  = 9;
ROBOT_GY  = 3;

ball_holders = [
    17.5, 0.5;
    17, 2;
    17, 3
];

selected_ball_idx = 1;
golf_tee = [9, 12.5];
golf_club_base = [3, 3];

ball_pick_z   = 0.028;   
tee_place_z   = 0.030;   
club_height   = 0.13;    
club_pick_z   = 0.14;    

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
    deg2rad(-180),              deg2rad(180);
    deg2rad(-90)  + shift_q2,   deg2rad(90) + shift_q2;
    deg2rad(-75)  + shift_q3,   deg2rad(85) + shift_q3;
    deg2rad(-135),              deg2rad(135)
];

fig = figure('Name','Combined Golf Task','Color','w','Position',[100 100 900 650]);
view(45, 30); axis equal; grid on; hold on;
xlabel('World X'); ylabel('World Y'); zlabel('World Z');
axis([-0.35 0.40 -0.40 0.40 0 0.6]);

home_x = 0.30; home_y = 0.00; home_z = 0.20; home_pitch = 0;
[q1,q2,q3,q4,valid] = inverse_kinematics(home_x,home_y,home_z,home_pitch, ...
    d1,a2,a3,L_tip_total,delta,joint_limits);

if ~valid
    error('home position unreachable!');
end

current_q = [q1, q2, q3, q4];

phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

attached_ball = false;
attached_club = false;

plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, ...
    attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
pause(2);


P0 = [0.08,  0.18,  0.26];   pitch0 =  0;     
Pm = [0.18,  0.03,  0.10];   pitchM = -pi/4;   
P1 = [0.08, -0.14,  0.26];   pitch1 =  0; 
A = 2*(P0 + P1) - 4*Pm;
B = (P1 - P0) - A;
C = P0;

ap = 2*(pitch0 + pitch1) - 4*pitchM;
bp = (pitch1 - pitch0) - ap;
cp = pitch0;


t_check = linspace(0, 1, 20);
any_fail = false;
for idx = 1:length(t_check)
    t  = t_check(idx);
    tx = A(1)*t^2 + B(1)*t + C(1);
    ty = A(2)*t^2 + B(2)*t + C(2);
    tz = A(3)*t^2 + B(3)*t + C(3);
    tp = ap*t^2   + bp*t   + cp;
    [~,~,~,~,ok] = inverse_kinematics(tx,ty,tz,tp, ...
                       d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok
        fprintf('IK FAIL  t=%.2f  xyz=[%.3f  %.3f  %.3f]  pitch=%.1f deg\n', ...
            t, tx, ty, tz, rad2deg(tp));
        any_fail = true;
    end
end
if any_fail
    error('FAILED');
else
    fprintf('GOOD\n\n');
end


t_ref = linspace(0,1,100);
ref_pts = (A' * t_ref.^2) + (B' * t_ref) + C';
plot3(ref_pts(1,:), ref_pts(2,:), ref_pts(3,:), '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5, 'DisplayName','Ideal arc');


try
    [bx, by, ~] = grid_to_world(ball_holders(selected_ball_idx,1), ball_holders(selected_ball_idx,2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
    [tx, ty, ~] = grid_to_world(golf_tee(1), golf_tee(2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    hover_z = 0.05;

    fprintf('Picking ball %d from Grid(%d,%d)\n', selected_ball_idx, ball_holders(selected_ball_idx,1), ball_holders(selected_ball_idx,2));
    fprintf('Placing ball on tee at Grid(%d,%d)\n', golf_tee(1), golf_tee(2));

    best_pitch = -pi/2;
    test_angles = deg2rad(-90:5:-45);
    for angle = test_angles
        [~,~,~,~, v1] = inverse_kinematics(bx, by, ball_pick_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, v2] = inverse_kinematics(bx, by, ball_pick_z + hover_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, v3] = inverse_kinematics(tx, ty, tee_place_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, v4] = inverse_kinematics(tx, ty, tee_place_z + hover_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);

        if v1 && v2 && v3 && v4
            best_pitch = angle;
            fprintf('  Valid pitch for ball: %.1f deg\n', rad2deg(best_pitch));
            break;
        end
    end

    waypoints_ball = [
        bx, by, ball_pick_z + hover_z, best_pitch, 0, 0; 
        bx, by, ball_pick_z,           best_pitch, 1, 0; 
        bx, by, ball_pick_z + hover_z, best_pitch, 2, 0;  
        tx, ty, tee_place_z + hover_z, best_pitch, 2, 0;   
        tx, ty, tee_place_z,           best_pitch, 3, 0;  
        tx, ty, tee_place_z + hover_z, best_pitch, 0, 0;   
        home_x, home_y, home_z,        0,          0, 0;   
    ];

    [current_q, current_gripper, attached_ball, attached_club] = execute_waypoints(waypoints_ball, current_q, current_gripper, ...
        attached_ball, attached_club, selected_ball_idx, port_num, PROTOCOL_VERSION, IDs, d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
        ball_holders, golf_tee, golf_club_base, GRID_UNIT, ROBOT_GX, ROBOT_GY, GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);

    [cx, cy, ~] = grid_to_world(golf_club_base(1), golf_club_base(2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    fprintf('Picking golf club at (%d,%d), pick height = %.3f m\n', olf_club_base(1), golf_club_base(2), club_pick_z);

    best_pitch_club = -pi/2;
    for angle = test_angles
        [~,~,~,~, v1] = inverse_kinematics(cx, cy, club_pick_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, v2] = inverse_kinematics(cx, cy, club_pick_z + hover_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
        [~,~,~,~, v3] = inverse_kinematics(home_x, home_y, home_z, 0, d1,a2,a3,L_tip_total,delta,joint_limits);

        if v1 && v2 && v3
            best_pitch_club = angle;
            fprintf('  Valid pitch for club task: %.1f deg\n', rad2deg(best_pitch_club));
            break;
        end
    end

    waypoints_club = [
        cx, cy, club_pick_z + hover_z, best_pitch_club, 0, 1; 
        cx, cy, club_pick_z,           best_pitch_club, 4, 1;  
        cx, cy, club_pick_z + hover_z, best_pitch_club, 5, 1;   
        home_x, home_y, home_z,        0,               5, 1;   
    ];

    [current_q, current_gripper, attached_ball, attached_club] = execute_waypoints(waypoints_club, current_q, current_gripper, ...
        attached_ball, attached_club, selected_ball_idx, port_num, PROTOCOL_VERSION, IDs, d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
        ball_holders, golf_tee, golf_club_base, GRID_UNIT, ROBOT_GX, ROBOT_GY, GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);

catch ME
    fprintf('Pickup program interrupted: %s\n', ME.message);
end

HOME_HIGH = [0.20, 0.00, 0.25];  home_pitch = 0;

current_pos_val   = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
current_pitch_val = current_q(2)+delta + current_q(3) + current_q(4);

NUM_APPROACH = 20;
for s = 1:NUM_APPROACH
    frac = s / NUM_APPROACH;
    tx = current_pos_val(1)*(1-frac) + HOME_HIGH(1)*frac;
    ty = current_pos_val(2)*(1-frac) + HOME_HIGH(2)*frac;
    tz = current_pos_val(3)*(1-frac) + HOME_HIGH(3)*frac;
    tp = current_pitch_val*(1-frac) + home_pitch*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
    
    plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, ...
        attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    drawnow;  pause(0.05);
end
pause(0.5);

current_pos_val   = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
current_pitch_val = current_q(2)+delta + current_q(3) + current_q(4);

for s = 1:NUM_APPROACH
    frac = s / NUM_APPROACH;
    tx = current_pos_val(1)*(1-frac) + P0(1)*frac;
    ty = current_pos_val(2)*(1-frac) + P0(2)*frac;
    tz = current_pos_val(3)*(1-frac) + P0(3)*frac;
    tp = current_pitch_val*(1-frac) + pitch0*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp,d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

    plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    drawnow;  pause(0.05);
end


set_profiles(port_num, PROTOCOL_VERSION, IDs, SWING_ACCEL, SWING_VEL);
pause(0.2);

SWING_KEYFRAMES = 5;
t_swing = linspace(0, 1, SWING_KEYFRAMES);


for idx = 1:SWING_KEYFRAMES
    t  = t_swing(idx);

    tx = A(1)*t^2 + B(1)*t + C(1);
    ty = A(2)*t^2 + B(2)*t + C(2);
    tz = A(3)*t^2 + B(3)*t + C(3);
    tp = ap*t^2   + bp*t   + cp;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp, d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok
        fprintf('  IK invalid at t=%.2f – skipping\n', t);
        continue;
    end
    current_q = [q1,q2,q3,q4];

    % Impact ball logic (visual only)
    if t >= 0.5 && attached_ball == false
        attached_ball = false; 
    end

    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

    pause(SWING_KEYFRAME_PAUSE);

    if t >= 0.5
        selected_ball_idx = -1; % Disappear from tee
    end

    plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, ...
        attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    drawnow limitrate;
end

pause(1.0);
fprintf('Swing complete.\n');


set_profiles(port_num, PROTOCOL_VERSION, IDs, NORMAL_ACCEL, NORMAL_VEL);
pause(0.2);

REST      = [0.15, 0.00, 0.15];
rest_pitch = -pi/4;

follow_pos   = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
follow_pitch = current_q(2)+delta + current_q(3) + current_q(4);

NUM_RETURN = 25;
for s = 1:NUM_RETURN
    frac = s / NUM_RETURN;
    tx = follow_pos(1)*(1-frac) + HOME_HIGH(1)*frac;
    ty = follow_pos(2)*(1-frac) + HOME_HIGH(2)*frac;
    tz = follow_pos(3)*(1-frac) + HOME_HIGH(3)*frac;
    tp = follow_pitch*(1-frac)  + home_pitch*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp, d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
    
    plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    drawnow;  pause(0.05);
end
pause(0.5);

home_pos_now = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
home_pitch_now = current_q(2)+delta + current_q(3) + current_q(4);

NUM_LOWER = 20;
for s = 1:NUM_LOWER
    frac = s / NUM_LOWER;
    tx = home_pos_now(1)*(1-frac) + REST(1)*frac;
    ty = home_pos_now(2)*(1-frac) + REST(2)*frac;
    tz = home_pos_now(3)*(1-frac) + REST(3)*frac;
    tp = home_pitch_now*(1-frac)  + rest_pitch*frac;

    [q1,q2,q3,q4,ok] = inverse_kinematics(tx,ty,tz,tp, d1,a2,a3,L_tip_total,delta,joint_limits);
    if ~ok, continue; end
    current_q = [q1,q2,q3,q4];

    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
    
    plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
    drawnow;  pause(0.05);
end
pause(1.5);


[cx, cy, ~] = grid_to_world(golf_club_base(1), golf_club_base(2), 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

if ~exist('best_pitch_club', 'var')
    best_pitch_club = deg2rad(-60);
end

hover_z = 0.05;

waypoints_return_club = [
    cx, cy, club_pick_z + hover_z, best_pitch_club, 0, 1;  
    cx, cy, club_pick_z,           best_pitch_club, 3, 1; 
    cx, cy, club_pick_z + hover_z, best_pitch_club, 0, 1;  
    home_x, home_y, home_z,        0,               0, 1;  
];

try
    [current_q, current_gripper, attached_ball, attached_club] = execute_waypoints(waypoints_return_club, current_q, current_gripper, ...
        attached_ball, attached_club, selected_ball_idx, port_num, PROTOCOL_VERSION, IDs, d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
        ball_holders, golf_tee, golf_club_base, GRID_UNIT, ROBOT_GX, ROBOT_GY, GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);
    fprintf('  Club returned successfully.\n');
catch ME
    fprintf('  Failed to return club: %s\n', ME.message);
end


final_x = 0.1; final_y = 0.0; final_z = 0.10; final_pitch = -pi/4;

[q1f, q2f, q3f, q4f, valid_final] = inverse_kinematics(final_x, final_y, final_z, final_pitch, d1, a2, a3, L_tip_total, delta, joint_limits);

if valid_final
    cur_pos   = forward_kinematics(current_q, d1, a2, a3, L_tip_total, delta);
    cur_pitch = current_q(2) + delta + current_q(3) + current_q(4);
    num_steps = 40;

    traj_x     = linspace(cur_pos(1), final_x,     num_steps);
    traj_y     = linspace(cur_pos(2), final_y,     num_steps);
    traj_z     = linspace(cur_pos(3), final_z,     num_steps);
    traj_pitch = linspace(cur_pitch,  final_pitch, num_steps);

    for t = 1:num_steps
        [q1t, q2t, q3t, q4t, vt] = inverse_kinematics(traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), d1, a2, a3, L_tip_total, delta, joint_limits);
        if vt
            current_q = [q1t, q2t, q3t, q4t];
            phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_club);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
            plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, ...
                attached_ball, attached_club, d1,a2,a3,L_tip_total,delta,GRID_UNIT,ROBOT_GX,ROBOT_GY);
            drawnow; pause(0.01);
        end
    end
    pause(1.0);
else
    fprintf('  WARNING: Final rest position unreachable, staying here.\n');
end


for k = 1:length(IDs)
    write1ByteTxRx(port_num, PROTOCOL_VERSION, IDs(k), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end
closePort(port_num);
unloadlibrary(lib_name);



function set_profiles(port_num, PROTO, IDs, accel, vel)
    ADDR_PRO_PROFILE_ACCELERATION = 108;
    ADDR_PRO_PROFILE_VELOCITY   = 112;
    for k = 1:length(IDs)
        write4ByteTxRx(port_num, PROTO, IDs(k), ADDR_PRO_PROFILE_ACCELERATION, accel);
        write4ByteTxRx(port_num, PROTO, IDs(k), ADDR_PRO_PROFILE_VELOCITY,   vel);
    end
end

function [current_q, current_gripper, attached_ball, attached_club] = execute_waypoints( ...
    waypoints, current_q, current_gripper, attached_ball, attached_club, selected_ball_idx, ...
    port_num, PROTOCOL_VERSION, IDs, d1,a2,a3,L4,delta,joint_limits,offset_classmate, ...
    ball_holders, golf_tee, golf_club_base, unit, rx, ry, GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB)

for wp_idx = 1:size(waypoints,1)
    target      = waypoints(wp_idx,:);
    goal_x      = target(1);
    goal_y      = target(2);
    goal_z      = target(3);
    goal_pitch  = target(4);
    action      = target(5);

    current_pos       = forward_kinematics(current_q, d1,a2,a3,L4,delta);
    current_pitch_val = current_q(2) + delta + current_q(3) + current_q(4);

    num_steps  = 12;
    traj_x     = linspace(current_pos(1), goal_x, num_steps);
    traj_y     = linspace(current_pos(2), goal_y, num_steps);
    traj_z     = linspace(current_pos(3), goal_z, num_steps);
    traj_pitch = linspace(current_pitch_val, goal_pitch, num_steps);

    for t = 1:num_steps
        [q1_t,q2_t,q3_t,q4_t,valid_t] = inverse_kinematics(traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), d1,a2,a3,L4,delta,joint_limits);

        if valid_t
            current_q = [q1_t, q2_t, q3_t, q4_t];

            if t == num_steps
                switch action
                    case 1 
                        attached_ball = true;
                        attached_club = false;
                        current_gripper = GRIPPER_CLOSE_BALL;
                        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_ball || attached_club);
                        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                        pause(0.3);

                    case 3 
                        attached_ball = false;
                        current_gripper = GRIPPER_OPEN;
                        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, false);
                        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                        pause(0.3);

                    case 4 
                        attached_club = true;
                        attached_ball = false;
                        current_gripper = GRIPPER_CLOSE_CLUB;
                        phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, true);
                        send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                        pause(0.3);

                    otherwise
                end
            end

            phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_ball || attached_club);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

            plot_scene_golf(current_q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, attached_ball, attached_club, d1,a2,a3,L4,delta,unit,rx,ry);
            drawnow;

            if t == num_steps && ismember(action,[1 3 4])
                phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate, attached_ball || attached_club);
                send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                pause(0.5);
            end

            pause(0.01);
        end
    end
end
end

function phys_angles = sim_to_phys_angles(sim_q, gripper_q, delta, offset_classmate, is_holding)
global MOTOR_11_OFFSET;

if is_holding
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
wx = (gy - r_gy) * unit;
wy = (r_gx - gx) * unit;
wz = gz_scale * unit;
end

function plot_scene_golf(q, ball_holders, selected_ball_idx, golf_tee, golf_club_base, ...
    attached_ball, attached_club, d1,a2,a3,L4,delta,unit,rx,ry)

cla; hold on; grid on; axis equal;
axis([-0.35 0.40 -0.40 0.40 0 0.6]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

P_tip = plot_robot(q, d1,a2,a3,L4,delta, 0.04);


for k = 1:size(ball_holders,1)
    [hx, hy, ~] = grid_to_world(ball_holders(k,1), ball_holders(k,2), 0, unit, rx, ry);
    plot3(hx, hy, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
end


if selected_ball_idx > 0
    for k = 1:size(ball_holders,1)
        [bx, by, ~] = grid_to_world(ball_holders(k,1), ball_holders(k,2), 0, unit, rx, ry);

        if k == selected_ball_idx && attached_ball
            plot3(P_tip(1), P_tip(2), P_tip(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        elseif k == selected_ball_idx && ~attached_ball
            plot3(bx, by, 0.02, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        else
            plot3(bx, by, 0.02, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        end
    end
else
    for k = 1:size(ball_holders,1)
        [bx, by, ~] = grid_to_world(ball_holders(k,1), ball_holders(k,2), 0, unit, rx, ry);
        if k ~= 1
             plot3(bx, by, 0.02, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        end
    end
end


[tx, ty, ~] = grid_to_world(golf_tee(1), golf_tee(2), 0, unit, rx, ry);
plot3(tx, ty, 0, '^', 'MarkerSize', 12, 'LineWidth', 2, 'Color', [0.2 0.2 0.2]);


[cx, cy, ~] = grid_to_world(golf_club_base(1), golf_club_base(2), 0, unit, rx, ry);

if attached_club
    line([P_tip(1) P_tip(1)], [P_tip(2) P_tip(2)], [P_tip(3)-0.09 P_tip(3)+0.01], 'Color', [0.85 0.65 0.13], 'LineWidth', 4);
else
    line([cx cx], [cy cy], [0 0.10], 'Color', [0.85 0.65 0.13], 'LineWidth', 4);
end

if selected_ball_idx > 0 && ~attached_ball
    plot3(tx, ty, 0.03, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end

end

function pos = forward_kinematics(q, d1,a2,a3,L4,delta)
t1=q(1); t2=q(2); t3=q(3); t4=q(4);
T01 = dh_matrix(t1,          d1, 0,  pi/2);
T12 = dh_matrix(t2 + delta,  0,  a2, 0);
T23 = dh_matrix(t3,          0,  a3, 0);
T34 = dh_matrix(t4,          0,  L4, 0);
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
    isValid = false; theta1=0; theta2=0; theta3=0; theta4=0; return;
end

sin_t3 = -sqrt(1 - cos_t3^2);
theta3 = atan2(sin_t3, cos_t3);

alpha = atan2(z_w, r_w);
cos_b = (a2^2 + D_sq - a3^2) / (2*a2*sqrt(D_sq));
if abs(cos_b) > 1
    cos_b = sign(cos_b);
end
beta = acos(cos_b);

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
T01 = dh_matrix(t1,         d1, 0,  pi/2);
T12 = dh_matrix(t2+delta,   0,  a2, 0);
T23 = dh_matrix(t3,         0,  a3, 0);
T34 = dh_matrix(t4,         0,  L4, 0);
T02 = T01*T12; T03 = T02*T23; T04 = T03*T34;

pts = [[0;0;0], T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4)];
plot3(pts(1,:), pts(2,:), pts(3,:), '-k', 'LineWidth', 3, 'Marker','o','MarkerFaceColor','y','MarkerSize',6);

plot_frame(eye(4), s);
plot_frame(T01, s);
plot_frame(T02, s);
plot_frame(T03, s);
plot_frame(T04, s);

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