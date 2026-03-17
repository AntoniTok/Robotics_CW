% golf_dynamic.m
% Swing Motion: Neutral → Close Gripper → 60° Pitch → Position → Swing Base -120°
% Phases:
%   1. Move to neutral/home position
%   2. Close gripper fully and hold
%   3. Tilt end-effector to -60° pitch (downward)
%   4. Move to target position (x=0, y=0.075, z=0.12) at -60° pitch
%   5. Swing base (theta1) -120° as fast as possible (velocity/accel from selected profile)

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
SWING_VEL   = 2000;
SWING_ACCEL = 500;

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
GRIPPER_CLOSE_FULL = deg2rad(45);   % Fully closed
GRIPPER_CLOSE_BALL = deg2rad(35);
GRIPPER_CLOSE_CLUB = deg2rad(35);

current_gripper = GRIPPER_OPEN;
attached_ball = false;
attached_club = false;

%% =========================================================
%% 3.1 GRID & SCENE CONFIGURATION
%% =========================================================
GRID_UNIT = 0.025;
GRID_W    = 17;
GRID_H    = 12;

ROBOT_GX  = 9;
ROBOT_GY  = 3;

% Ball holders
ball_holders = [
    17.5, 0.8;
    17.5, 2.8;
    17.5, 4.8
    ];
golf_tee = [9, 10.5];
golf_club_base = [3, 3];

% Object heights
ball_pick_z   = 0.028;
tee_place_z   = 0.030;
club_height   = 0.14;
club_pick_z   = 0.15;

%% =========================================================
%% 4. VISUALIZATION SETUP
%% =========================================================
fig = figure('Name','Swing Motion','Color','w','Position',[100 100 900 700]);
view(45, 30); axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis([-0.4 0.4 -0.4 0.4 0 0.5]);

%% =========================================================
%% BEGIN MAIN LOOP FOR 3 BALLS
%% =========================================================
for selected_ball_idx = 1:size(ball_holders, 1)
    fprintf('\n=========================================================\n');
    fprintf('=== STARTING SEQUENCE FOR BALL %d OF %d ===\n', selected_ball_idx, size(ball_holders,1));
    fprintf('=========================================================\n');

    % --- Interactive Input ---
    prompt = sprintf('\n>> Enter target distance for Ball %d (in cm, <= 0 or empty to exit): ', selected_ball_idx);
    target_dist_cm = input(prompt);

    if isempty(target_dist_cm) || target_dist_cm <= 0
        fprintf('  User requested exit or empty input. Exiting loop and shutting down.\n');
        break;
    end

    % --- Predefined Profiles for Specific Distances ---
    % 8 Valid targeting distances (cm): 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0
    valid_dists   = [2.5,   5.0,   7.5,   10.0,  12.5,  15.0,  17.5,  20.0];

    % YOU CAN TUNE THESE ARRAYS FOR REAL WORLD BEHAVIOR
    % Current defaults are just a starting proportional baseline
    profile_vel   = [50,   100,   120,   150,   180,   200,   1000,  1023];
    profile_accel = [40,    70,    100,   130,   160,   190,   220,   255];

    target_dist_cm = double(target_dist_cm);

    % Find the closest predefined distance if user typed something in between
    [~, closest_idx] = min(abs(valid_dists - target_dist_cm));

    matched_dist  = valid_dists(closest_idx);
    DYNAMIC_SWING_VEL   = profile_vel(closest_idx);
    DYNAMIC_SWING_ACCEL = profile_accel(closest_idx);

    if matched_dist ~= target_dist_cm
        fprintf('  [!] Input %.1f cm snapped to closest valid tier: %.1f cm\n', ...
            target_dist_cm, matched_dist);
    end

    fprintf('  Mapped %.1f cm to SWING_VEL=%d, SWING_ACCEL=%d\n', ...
        matched_dist, DYNAMIC_SWING_VEL, DYNAMIC_SWING_ACCEL);

    %% =========================================================
    %% PHASE 1: MOVE TO NEUTRAL HOME POSITION
    %% =========================================================
    fprintf('\n--- PHASE 1: Moving to Neutral Home ---\n');

    % Match task_golf_combined.m home direction (+X)
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
    %% PHASE 2: TASK A (PICK BALL) & TASK B (PICK CLUB)
    %% =========================================================
    fprintf('\n--- PHASE 2: Picking up Ball and Club ---\n');

    try
        %% ---------- TASK A: PICK ONE BALL AND PLACE IT ON THE TEE ----------
        [bx, by, ~] = grid_to_world(ball_holders(selected_ball_idx,1), ball_holders(selected_ball_idx,2), ...
            0, GRID_UNIT, ROBOT_GX, ROBOT_GY);
        [tx, ty, ~] = grid_to_world(golf_tee(1), golf_tee(2), ...
            0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

        hover_z = 0.05;

        fprintf('Picking ball %d from Grid(%d,%d)\n', selected_ball_idx, ...
            ball_holders(selected_ball_idx,1), ball_holders(selected_ball_idx,2));
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
                fprintf('  Valid pitch for ball task: %.1f deg\n', rad2deg(best_pitch));
                break;
            end
        end

        waypoints_ball = [
            bx, by, ball_pick_z + hover_z, best_pitch, 0;   % approach ball, gripper open
            bx, by, ball_pick_z,           best_pitch, 1;   % close on ball
            bx, by, ball_pick_z + hover_z, best_pitch, 2;   % lift ball
            tx, ty, tee_place_z + hover_z, best_pitch, 2;   % move above tee
            tx, ty, tee_place_z,           best_pitch, 3;   % release on tee
            tx, ty, tee_place_z + hover_z, best_pitch, 0;   % retract
            home_x, home_y, home_z,        0,          0;   % back toward home corridor
            ];

        [current_q, current_gripper, attached_ball, attached_club] = ...
            execute_waypoints(waypoints_ball, current_q, current_gripper, ...
            attached_ball, attached_club, ...
            port_num, PROTOCOL_VERSION, IDs, ...
            d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
            GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);

        %% ---------- TASK B: PICK THE GOLF CLUB ----------
        [cx, cy, ~] = grid_to_world(golf_club_base(1), golf_club_base(2), ...
            0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

        fprintf('Picking golf club at Grid(%d,%d), pick height = %.3f m\n', ...
            golf_club_base(1), golf_club_base(2), club_pick_z);

        best_pitch_club = -pi/2;
        % User wants roughly 60 degrees. Try angles close to 60.
        % Also relax the hover check slightly if necessary
        test_angles_club = deg2rad(-60:5:-45);
        fallback_angles_club = deg2rad(-75:5:-65);
        all_club_angles = [test_angles_club, fallback_angles_club];

        for angle = all_club_angles
            [~,~,~,~, v1] = inverse_kinematics(cx, cy, club_pick_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
            [~,~,~,~, v2] = inverse_kinematics(cx, cy, club_pick_z + hover_z, angle, d1,a2,a3,L_tip_total,delta,joint_limits);
            [~,~,~,~, v3] = inverse_kinematics(home_x, home_y, home_z, 0, d1,a2,a3,L_tip_total,delta,joint_limits);

            if v1 && v2 && v3
                best_pitch_club = angle;
                fprintf('  Valid pitch for club task: %.1f deg\n', rad2deg(best_pitch_club));
                break;
            end
        end

        if best_pitch_club == -pi/2
            fprintf('  WARNING: Could not find strict reachable angle for club pick. Using -60.\n');
            best_pitch_club = deg2rad(-60); % Force it, might jump or fail IK later
        end

        waypoints_club = [
            cx, cy, club_pick_z + hover_z, best_pitch_club, 0;   % approach club
            cx, cy, club_pick_z,           best_pitch_club, 4;   % close on club
            cx, cy, club_pick_z + hover_z, best_pitch_club, 5;   % lift club
            home_x, home_y, home_z,        0,               5;   % move away holding club
            ];

        [current_q, current_gripper, attached_ball, attached_club] = ...
            execute_waypoints(waypoints_club, current_q, current_gripper, ...
            attached_ball, attached_club, ...
            port_num, PROTOCOL_VERSION, IDs, ...
            d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
            GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);

    catch ME
        fprintf('Pickup program interrupted: %s\n', ME.message);
    end

    %% =========================================================
    %% PHASE 3: TRANSITION TO -60° PITCH (at home XY, same Z)
    %% =========================================================
    fprintf('\n--- PHASE 3: Tilting to -60 deg pitch ---\n');

    target_pitch = deg2rad(-60);  % Enforce 60 degree swing tilt
    num_steps    = 30;

    % Interpolate pitch from current value to -60 deg, keeping XY/Z at home
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
    fprintf('  Pitch at -60 deg. Settling...\n');
    pause(0.5);

    %% =========================================================
    %% PHASE 4: MOVE TO SWING POSITION (x=0, y=0.075, z=0.12)
    %% =========================================================
    fprintf('\n--- PHASE 4: Moving to Swing Position ---\n');

    % Target in world frame (Start from positive Y, +140mm left side)
    swing_x = 0.030;
    swing_y = 0.13;
    swing_z = 0.15;

    % NOTE: theta1 here = atan2(0.14, 0.03) roughly +78 deg, starting from positive Y
    [q1_sw, q2_sw, q3_sw, q4_sw, valid_sw] = inverse_kinematics( ...
        swing_x, swing_y, swing_z, target_pitch, ...
        d1, a2, a3, L_tip_total, delta, joint_limits);

    if ~valid_sw
        fprintf('  WARNING: Swing position unreachable — check IK. Trying relaxed pitch...\n');
        % Try slightly different pitches as fallback around 60 degrees
        for fallback_pitch = deg2rad(-50:-2:-75)
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
    %% PHASE 5: SWING — BASE ROTATES -120° AT MAXIMUM SPEED
    %% =========================================================
    fprintf('\n--- PHASE 5: SWING — Base -120 deg at MAX speed ---\n');

    % Boost ONLY the base motor (ID 11) to swing profile calculated from input
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_ACCELERATION, DYNAMIC_SWING_ACCEL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PROFILE_VELOCITY,     DYNAMIC_SWING_VEL);
    fprintf('  Base motor: vel=%d, accel=%d\n', DYNAMIC_SWING_VEL, DYNAMIC_SWING_ACCEL);

    % Compute target theta1 using configured swing delta from current base angle
    % Negative delta means clockwise rotation viewed from above
    % Swing to match P1 angle across +X axis (from +78 deg to -80 deg)
    swing_delta_deg = -120;  % Negative indicates clockwise swing (Left -> Right)
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

    % Return pointing safely in +X
    end_x = 0.05; end_y = 0.0; end_z = 0.10; end_pitch = 0;
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
    %% PHASE 7: RETURN CLUB TO HOLDER
    %% =========================================================
    fprintf('\n--- PHASE 7: Returning Club to Holder ---\n');

    % The club base position was already calculated in Phase 2
    [cx, cy, ~] = grid_to_world(golf_club_base(1), golf_club_base(2), ...
        0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

    % Use the same pitch we used to pick the club up
    % best_pitch_club was defined in Phase 2. If for some reason it's lost, we default to -60.
    if ~exist('best_pitch_club', 'var')
        best_pitch_club = deg2rad(-60);
    end

    hover_z = 0.05;

    waypoints_return_club = [
        cx, cy, club_pick_z + hover_z, best_pitch_club, 0;   % approach club holder
        cx, cy, club_pick_z,           best_pitch_club, 3;   % place and open gripper
        cx, cy, club_pick_z + hover_z, best_pitch_club, 0;   % lift away from club
        home_x, home_y, home_z,        0,               0;   % return to home safely
        ];

    try
        [current_q, current_gripper, attached_ball, attached_club] = ...
            execute_waypoints(waypoints_return_club, current_q, current_gripper, ...
            attached_ball, attached_club, ...
            port_num, PROTOCOL_VERSION, IDs, ...
            d1,a2,a3,L_tip_total,delta,joint_limits,offset_classmate, ...
            GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB);
        fprintf('  Club returned successfully.\n');
    catch ME
        fprintf('  Failed to return club: %s\n', ME.message);
    end

end % END OF BALL LOOP

%% =========================================================
%% PHASE 8: FINAL REST POSITION
%% =========================================================
fprintf('\n--- PHASE 8: Moving to Final Rest Position ---\n');
% Define a safe, relaxed position to park the arm before disabling torque
final_x = 0.1; final_y = 0.0; final_z = 0.10; final_pitch = -pi/4;

[q1f, q2f, q3f, q4f, valid_final] = inverse_kinematics( ...
    final_x, final_y, final_z, final_pitch, ...
    d1, a2, a3, L_tip_total, delta, joint_limits);

if valid_final
    % Move smoothly to the final rest position
    cur_pos   = forward_kinematics(current_q, d1, a2, a3, L_tip_total, delta);
    cur_pitch = current_q(2) + delta + current_q(3) + current_q(4);
    num_steps = 40;

    traj_x     = linspace(cur_pos(1), final_x,     num_steps);
    traj_y     = linspace(cur_pos(2), final_y,     num_steps);
    traj_z     = linspace(cur_pos(3), final_z,     num_steps);
    traj_pitch = linspace(cur_pitch,  final_pitch, num_steps);

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
    fprintf('  Arm resting safely at final position.\n');
    pause(1.0);
else
    fprintf('  WARNING: Final rest position unreachable, staying here.\n');
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

function [current_q, current_gripper, attached_ball, attached_club] = execute_waypoints( ...
    waypoints, current_q, current_gripper, attached_ball, attached_club, ...
    port_num, PROTOCOL_VERSION, IDs, ...
    d1,a2,a3,L4,delta,joint_limits,offset_classmate, ...
    GRIPPER_OPEN, GRIPPER_CLOSE_BALL, GRIPPER_CLOSE_CLUB)

if isempty(waypoints)
    return;
end
if size(waypoints,2) ~= 5
    error('execute_waypoints expects Nx5 waypoints.');
end
if numel(current_q) ~= 4
    error('execute_waypoints expects current_q to have 4 joints.');
end
if numel(IDs) < 5
    error('execute_waypoints expects at least 5 motor IDs.');
end

n_wp = size(waypoints,1);
i_wp = 1;
while i_wp <= n_wp
    wp_row      = waypoints(i_wp,:);
    goal_x      = wp_row(1);
    goal_y      = wp_row(2);
    goal_z      = wp_row(3);
    goal_pitch  = wp_row(4);
    act_code    = wp_row(5);

    curr_pos    = forward_kinematics(current_q, d1,a2,a3,L4,delta);
    curr_pitch  = current_q(2) + delta + current_q(3) + current_q(4);

    n_step      = 12;
    traj_x      = linspace(curr_pos(1), goal_x, n_step);
    traj_y      = linspace(curr_pos(2), goal_y, n_step);
    traj_z      = linspace(curr_pos(3), goal_z, n_step);
    traj_pitch  = linspace(curr_pitch, goal_pitch, n_step);

    for t = 1:n_step
        [q1_t,q2_t,q3_t,q4_t,is_ok] = inverse_kinematics( ...
            traj_x(t), traj_y(t), traj_z(t), traj_pitch(t), ...
            d1,a2,a3,L4,delta,joint_limits);

        if is_ok
            current_q = [q1_t, q2_t, q3_t, q4_t];

            if t == n_step
                % At the last interpolation point I handle gripper first, then
                % send the normal pose command. This avoids occasional one-frame
                % mismatch where the wrist reaches target but gripper lags.
                if act_code == 1
                    attached_ball = true;
                    attached_club = false;
                    current_gripper = GRIPPER_CLOSE_BALL;
                    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
                    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                    pause(0.3);
                elseif act_code == 3
                    attached_ball = false;
                    current_gripper = GRIPPER_OPEN;
                    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
                    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                    pause(0.3);
                elseif act_code == 4
                    attached_club = true;
                    attached_ball = false;
                    current_gripper = GRIPPER_CLOSE_CLUB;
                    phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
                    send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                    pause(0.3);
                end
            end

            phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
            send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);

            plot_robot_simple(current_q, d1,a2,a3,L4,delta);
            drawnow;

            if t == n_step && (act_code == 1 || act_code == 3 || act_code == 4)
                phys_angles = sim_to_phys_angles(current_q, current_gripper, delta, offset_classmate);
                send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles);
                pause(0.5);
            end

            pause(0.01);
        end
    end

    i_wp = i_wp + 1;
end
end

function [wx, wy, wz] = grid_to_world(gx, gy, gz_scale, unit, r_gx, r_gy)
if nargin ~= 6
    error('grid_to_world expects 6 inputs.');
end
if ~isscalar(gx) || ~isscalar(gy) || ~isscalar(gz_scale) || ...
        ~isscalar(unit) || ~isscalar(r_gx) || ~isscalar(r_gy)
    error('grid_to_world inputs must be scalar values.');
end
wx = (gy - r_gy) * unit;
wy = (r_gx - gx) * unit;
wz = gz_scale * unit;
end

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
if numel(IDs) < 5 || numel(phys_angles) < 5
    error('send_to_robot expects at least 5 IDs and 5 target angles.');
end
if any(~isfinite(phys_angles(1:5)))
    error('send_to_robot got non-finite angle values.');
end

ADDR_PRO_GOAL_POSITION = 116;
LEN_GOAL_POSITION      = 4;
groupwrite_pos = groupSyncWrite(port_num, PROTOCOL_VERSION, ...
    ADDR_PRO_GOAL_POSITION, LEN_GOAL_POSITION);
i = 1;
while i <= 5
    deg_val  = rad2deg(phys_angles(i));
    pos_tick = round(deg_val * (4096 / 360));
    pos_tick = max(0, min(4095, pos_tick));
    groupSyncWriteAddParam(groupwrite_pos, IDs(i), ...
        typecast(int32(pos_tick), 'uint32'), LEN_GOAL_POSITION);
    i = i + 1;
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

if any(~isfinite([x,y,z,phi,d1,a2,a3,L4,delta]))
    isValid = false; t1=0;t2=0;t3=0;t4=0; return;
end

t1      = atan2(y, x);
r_target = sqrt(x^2 + y^2);
z_target = z - d1;
r_w      = r_target - L4*cos(phi);
z_w      = z_target - L4*sin(phi);
D_sq     = r_w^2 + z_w^2;

if D_sq < 1e-12
    isValid = false; t1=0;t2=0;t3=0;t4=0; return;
end

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