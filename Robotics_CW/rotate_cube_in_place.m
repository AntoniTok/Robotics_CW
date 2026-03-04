function rotate_cube_in_place(k, cube_gx, cube_gy, port_num, PROTOCOL_VERSION, IDs)
% ROTATE_CUBE_IN_PLACE  Pick up cube vertically, place it back horizontally.
%
%   rotate_cube_in_place(k, cube_gx, cube_gy, port_num, PROTOCOL_VERSION, IDs)
%
%   Pick:  pitch = -pi/2  (gripper pointing straight down)
%   Place: pitch = 0      (gripper horizontal)
%   The cube stays in the same (gx, gy) position. Repeated k times.

%% ---- Robot geometry (must match your main script) ----
d1          = 0.077;
a2          = sqrt(0.128^2 + 0.024^2);
delta       = atan2(0.024, 0.128);
a3          = 0.124;
a4          = 0.126;
L_finger    = 0.02;
L_tip_total = a4 + L_finger;

offset_classmate = deg2rad(90 - rad2deg(delta));

joint_limits = [
    deg2rad(-180), deg2rad(180);
    deg2rad(-90)  + (offset_classmate - delta),  deg2rad(90) + (offset_classmate - delta);
    deg2rad(-75)  - offset_classmate,            deg2rad(85) - offset_classmate;
    deg2rad(-135),                               deg2rad(135)
];

%% ---- Grid / height constants ----
GRID_UNIT   = 0.025;
ROBOT_GX    = 9;
ROBOT_GY    = 3;
cube_height = 0.025;
pick_z      = cube_height/2 + 0.018;  % lower grab
place_z     = cube_height/2 + 0.030;
hover_z     = 0.04;
rotate_z    = 0.10;  % extra height for safe wrist rotation

%% ---- Gripper angles ----
GRIPPER_OPEN  = deg2rad(-45);
GRIPPER_CLOSE = deg2rad(20);

%% ---- Convert cube grid position to world ----
[cx, cy, ~] = grid_to_world(cube_gx, cube_gy, 0, GRID_UNIT, ROBOT_GX, ROBOT_GY);

% Compensate place position: horizontal gripper shifts cube outward by ~L_finger
% Pull place coords closer to robot base to keep cube centered on holder
theta_cube = atan2(cy, cx);
place_cx = cx - L_finger * cos(theta_cube);
place_cy = cy - L_finger * sin(theta_cube);

%% ---- Find valid pick pitch (search like original script) ----
pitch_pick = [];
test_angles = deg2rad(-90 : 5 : -30);
for angle = test_angles
    [~,~,~,~,v1] = inverse_kinematics(cx,cy,pick_z,           angle, d1,a2,a3,L_tip_total,delta,joint_limits);
    [~,~,~,~,v2] = inverse_kinematics(cx,cy,pick_z+hover_z,   angle, d1,a2,a3,L_tip_total,delta,joint_limits);
    if v1 && v2
        pitch_pick = angle;
        fprintf('  Pick pitch found: %.1f deg\n', rad2deg(pitch_pick));
        break;
    end
end
if isempty(pitch_pick)
    error('No valid pick pitch for cube at grid(%d,%d).', cube_gx, cube_gy);
end

%% ---- Find valid place pitch (search from 0 toward pick pitch) ----
pitch_place = [];
test_angles_place = deg2rad(0 : -5 : -90);
for angle = test_angles_place
    [~,~,~,~,v1] = inverse_kinematics(cx,cy,place_z,           angle, d1,a2,a3,L_tip_total,delta,joint_limits);
    [~,~,~,~,v2] = inverse_kinematics(cx,cy,place_z+hover_z,   angle, d1,a2,a3,L_tip_total,delta,joint_limits);
    [~,~,~,~,v3] = inverse_kinematics(cx,cy,pick_z+rotate_z,   angle, d1,a2,a3,L_tip_total,delta,joint_limits);
    if v1 && v2 && v3
        pitch_place = angle;
        fprintf('  Place pitch found: %.1f deg\n', rad2deg(pitch_place));
        break;
    end
end
if isempty(pitch_place)
    error('No valid place pitch for cube at grid(%d,%d).', cube_gx, cube_gy);
end

%% ---- Rotation loop ----
for iter = 1:k
    fprintf('--- Rotation %d / %d ---\n', iter, k);

    % Waypoints: [x, y, z, pitch, action]
    %   action: 0 = no gripper change, 1 = close (pick), 2 = open (place)
    % Offset place position 0.02m radially outward from robot
    [ox, oy] = radial_offset(cx, cy, 0.02);

    waypoints = [
        cx, cy, pick_z + hover_z,    pitch_pick,  0;   % 1 hover above (vertical)
        cx, cy, pick_z,              pitch_pick,  1;   % 2 descend & grip (vertical)
        cx, cy, pick_z + rotate_z,   pitch_pick,  0;   % 3 lift high
        cx, cy, pick_z + rotate_z,   pitch_place, 0;   % 4 rotate wrist at safe height
        ox, oy, place_z + hover_z,   pitch_place, 0;   % 5 move outward + lower
        ox, oy, place_z,             pitch_place, 2;   % 6 descend & release (horizontal)
        ox, oy, place_z + hover_z,   pitch_place, 0;   % 7 retract
    ];

    current_gripper = GRIPPER_OPEN;

    % Start from hover above cube
    current_q = ik_or_die(cx, cy, pick_z + hover_z, pitch_pick, d1,a2,a3,L_tip_total,delta,joint_limits);
    send_arm(port_num, PROTOCOL_VERSION, IDs, current_q, current_gripper, delta, offset_classmate);
    pause(1);

    for wp = 1:size(waypoints,1)
        gx = waypoints(wp,1); gy = waypoints(wp,2); gz = waypoints(wp,3);
        gp = waypoints(wp,4); act = waypoints(wp,5);

        cur_pos   = forward_kinematics(current_q, d1,a2,a3,L_tip_total,delta);
        cur_pitch = current_q(2) + delta + current_q(3) + current_q(4);
        N = 30;
        for t = 1:N
            frac = t / N;
            ix = cur_pos(1) + (gx - cur_pos(1)) * frac;
            iy = cur_pos(2) + (gy - cur_pos(2)) * frac;
            iz = cur_pos(3) + (gz - cur_pos(3)) * frac;
            ip = cur_pitch  + (gp - cur_pitch)  * frac;

            [q1,q2,q3,q4,ok] = inverse_kinematics(ix,iy,iz,ip, ...
                                    d1,a2,a3,L_tip_total,delta,joint_limits);
            if ok
                current_q = [q1,q2,q3,q4];
                send_arm(port_num, PROTOCOL_VERSION, IDs, current_q, ...
                         current_gripper, delta, offset_classmate);
                pause(0.01);
            end
        end

        % Gripper action
        if act == 1      % CLOSE (pick)
            current_gripper = GRIPPER_CLOSE;
            send_arm(port_num, PROTOCOL_VERSION, IDs, current_q, current_gripper, delta, offset_classmate);
            pause(0.5);
        elseif act == 2  % OPEN (place)
            current_gripper = GRIPPER_OPEN;
            send_arm(port_num, PROTOCOL_VERSION, IDs, current_q, current_gripper, delta, offset_classmate);
            pause(0.5);
        end
    end

    fprintf('  Rotation %d complete.\n', iter);
end

fprintf('All %d rotation(s) done.\n', k);
end

%% =====================  LOCAL HELPERS  =====================

function [ox, oy] = radial_offset(x, y, dist)
% Offsets point (x,y) by dist meters radially outward from robot origin (0,0)
    theta = atan2(y, x);
    r = sqrt(x^2 + y^2) + dist;
    ox = r * cos(theta);
    oy = r * sin(theta);
end

function q = ik_or_die(x,y,z,phi, d1,a2,a3,L4,delta,limits)
    [q1,q2,q3,q4,ok] = inverse_kinematics(x,y,z,phi,d1,a2,a3,L4,delta,limits);
    if ~ok, error('IK unreachable at [%.3f %.3f %.3f] pitch=%.1f deg', x,y,z,rad2deg(phi)); end
    q = [q1,q2,q3,q4];
end

function send_arm(port_num, PVER, IDs, q, grip, delta, offset_classmate)
    phys = sim_to_phys_angles(q, grip, delta, offset_classmate);
    send_to_robot(port_num, PVER, IDs, phys);
end

function phys_angles = sim_to_phys_angles(sim_q, gripper_q, delta, offset_classmate)
    q1 = sim_q(1);
    q2 = -(sim_q(2) + delta - offset_classmate);
    q3 = -(sim_q(3) + offset_classmate);
    q4 = -sim_q(4);
    q5 = gripper_q;
    phys_angles = [q1; q2; q3; q4; q5] + deg2rad(180);
end

function send_to_robot(port_num, PROTOCOL_VERSION, IDs, phys_angles)
    ADDR = 116; LEN = 4;
    gw = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR, LEN);
    for k = 1:5
        pos_tick = round(rad2deg(phys_angles(k)) * (4096/360));
        pos_tick = max(0, min(4095, pos_tick));
        groupSyncWriteAddParam(gw, IDs(k), typecast(int32(pos_tick),'uint32'), LEN);
    end
    groupSyncWriteTxPacket(gw);
end

function [wx,wy,wz] = grid_to_world(gx,gy,gz_scale,unit,r_gx,r_gy)
    wx = (gy  - r_gy) * unit;
    wy = (r_gx - gx)  * unit;
    wz = gz_scale      * unit;
end

function pos = forward_kinematics(q, d1,a2,a3,L4,delta)
    T = dh(q(1),d1,0,pi/2) * dh(q(2)+delta,0,a2,0) * dh(q(3),0,a3,0) * dh(q(4),0,L4,0);
    pos = T(1:3,4);
end

function [t1,t2,t3,t4,ok] = inverse_kinematics(x,y,z,phi,d1,a2,a3,L4,delta,limits)
    ok = true; t1 = atan2(y,x);
    rw = sqrt(x^2+y^2) - L4*cos(phi);
    zw = z - d1        - L4*sin(phi);
    D  = rw^2 + zw^2;
    c3 = (D - a2^2 - a3^2)/(2*a2*a3);
    if abs(c3)>1, ok=false; t1=0;t2=0;t3=0;t4=0; return; end
    t3 = atan2(-sqrt(1-c3^2), c3);
    al = atan2(zw, rw);
    cb = (a2^2+D-a3^2)/(2*a2*sqrt(D)); if abs(cb)>1, cb=sign(cb); end
    lk2 = al + acos(cb);
    t2  = lk2 - delta;
    t4  = phi - (lk2 + t3);
    if t1<limits(1,1)||t1>limits(1,2)||t2<limits(2,1)||t2>limits(2,2)|| ...
       t3<limits(3,1)||t3>limits(3,2)||t4<limits(4,1)||t4>limits(4,2)
        ok = false;
    end
end

function T = dh(theta,d,a,alpha)
    ct=cos(theta); st=sin(theta); ca=cos(alpha); sa=sin(alpha);
    T = [ct -st*ca  st*sa a*ct;
         st  ct*ca -ct*sa a*st;
         0   sa     ca    d;
         0   0      0     1];
end