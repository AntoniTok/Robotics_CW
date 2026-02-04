% Robot parameters
alpha = [0; 90; 0; 0; 0];          % in degrees
a = [0; 0; 0.130; 0.124; 0.126]; 
d = [0.077; 0; 0; 0; 0]; 

n_steps = 100;  % animation frames

% Joint limits
joint_limits = [
    deg2rad(-180), deg2rad(180);   % theta1
    deg2rad(-90),  deg2rad(90);    % theta2
    deg2rad(-75),  deg2rad(85);    % theta3
    deg2rad(-90),  deg2rad(90);    % theta4
    deg2rad(-180), deg2rad(180)    % theta5
];

% Starting position 
start_X = 0.0; 
start_Y = -0.20;
start_Z = 0.10;

% Target position
target_X = 0.0;
target_Y = 0.2; 
target_Z = 0.2;

% pitch angles
pitch_start = 0; 
pitch_end = 0; 


% First, calculate starting position
theta_prev = [0, 0, 0, 0, 0];  % Initialize previous angles
[theta1_start, theta2_start, theta3_start, theta4_start, theta5_start] = ...
    inverse_kinematics_continuous(start_X, start_Y, start_Z, pitch_start, theta_prev, joint_limits);

% Update previous angles
theta_prev = [theta1_start, theta2_start, theta3_start, theta4_start, theta5_start];

% Calculate target angles with desired pitch
[theta1_end, theta2_end, theta3_end, theta4_end, theta5_end] = ...
    inverse_kinematics_continuous(target_X, target_Y, target_Z, pitch_end, theta_prev, joint_limits);

% Interpolate joint angles for smooth motion
theta1_range = linspace(rad2deg(theta1_start), rad2deg(theta1_end), n_steps);
theta2_range = linspace(rad2deg(theta2_start), rad2deg(theta2_end), n_steps);
theta3_range = linspace(rad2deg(theta3_start), rad2deg(theta3_end), n_steps);
theta4_range = linspace(rad2deg(theta4_start), rad2deg(theta4_end), n_steps);


figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('Robot Arm Animation - (%.3f,%.3f,%.3f) to (%.3f,%.3f,%.3f)', ...
    start_X, start_Y, start_Z, target_X, target_Y, target_Z));
view(3);
xlim([-0.4 0.4]); ylim([-0.4 0.4]); zlim([0 0.5]);

% path storage
path_X = [];
path_Y = [];
path_Z = [];

% Animation loop
for i = 1:n_steps
    % Get interpolated joint angles
    theta1 = theta1_range(i);
    theta2 = theta2_range(i);
    theta3 = theta3_range(i);
    theta4 = theta4_range(i);
    
    offset_angle = 90 - 10.64;
    theta = [theta1; theta2 + offset_angle; theta3 - offset_angle; theta4; 0];

    % Forward kinematics
    T0_0 = eye(4);
    T0_1 = T0_0 * craig_dh_transform(alpha(1), a(1), d(1), theta(1));
    T1_2 = craig_dh_transform(alpha(2), a(2), d(2), theta(2));
    T0_2 = T0_1 * T1_2;
    T2_3 = craig_dh_transform(alpha(3), a(3), d(3), theta(3));
    T0_3 = T0_2 * T2_3;
    T3_4 = craig_dh_transform(alpha(4), a(4), d(4), theta(4));
    T0_4 = T0_3 * T3_4;
    T4_5 = craig_dh_transform(alpha(5), a(5), d(5), theta(5));
    T0_5 = T0_4 * T4_5;
    
    % Joint positions
    p0 = [0; 0; 0];  
    p1 = T0_1(1:3, 4);
    p2 = T0_2(1:3, 4);
    p3 = T0_3(1:3, 4);
    p4 = T0_4(1:3, 4);
    p5 = T0_5(1:3, 4);

    % Store end-effector position for path
    path_X = [path_X; p5(1)];
    path_Y = [path_Y; p5(2)];
    path_Z = [path_Z; p5(3)];

    cla;  % Clear current axes
    
    % Draw arm links
    line([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'Color', 'm', 'LineWidth', 2);
    line([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', 'm', 'LineWidth', 2);
    line([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'Color', 'm', 'LineWidth', 2);
    line([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)], 'Color', 'm', 'LineWidth', 2);
    line([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)], 'Color', 'm', 'LineWidth', 2);
    
    % Draw the path line
    if length(path_X) > 1
        plot3(path_X, path_Y, path_Z, 'y-', 'LineWidth', 2);
    end
    
    % Draw coordinate frames
    frame_size = 0.05; 
    plot_frame(T0_0, frame_size); 
    plot_frame(T0_1, frame_size);
    plot_frame(T0_2, frame_size);
    plot_frame(T0_3, frame_size);
    plot_frame(T0_4, frame_size);
    plot_frame(T0_5, frame_size);
    
    drawnow;
    pause(0.05); 
end
