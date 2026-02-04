% Robot parameters
alpha = [0; 90; 0; 0; 0];          % in degrees
a = [0; 0; 0.130; 0.124; 0.126];   
d = [0.077; 0; 0; 0; 0];           

n_steps_per_segment = 50;  % animation frames per line segment

% Joint limits
joint_limits = [
    deg2rad(-180), deg2rad(180);   % theta1
    deg2rad(-90),  deg2rad(90);    % theta2
    deg2rad(-75),  deg2rad(85);    % theta3
    deg2rad(-90),  deg2rad(90);    % theta4
    deg2rad(-180), deg2rad(180)    % theta5
];

% center position and square size
center_X = 0.18;
center_Y = 0.00;
center_Z = 0.20;
square_size = 0.05;  % 10cm square = 5cm from center to edge

% Define pitch angles for each square
pitch_XY = 0;           % Horizontal
pitch_XZ = 0;           % Horizontal 
pitch_YZ = 0;           % Horizontal 


% waypoints for three squares
% Square 1: XY plane (Z constant) - RED
square_XY = [
    center_X - square_size, center_Y - square_size, center_Z;  % bottom-left
    center_X + square_size, center_Y - square_size, center_Z;  % bottom-right
    center_X + square_size, center_Y + square_size, center_Z;  % top-right
    center_X - square_size, center_Y + square_size, center_Z;  % top-left
    center_X - square_size, center_Y - square_size, center_Z;  % back to start
];

% Square 2: XZ plane (Y constant) - GREEN
square_XZ = [
    center_X - square_size, center_Y, center_Z - square_size;  % bottom-left
    center_X + square_size, center_Y, center_Z - square_size;  % bottom-right
    center_X + square_size, center_Y, center_Z + square_size;  % top-right
    center_X - square_size, center_Y, center_Z + square_size;  % top-left
    center_X - square_size, center_Y, center_Z - square_size;  % back to start
];

% Square 3: YZ plane (X constant) - BLUE
square_YZ = [
    center_X, center_Y - square_size, center_Z - square_size;  % bottom-left
    center_X, center_Y + square_size, center_Z - square_size;  % bottom-right
    center_X, center_Y + square_size, center_Z + square_size;  % top-right
    center_X, center_Y - square_size, center_Z + square_size;  % top-left
    center_X, center_Y - square_size, center_Z - square_size;  % back to start
];

% Combine all waypoints
all_waypoints = [square_XY; square_XZ; square_YZ];
num_waypoints = size(all_waypoints, 1);

% Define pitch angles for each waypoint - ALL ZERO
pitch_angles = [
    repmat(pitch_XY, 5, 1);   % XY plane
    repmat(pitch_XZ, 5, 1);   % XZ plane
    repmat(pitch_YZ, 5, 1);   % YZ plane
];

% Define colors for each square (5 segments per square)
colors = [
    repmat([1, 0, 0], 5, 1);  % RED for XY plane
    repmat([0, 1, 0], 5, 1);  % GREEN for XZ plane
    repmat([0, 0, 1], 5, 1);  % BLUE for YZ plane
];

fprintf('Robot reach limits: min=%.3fm, max=%.3fm\n\n', abs(0.130-0.124), 0.130+0.124);

joint_angles = zeros(num_waypoints, 5);
theta_prev = [0, 0, 0, 0, 0];

for i = 1:num_waypoints
    fprintf('Waypoint %d/%d: (%.3f, %.3f, %.3f) pitch=%.1f°\n', ...
            i, num_waypoints, all_waypoints(i,1), all_waypoints(i,2), ...
            all_waypoints(i,3), rad2deg(pitch_angles(i)));
    
    try
        [theta1, theta2, theta3, theta4, theta5] = ...
            inverse_kinematics_continuous(all_waypoints(i,1), all_waypoints(i,2), ...
                                         all_waypoints(i,3), pitch_angles(i), ...
                                         theta_prev, joint_limits);
        joint_angles(i, :) = [theta1, theta2, theta3, theta4, theta5];
        theta_prev = joint_angles(i, :);
        fprintf('  ✓ Success\n');
    catch ME
        fprintf('\n*** ERROR at waypoint %d ***\n', i);
        fprintf('Position: (%.3f, %.3f, %.3f)\n', all_waypoints(i,1), all_waypoints(i,2), all_waypoints(i,3));
        fprintf('Pitch: %.1f°\n', rad2deg(pitch_angles(i)));
        fprintf('Error: %s\n', ME.message);
        rethrow(ME);
    end
end

fprintf('\nAll waypoints calculated successfully!\n\n');


figure;
hold on;
grid on;
axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Robot Tracing 10cm Squares: XY-Red, XZ-Green, YZ-Blue (all pitch=0°)');
view(3);
xlim([-0.3 0.3]); ylim([-0.3 0.3]); zlim([0 0.4]);


path_XY = []; path_XZ = []; path_YZ = [];
current_path_X = []; current_path_Y = []; current_path_Z = [];

% Animation loop
segment_idx = 0;
for waypoint_idx = 1:(num_waypoints-1)
    segment_idx = segment_idx + 1;
    
    % Determine which square we're drawing
    if segment_idx <= 5
        square_name = 'XY (RED)';
    elseif segment_idx <= 10
        square_name = 'XZ (GREEN)';
    else
        square_name = 'YZ (BLUE)';
    end
    
    % Interpolate between current and next waypoint
    theta1_range = linspace(rad2deg(joint_angles(waypoint_idx, 1)), ...
                           rad2deg(joint_angles(waypoint_idx+1, 1)), n_steps_per_segment);
    theta2_range = linspace(rad2deg(joint_angles(waypoint_idx, 2)), ...
                           rad2deg(joint_angles(waypoint_idx+1, 2)), n_steps_per_segment);
    theta3_range = linspace(rad2deg(joint_angles(waypoint_idx, 3)), ...
                           rad2deg(joint_angles(waypoint_idx+1, 3)), n_steps_per_segment);
    theta4_range = linspace(rad2deg(joint_angles(waypoint_idx, 4)), ...
                           rad2deg(joint_angles(waypoint_idx+1, 4)), n_steps_per_segment);
    
    % Reset current path for new segment
    current_path_X = [];
    current_path_Y = [];
    current_path_Z = [];
    
    for i = 1:n_steps_per_segment
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

        % Store end-effector position for current segment
        current_path_X = [current_path_X; p5(1)];
        current_path_Y = [current_path_Y; p5(2)];
        current_path_Z = [current_path_Z; p5(3)];
        
        % Store in appropriate square history
        if segment_idx <= 5
            path_XY = [path_XY; p5(1), p5(2), p5(3)];
        elseif segment_idx <= 10
            path_XZ = [path_XZ; p5(1), p5(2), p5(3)];
        else
            path_YZ = [path_YZ; p5(1), p5(2), p5(3)];
        end

        cla;  % Clear current axes
        
        % Draw arm links
        line([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'Color', 'm', 'LineWidth', 2);
        line([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', 'm', 'LineWidth', 2);
        line([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'Color', 'm', 'LineWidth', 2);
        line([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)], 'Color', 'm', 'LineWidth', 2);
        line([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)], 'Color', 'm', 'LineWidth', 2);
        
        % Draw completed squares
        if size(path_XY, 1) > 1
            plot3(path_XY(:,1), path_XY(:,2), path_XY(:,3), 'r-', 'LineWidth', 3);
        end
        if size(path_XZ, 1) > 1
            plot3(path_XZ(:,1), path_XZ(:,2), path_XZ(:,3), 'g-', 'LineWidth', 3);
        end
        if size(path_YZ, 1) > 1
            plot3(path_YZ(:,1), path_YZ(:,2), path_YZ(:,3), 'b-', 'LineWidth', 3);
        end
        
        % Draw current segment being traced
        if length(current_path_X) > 1
            plot3(current_path_X, current_path_Y, current_path_Z, ...
                  'Color', colors(segment_idx, :), 'LineWidth', 3);
        end
        
        % Draw coordinate frames for all joints
        frame_size = 0.03; 
        plot_frame(T0_0, frame_size); 
        plot_frame(T0_1, frame_size);
        plot_frame(T0_2, frame_size);
        plot_frame(T0_3, frame_size);
        plot_frame(T0_4, frame_size);
        plot_frame(T0_5, frame_size);
        
        % Add text showing current square
        text(0.22, 0.22, 0.35, sprintf('Drawing: %s', square_name), ...
             'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'white');
        
        drawnow;
        pause(0.03); 
    end
    
    fprintf('Completed segment %d/%d (%s)\n', segment_idx, num_waypoints-1, square_name);
end

fprintf('\nAll squares completed!\n');

% Add legend
legend('', '', '', '', '', 'XY Plane', 'XZ Plane', 'YZ Plane', 'Location', 'best');
