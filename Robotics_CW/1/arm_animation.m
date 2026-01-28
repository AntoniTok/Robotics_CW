
alpha = [0; 90; 0; 0; 0];          % in degrees
a = [0; 0; 0.130; 0.124; 0.126];   % link lengths
d = [0.077; 0; 0; 0; 0];           % offsets


n_steps = 100;  % animation frames

% motion ranges for each joint
theta1_range = linspace(0, 90, n_steps);    % Base rotates 0° to 90°
theta2_range = linspace(0, 45, n_steps);    % Joint 2 moves 0° to 45°
theta3_range = linspace(0, -60, n_steps);   % Joint 3 moves 0° to -60°
theta4_range = linspace(0, 30, n_steps);    % Joint 4 moves 0° to 30°


figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Arm Animation');
view(3);


for i = 1:n_steps

    theta1 = theta1_range(i);
    theta2 = theta2_range(i);
    theta3 = theta3_range(i);
    theta4 = theta4_range(i);
    
    offset_angle = 90 - 10.64;
    theta = [theta1; theta2 + offset_angle; theta3 - offset_angle; theta4; 0];

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
    
    
    p0 = [0; 0; 0];  
    p1 = T0_1(1:3, 4);
    p2 = T0_2(1:3, 4);
    p3 = T0_3(1:3, 4);
    p4 = T0_4(1:3, 4);
    p5 = T0_5(1:3, 4);

    cla;  % Clears current axes
    
    % base to frame 1
    line([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'Color', 'm', 'LineWidth', 2);
    % frame 1 to frame 2
    line([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', 'm', 'LineWidth', 2);
    % frame 2 to frame 3
    line([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'Color', 'm', 'LineWidth', 2);
    % frame 3 to frame 4
    line([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)], 'Color', 'm', 'LineWidth', 2);
    % frame 4 to frame 5
    line([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)], 'Color', 'm', 'LineWidth', 2);
    
    
    frame_size = 0.05; 
    
    plot_frame(T0_0, frame_size); 
    plot_frame(T0_1, frame_size);
    plot_frame(T0_2, frame_size);
    plot_frame(T0_3, frame_size);
    plot_frame(T0_4, frame_size);
    plot_frame(T0_5, frame_size);
        
    pause(0.05); 
end