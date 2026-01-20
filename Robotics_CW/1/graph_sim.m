
% Define your DH parameters (as matrices for easy access)
% Each row is [alpha, a, d, theta]
% We'll define the constant parts, theta will be variable

alpha = [0; 90; 0; 0; 0];          % in degrees
a = [0; 0; 0.130; 0.124; 0.126];   % link lengths
d = [0.077; 0; 0; 0; 0];           % offsets

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;

% Apply offsets
theta = [theta1; theta2 + (90 - 10.64); theta3 - (90 - 10.64); theta4; 0];  % theta5 is always 0


% identity matrix
T0_0 = eye(4);

% Craig DH transformation matrix inputs: alpha_{i-1}, a_{i-1}, d_i, theta_i (angles in degrees)

% Frame 0 to Frame 1
T0_1 = T0_0 * craig_dh_transform(alpha(1), a(1), d(1), theta(1));

% Frame 2
T1_2 = craig_dh_transform(alpha(2), a(2), d(2), theta(2));
T0_2 = T0_1 * T1_2;

% Frame 3
T2_3 = craig_dh_transform(alpha(3), a(3), d(3), theta(3));
T0_3 = T0_2 * T2_3;

% Frame 4
T3_4 = craig_dh_transform(alpha(4), a(4), d(4), theta(4));
T0_4 = T0_3 * T3_4;

% Frame 5 
T4_5 = craig_dh_transform(alpha(5), a(5), d(5), theta(5));
T0_5 = T0_4 * T4_5;



% Extract positions of each frame origin
p0 = [0; 0; 0];  % Base frame at origin
p1 = T0_1(1:3, 4);  % Extract x,y,z from last column
p2 = T0_2(1:3, 4);
p3 = T0_3(1:3, 4);
p4 = T0_4(1:3, 4);
p5 = T0_5(1:3, 4);



% Create a 3D figure
figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Arm Visualization');
view(3);  % 3D view

% Plot the links connecting each frame
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






