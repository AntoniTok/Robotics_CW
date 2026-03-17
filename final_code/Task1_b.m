alpha = [0; 90; 0; 0; 0];   % deg
a     = [0; 0; 0.130; 0.124; 0.126];
d     = [0.077; 0; 0; 0; 0];

offset_angle = 90 - 10.64;

joint_limits = [
    deg2rad(-180), deg2rad(180);  % theta1
    deg2rad(-90),  deg2rad(90);   % theta2
    deg2rad(-75),  deg2rad(85);   % theta3
    deg2rad(-90),  deg2rad(90);   % theta4
    deg2rad(-180), deg2rad(180)   % theta5
];

joint_names = {'Joint 1 (Base Rotation)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)', 'Joint 5 (End Roll)'};

N_steps    = 200;
dt_pause   = 0.01;
home_theta = zeros(1, 5);


figure('Name', 'FK Joint Sweep');
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 25);
xlim([-0.45 0.45]); ylim([-0.45 0.45]); zlim([0 0.55]);

frame_size = 0.04;


for active_joint = 1:5

    lo = joint_limits(active_joint, 1);
    hi = joint_limits(active_joint, 2);
    sweep_angles = linspace(lo, hi, N_steps);

    path_X = []; path_Y = []; path_Z = [];

    for k = 1:N_steps

        % joint angle vector
        theta_rad = home_theta;
        theta_rad(active_joint) = sweep_angles(k);

        % mechanical offsets
        theta_deg = rad2deg(theta_rad);
        theta = [theta_deg(1);
                 theta_deg(2) + offset_angle;
                 theta_deg(3) - offset_angle;
                 theta_deg(4);
                 theta_deg(5)];

        T0_0 = eye(4);
        T0_1 = T0_0 * craig_dh_transform(alpha(1), a(1), d(1), theta(1));
        T0_2 = T0_1 * craig_dh_transform(alpha(2), a(2), d(2), theta(2));
        T0_3 = T0_2 * craig_dh_transform(alpha(3), a(3), d(3), theta(3));
        T0_4 = T0_3 * craig_dh_transform(alpha(4), a(4), d(4), theta(4));
        T0_5 = T0_4 * craig_dh_transform(alpha(5), a(5), d(5), theta(5));

        p0 = [0; 0; 0];
        p1 = T0_1(1:3, 4);
        p2 = T0_2(1:3, 4);
        p3 = T0_3(1:3, 4);
        p4 = T0_4(1:3, 4);
        p5 = T0_5(1:3, 4);

        path_X(end+1) = p5(1);
        path_Y(end+1) = p5(2);
        path_Z(end+1) = p5(3);

        cla;
        title(sprintf('Sweeping %s', joint_names{active_joint}));

        link_pts = [p0, p1, p2, p3, p4, p5];
        for seg = 1:5
            line(link_pts(1, seg:seg+1), ...
                 link_pts(2, seg:seg+1), ...
                 link_pts(3, seg:seg+1), ...
                 'Color', 'm', 'LineWidth', 2);
        end

        if length(path_X) > 1
            plot3(path_X, path_Y, path_Z, 'y-', 'LineWidth', 2);
        end

        plot_frame(T0_0, frame_size);
        plot_frame(T0_1, frame_size);
        plot_frame(T0_2, frame_size);
        plot_frame(T0_3, frame_size);
        plot_frame(T0_4, frame_size);
        plot_frame(T0_5, frame_size);

        drawnow;
        pause(dt_pause);
    end

    fprintf('Finished');
end

