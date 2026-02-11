function [Q, Qd, Qdd, t_vec] = multi_joint_trajectory(theta_start, theta_end, T, dt)
    num_joints = length(theta_start);
    t_vec = 0:dt:T;
    N = length(t_vec);
    
    Q = zeros(N, num_joints);
    Qd = zeros(N, num_joints);
    Qdd = zeros(N, num_joints);
    
    for j = 1:num_joints
        [q, qd, qdd, ~] = trajectory(...
            theta_start(j), ...  % start position
            theta_end(j), ...    % end position
            0, ...               % start velocity = 0
            0, ...               % end velocity = 0
            0, ...               % start acceleration = 0
            0, ...               % end acceleration = 0
            T, ...               % total time
            dt ...               % time step
        );
        
        % fill row by row
        Q(:, j) = q;
        Qd(:, j) = qd;
        Qdd(:, j) = qdd;
    end
end