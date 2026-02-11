function T_segment = segment_time(theta_start, theta_end, max_v, max_a)
    % minimum time needed to move between two poses
    
    d_theta = abs(theta_end - theta_start);
    
    % time based on velocity limit: T = 1.5 * delta_q / v_max
    % (factor 1.5 accounts for acceleration/deceleration phases)
    T_vel = 1.5 * d_theta ./ max_v;
    
    % time based on acceleration limit: T = sqrt(6 * delta_q / a_max)
    T_accel = sqrt(6 * d_theta ./ max_a);
    
    % take the max across all joints
    T_segment = max([T_vel; T_accel]);
    
    % safety margin
    T_segment = T_segment * 1.2;
end