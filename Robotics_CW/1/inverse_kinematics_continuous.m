function [theta1, theta2, theta3, theta4, theta5] = inverse_kinematics_continuous(X, Y, Z, pitch_desired, theta_prev, joint_limits)
    
    % joint_limits = [theta1_min, theta1_max;  % Row 1: theta1 limits
    %                 theta2_min, theta2_max;  % Row 2: theta2 limits
    %                 ... ]
    
    %Parameters 
    d1 = 0.077;
    L3 = 0.130;
    L4 = 0.124;
    L5 = 0.126;
    offset = deg2rad(90 - 10.64);
    theta5 = 0;
    
    if nargin < 4, pitch_desired = -pi/2; end
    if nargin < 5, theta_prev = [0, 0, 0, 0, 0]; end
    if nargin < 6, joint_limits = []; end  % No limits by default
    
    
    %% SUBSYSTEM 1: Base Rotation

    theta1_raw = atan2(Y, X);
    
    % equivalent angles: theta-360°, theta, theta+360°
    theta1_candidates = [theta1_raw - 2*pi, theta1_raw, theta1_raw + 2*pi];
    
    % filter by joint limits
    if ~isempty(joint_limits)
        valid_mask = (theta1_candidates >= joint_limits(1,1)) & ...
                     (theta1_candidates <= joint_limits(1,2));
        theta1_candidates = theta1_candidates(valid_mask);
        
        if isempty(theta1_candidates)
            error('No valid theta1 within joint limits [%.1f°, %.1f°]', ...
                  rad2deg(joint_limits(1,1)), rad2deg(joint_limits(1,2)));
        end
    end
    
    % Choose the candidate closest to previous angle
    [~, idx] = min(abs(theta1_candidates - theta_prev(1)));
    theta1 = theta1_candidates(idx);
    
    
    %% SUBSYSTEM 2: Arm

    % Wrist position
    R = sqrt(X^2 + Y^2);
    Z_adj = Z - d1;
    R_wrist = R - L5 * cos(pitch_desired);
    Z_wrist = Z_adj - L5 * sin(pitch_desired);
    D = sqrt(R_wrist^2 + Z_wrist^2);
    
    % Reachability check
    max_reach = L3 + L4;
    min_reach = abs(L3 - L4);
    
    if D > max_reach || D < min_reach
        error('Target unreachable: D=%.3f m (valid: %.3f to %.3f m)', ...
              D, min_reach, max_reach);
    end

    cos_theta3 = (D^2 - L3^2 - L4^2) / (2 * L3 * L4);
    
    if abs(cos_theta3) > 1
        error('No valid elbow angle: cos(theta3) = %.4f', cos_theta3);
    end
    
    % Elbow-down solution
    sin_theta3_down = -sqrt(1 - cos_theta3^2);
    theta3_down = atan2(sin_theta3_down, cos_theta3);
    k1_down = L3 + L4 * cos(theta3_down);
    k2_down = L4 * sin(theta3_down);
    theta2_down = atan2(Z_wrist, R_wrist) - atan2(k2_down, k1_down);
    theta4_down = pitch_desired - theta2_down - theta3_down;
    theta2_down = theta2_down - offset;
    theta3_down = theta3_down + offset;
    
    % Elbow-up solution
    sin_theta3_up = +sqrt(1 - cos_theta3^2);
    theta3_up = atan2(sin_theta3_up, cos_theta3);
    k1_up = L3 + L4 * cos(theta3_up);
    k2_up = L4 * sin(theta3_up);
    theta2_up = atan2(Z_wrist, R_wrist) - atan2(k2_up, k1_up);
    theta4_up = pitch_desired - theta2_up - theta3_up;
    theta2_up = theta2_up - offset;
    theta3_up = theta3_up + offset;
    
    % Check joint limits for both solutions
    solutions = [theta2_down, theta3_down, theta4_down; 
                 theta2_up, theta3_up, theta4_up];
    valid_solutions = [true; true];
    
    if ~isempty(joint_limits) && size(joint_limits, 1) >= 4
        for i = 1:2
            for j = 2:4
                if solutions(i, j-1) < joint_limits(j,1) || ...
                   solutions(i, j-1) > joint_limits(j,2)
                    valid_solutions(i) = false;
                    break;
                end
            end
        end
    end
    
    if ~any(valid_solutions)
        error('No valid elbow configuration within joint limits');
    end
    
    % Choose solution closest to previous configuration
    best_dist = inf;
    best_sol = 1;
    
    for i = find(valid_solutions)'
        dist = abs(wrap_to_pi(solutions(i,1) - theta_prev(2))) + ...
               abs(wrap_to_pi(solutions(i,2) - theta_prev(3))) + ...
               abs(wrap_to_pi(solutions(i,3) - theta_prev(4)));
        
        if dist < best_dist
            best_dist = dist;
            best_sol = i;
        end
    end
    
    theta2 = solutions(best_sol, 1);
    theta3 = solutions(best_sol, 2);
    theta4 = solutions(best_sol, 3);
    
    
    fprintf('\n=== IK Results ===\n');
    fprintf('θ₁=%7.2f° θ₂=%7.2f° θ₃=%7.2f° θ₄=%7.2f° θ₅=%7.2f°\n', ...
            rad2deg([theta1, theta2, theta3, theta4, theta5]));
    fprintf('Config: %s\n', iif(best_sol==1, 'elbow-down', 'elbow-up'));
    
end

function out = iif(cond, true_val, false_val)
    if cond, out = true_val; else, out = false_val; end
end
