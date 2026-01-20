function T = craig_dh_transform(alpha, a, d, theta)
    % Craig DH transformation matrix
    % Inputs: alpha_{i-1}, a_{i-1}, d_i, theta_i (angles in degrees)
    
    %  angles to radians
    alpha_rad = deg2rad(alpha);
    theta_rad = deg2rad(theta);
    
    
    T = [cos(theta_rad), -sin(theta_rad), 0, a;
         sin(theta_rad)*cos(alpha_rad), cos(theta_rad)*cos(alpha_rad), -sin(alpha_rad), -sin(alpha_rad)*d;
         sin(theta_rad)*sin(alpha_rad), cos(theta_rad)*sin(alpha_rad),  cos(alpha_rad),  cos(alpha_rad)*d;
         0, 0, 0, 1];
end