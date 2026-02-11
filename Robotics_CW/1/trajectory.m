function [q, qd, qdd, t_vec] = trajectory(q0, qf, qd0, qdf, qdd0, qddf, T, dt)

    % Time vector
    t_vec = 0:dt:T;
    N = length(t_vec);
    
    % Solve for polynomial coefficients
    % q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    A = [1,  0,   0,    0,     0,      0;
         0,  1,   0,    0,     0,      0;
         0,  0,   2,    0,     0,      0;
         1,  T,   T^2,  T^3,   T^4,    T^5;
         0,  1,   2*T,  3*T^2, 4*T^3,  5*T^4;
         0,  0,   2,    6*T,   12*T^2, 20*T^3];
    
    b = [q0; qd0; qdd0; qf; qdf; qddf];
    
    coeff = A \ b;  % [a0; a1; a2; a3; a4; a5]
    
    % Calculate trajectory
    q = zeros(N, 1);
    qd = zeros(N, 1);
    qdd = zeros(N, 1);
    
    for i = 1:N
        t = t_vec(i);
        
        % Position: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        q(i) = coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5;
        
        % Velocity: qd(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
        qd(i) = coeff(2) + 2*coeff(3)*t + 3*coeff(4)*t^2 + 4*coeff(5)*t^3 + 5*coeff(6)*t^4;
        
        % Acceleration: qdd(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
        qdd(i) = 2*coeff(3) + 6*coeff(4)*t + 12*coeff(5)*t^2 + 20*coeff(6)*t^3;
    end
end