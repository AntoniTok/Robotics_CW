function angle_wrapped = wrap_to_pi(angle)
    % Wrap angle to [-pi, pi]
    angle_wrapped = atan2(sin(angle), cos(angle));
end