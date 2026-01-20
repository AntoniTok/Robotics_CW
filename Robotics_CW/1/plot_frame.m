function plot_frame(T, frame_size)
    % Extract position (origin of frame)
    origin = T(1:3, 4);
    
    % Extract orientation (rotation matrix - first 3x3 of T)
    R = T(1:3, 1:3);
    
    % X-axis (red) - first column of R
    x_axis = R(1:3, 1);
    % Y-axis (green) - second column of R
    y_axis = R(1:3, 2);
    % Z-axis (blue) - third column of R
    z_axis = R(1:3, 3);
    
    % Draw the axes (scaled by frame_size)
    % X-axis is red
    line([origin(1), origin(1) + frame_size*x_axis(1)], ...
         [origin(2), origin(2) + frame_size*x_axis(2)], ...
         [origin(3), origin(3) + frame_size*x_axis(3)], ...
         'Color', 'r', 'LineWidth', 1.5);
    
    % Y-axis is green
    line([origin(1), origin(1) + frame_size*y_axis(1)], ...
         [origin(2), origin(2) + frame_size*y_axis(2)], ...
         [origin(3), origin(3) + frame_size*y_axis(3)], ...
         'Color', 'g', 'LineWidth', 1.5);

    % Z-axis is blue
    line([origin(1), origin(1) + frame_size*z_axis(1)], ...
         [origin(2), origin(2) + frame_size*z_axis(2)], ...
         [origin(3), origin(3) + frame_size*z_axis(3)], ...
         'Color', 'b', 'LineWidth', 1.5);
end