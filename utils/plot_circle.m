function circle_points_world = plot_circle(lh, r, g_o, options)
    arguments
        lh
        r = 1;
        g_o = eye(4);
        options.t = linspace(0, 2*pi, 20)
    end
    
    circle_points_body = [cos(options.t); sin(options.t); zeros(size(options.t))] * r;
    circle_points_world = g_o(1:3, 1:3) * circle_points_body + g_o(1:3, 4);
    lh.XData = circle_points_world(1, :);
    lh.YData = circle_points_world(2, :);
    lh.ZData = circle_points_world(3, :);
end