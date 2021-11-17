function spacer_points_world = plot_spacer(lh, rho, g_spacer)
    spacer_points_body = [-rho rho; 0 0];
    spacer_points_world = g_spacer(1:2, 1:2) * spacer_points_body + g_spacer(1:2, 3);
    
    lh.XData = spacer_points_world(1, :);
    lh.YData = spacer_points_world(2, :);
    lh.ZData = zeros(size(spacer_points_body(1, :)));
end