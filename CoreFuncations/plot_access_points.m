function plot_access_points(h, footprint_on)

% plot access points and communication footprints
for i = 1:size(h.params.env.access_points,2)
    plot(h.params.env.access_points(1,i), h.params.env.access_points(2,i), 'b^', ...
        'MarkerSize', 11, 'Linewidth', 2, 'MarkerFaceColor', 'b');
    
    if footprint_on == true
        theta = linspace(0, 2*pi);
        x = h.params.env.access_points(1,i) + h.params.comm.Rc_server*cos(theta);
        y = h.params.env.access_points(2,i) + h.params.comm.Rc_server*sin(theta);
        plot(x, y, 'b--', 'LineWidth', 2)
    end
end
