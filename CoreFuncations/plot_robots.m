function h = plot_robots(h, q, window, footprint_on, path_on)

% plot robot locations and paths
for n = 1:size(q, 2)
    h.viz.handle(window).robots(n) = plot(q(1,n), q(2,n), 'gs');
    set(h.viz.handle(window).robots(n), 'LineWidth', 2, ...
        'MarkerSize', 6, ...
        'MarkerFaceColor', 'g');
    
    if footprint_on
        h.viz.handle(window).footprints(n) = plot(q(1,n)+h.robots(n).sensor.footprint(:,1),...
            q(2,n)+h.robots(n).sensor.footprint(:,2), 'g', 'LineWidth', 2);
    end
    %h.viz.handle(window).voronoi(n) = plot(h.robots(n).voronoi(:,1), h.robots(n).voronoi(:,2), 'k:');
    if path_on
        h.viz.handle(window).paths(n) = plot(NaN, NaN, ...
            'rx', 'LineWidth', 2);
    end
end
