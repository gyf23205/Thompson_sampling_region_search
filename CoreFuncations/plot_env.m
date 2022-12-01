function plot_env(h)
% plot environment boundary
patch(h.params.env.bndry(:,1), h.params.env.bndry(:,2), 'w', ...
    'LineWidth', h.params.sim.linewidth)

% set background color to grey
back_color = get(h.viz.fig, 'Color');
set(gca, 'Color', back_color)

% plot obstacles
for i = 1:length(h.params.env.obst)
    patch(h.params.env.obst{i}(:,1), h.params.env.obst{i}(:,2), 'w', ...
        'FaceColor', back_color, 'LineWidth', h.params.sim.linewidth)
end

% bounding box and ticks for global view
xlims = [min(h.params.env.bndry(:,1)) max(h.params.env.bndry(:,1))] + 5*[-1 1];
ylims = [min(h.params.env.bndry(:,2)) max(h.params.env.bndry(:,2))] + 5*[-1 1];
axis([xlims, ylims])
