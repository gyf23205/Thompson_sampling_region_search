function h = plot_targets(h, window)

% plot target locations
if ~isempty(h.targets)
    q = [h.targets.q];
    q = q(:, [h.targets.active]); % remove inactive targets
    h.viz.handle(window).targets = plot(q(1,:), q(2,:), 'd');
    set(h.viz.handle(window).targets, 'LineWidth', h.params.sim.linewidth, ...
        'Markersize', 7, 'Color', [1 .5 0], 'MarkerFaceColor', [1 .5 0]);
end
