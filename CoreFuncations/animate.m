function h = animate(h, t)

nr = length(h.robots);

% open figure
figure(h.viz.fig);

% Plot Server Belief %%%%%%%%%%%%%%%%%%%%%%
window = 1;
axes(h.viz.axes(window))

% update target locations
q = [h.targets.q];
q = q(:, [h.targets.active]); % remove inactive targets
set(h.viz.handle(window).targets, 'XData', q(1,:), 'YData', q(2,:));

% plot robot locations
for n = 1:nr
    r = h.robots(n);
    set(h.viz.handle(window).robots(n), 'XData', r.q(1), 'YData', r.q(2));

%     path = r.controller.control_points(r.data.path{t}, :);
%     set(h.viz.handle(window).paths(n), 'XData', path(:,1), 'YData', path(:,2));
%     if r.temp.explore_mode
%         set(h.viz.handle(window).paths(n), 'Color', 'r');
%     elseif r.temp.checkin_mode
%         set(h.viz.handle(window).paths(n), 'Color', 'b');
%     else
%         set(h.viz.handle(window).paths(n), 'Color', 'g');
%     end

    %set(h.viz.handle(window).voronoi(n), 'XData', r.voronoi(:,1), 'YData', r.voronoi(:,2));

    %set(h.viz.handle(window).paths(n), 'XData', r.temp.goal(1), 'YData', r.temp.goal(2));

    if isfield(h.viz.handle(window), 'footprints') && ~isempty(h.viz.handle(window).footprints(n))
        footprint = r.sensor.footprint;
        set(h.viz.handle(window).footprints(n), 'XData', r.q(1) + footprint(:,1),...
            'YData', r.q(2) + footprint(:,2));
    end
end


% Save to movie if desired %%%%%%%%%%%%%%%%%%%%%%
if h.params.sim.movie == 1
    currFrame = getframe(h.viz.fig);
    writeVideo(h.viz.aviobj, currFrame);
end
