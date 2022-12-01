function h = init_animation(h)

% open figure
base_size = 250;
width = 1.8*base_size;
height = 2.1*base_size;
x0 = 250;
y0 = 50;
h.viz.fig = figure('Position', [x0, y0, x0+width, y0+height]);

p_top = h.params.sim.p_top;
p_right = h.params.sim.p_right;


% Plot Server Belief %%%%%%%%%%%%%%%%%%%%%%
window = 1;
h.viz.axes(window) = subaxis(10, 10, [1 66], 'PaddingRight', p_right, 'PaddingTop', p_top); %Declare subplot

hold on;

plot_env(h)
h = plot_targets(h, window);

% Make grid
% min_xy = min(h.params.env.bndry);
% max_xy = max(h.params.env.bndry);
% sz = ceil((max_xy - min_xy) / h.params.sim.phd_grid_size);
% subs = ceil(bsxfun(@minus, h.server.state.x', min_xy) / h.params.sim.phd_grid_size);
% phd_grid = accumarray(subs, h.server.state.w, sz).';
% 
% h.viz.handle(window).grid = imagesc(phd_grid);
% set(h.viz.handle(window).grid, 'XData', [min_xy(1) max_xy(1)],...
%     'YData', [min_xy(2) max_xy(2)], 'AlphaData', 0.5)
% colormap(1-gray)
% set(h.viz.axes(window), 'CLim', [0 1])

% Plot robots
h = plot_robots(h, [h.robots.q], window, true, true);

% make legend
l1 = legend([h.viz.handle(window).robots(1), h.viz.handle(window).targets,...
  ],...
    'Sensor with footprint', 'Target','location', [0.2 0.15 0.2 0.1]);

hold off;
axis image;
box on;

% set axis properties
back_color = get(h.viz.fig, 'Color');
set_font_options();
set(gca, 'LineWidth', h.params.sim.linewidth, 'Color', back_color);
% title('Global View')

min_xy = min(h.params.env.bndry);
max_xy = max(h.params.env.bndry);
sz = max_xy - min_xy;
xlims = [min_xy(1) - 0.1*sz(1), max_xy(1) + 0.1*sz(1)];
ylims = [min_xy(2) - 0.1*sz(2), max_xy(2) + 0.1*sz(2)];
xlim(xlims);
ylim(ylims);

 
% 
% % preserve background color in axes
% set(h.viz.fig, 'Color', 'w', 'InvertHardCopy', 'off');

% Save to movie if desired %%%%%%%%%%%%%%%%%%%%%%
if h.params.sim.movie == 1
    h.viz.aviobj = VideoWriter( ...
        sprintf('%s/Video%s.avi', h.params.sim.folder, datestr(h.params.sim.date,'yymmdd_HHMMSS')), ...
        'Motion JPEG AVI');
    h.viz.aviobj.FrameRate = h.params.sim.frame_rate;
    
    open(h.viz.aviobj);
end

end
