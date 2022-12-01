function h = main(options)
% This is the master function for the simulation
options.n_robots = 1; 
options.env_size = [100 100]; 
options.seed = 0; 
options.v_max = 0; 
options.animate = true;
options.tMax = 400; 
options.mu_b = 0; 
options.static = true;
close all;

% Initialize constants and data structures
h.params = environ_constants(options);
if h.params.sim.movie
    h.params.sim.animate = 1;
end

% Initialize robots
bbox = h.params.env.bbox; % bounding box of env
width = bbox(3) - bbox(1);
height = bbox(4) - bbox(2);
factor = 0.1; % factor to grow the bounding box to place targets outside of env

q0 = bsxfun(@plus, bbox(1:2)' + [0.4*width; 0], bsxfun(@times, [0.2*width; 0.1*height], rand(2, h.params.env.nBots)));
h.server = init_server(h); % for centralized filter for comparison
for i = 1:h.params.env.nBots
    h.robots(i) = init_robot(h, q0(:,i),i);
end

% Initialize targets
t0 = bsxfun(@plus, bbox(1:2)', ...
    bsxfun(@times, [width; height]/3, rand(2, 40)));
t0 = [t0; 2*pi*rand(1, 40)]; % add orientation

for i = 1:40
    h.targets(i) = init_target(h, t0(:,i), i);
end

t0 = bsxfun(@plus, bbox(1:2)' + 2*[width; height]/3, ...
    bsxfun(@times, [width; height]/3, rand(2, 10)));
t0 = [t0; 2*pi*rand(1, 10)]; % add orientation

for i = 31:40
    h.targets(i) = init_target(h, t0(:,i-30), i);
end

% Initialize animation window, if desired
if h.params.sim.animate > 0
    h = init_animation(h);
end

% Run the simulation
try
    parpool;
catch
    
end

h = run_simulation(h);

% Save data
if h.params.sim.movie == 1
    close(h.viz.aviobj);
end
if ~exist(h.params.sim.folder, 'dir')
    mkdir(h.params.sim.folder)
end

% Remove extraneous data
if h.params.sim.animate == 1
    h = rmfield(h, 'viz');
end

% h = rmfield(h, 'temp');

% scenario = h.params.env.scenario;
% h.params = rmfield(h.params, 'env');
% h.params.env.scenario = scenario;

names = fieldnames(h.robots);
names = names(~strcmp(names, 'data'));
h.robots = rmfield(h.robots, names);

names = fieldnames(h.server);
names = names(~strcmp(names, 'data'));
h.server = rmfield(h.server, names);
disp(h.params.env.nBots);
save(sprintf('./%s/Results_%s.mat', h.params.sim.folder, num2str(h.params.env.nBots)),'h')
