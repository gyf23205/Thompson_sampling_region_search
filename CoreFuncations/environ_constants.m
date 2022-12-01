function p = environ_constants(options)
% Holds user options and initializes data structures

    if ~isfield(options, 'env_size')
        options.env_size = 50;
    end
    if ~isfield(options, 'n_targets')
        options.n_targets = 50;
    end
    if ~isfield(options, 'n_robots')
        options.n_robots = 20;
    end
    if ~isfield(options, 'seed')
        options.seed = prod(clock);
    end
    if ~isfield(options, 'v_max')
        options.v_max = 1;
    end
    if ~isfield(options, 'animate')
        options.animate = true;
    end
    if ~isfield(options, 'tMax')
        options.tMax = 1000;
    end
    if ~isfield(options, 'static')
        options.static = false;
    end
    if ~isfield(options, 'folder')
        options.folder = 'results';
    end
    if ~isfield(options, 'decentralized')
        options.decentralized = true;
    end

    s = RandStream('mt19937ar', 'Seed', int32(options.seed));
    RandStream.setGlobalStream(s);
    p.sim.seed = s; % save random seed


% SIMULATION OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.sim.animate = options.animate;
    p.sim.movie = 0;
    p.sim.frame_rate = 15;

    p.sim.entropy_thresh = 0.1; % once threshold is reached, stop the simulation
    p.sim.ospa_cutoff = 10; % distance cutoff for OSPA metric

    % folder to save results in
    p.sim.folder = options.folder;

    p.sim.tMax = options.tMax;
    p.sim.decentralized = options.decentralized;

    
% SENSOR OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta = linspace(0, 2*pi, 20);
    p.sensor.Rs = 5; % select finite sensor footprint
    footprint = p.sensor.Rs*[cos(theta') sin(theta')];
    [x, y] = poly2cw(footprint(:,1), footprint(:,2));
    p.sensor.footprint = [x y];
    

    p.sensor.CovD = 0.01; % measurement covariance
    p.sensor.Sigma = Inf; %(0.4*h.sensor.Rs)^2; %(r.sensor.Rs/2)^2; %4; % detection covariance
    p.sensor.pk0 = 1; %exp(-h.phd.mu_c*r.sensor.area/h.env.area); % probability of no false positives
    p.sensor.Pfn = 0;%0.1; % probability of a false negative

    p.sensor.rate = 2; % Hz of sensor updates
    

% ENVIRONMENT OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if numel(options.env_size) == 1
        options.env_size = options.env_size * [1 1];
    end
    p.env.bndry = bsxfun(@times, options.env_size, [0 0; 1 0; 1 1; 0 1]);
    p.env.obst = {};
    p.env.polygons = cell(1);
    p.env.polygons{1} = p.env.bndry;
    p.env.bbox = bounding_box(p.env.bndry);

    % convert polygons to cw
    [x, y] = poly2cw(p.env.bndry(:,1), p.env.bndry(:,2));
    p.env.bndry = [x y];
    for i = 1:length(p.env.obst)
        [x, y] = poly2cw(p.env.obst{i}(:,1), p.env.obst{i}(:,2));
        p.env.obst{i} = [x y];
    end

    p.env.area = polyarea(p.env.bndry(:,1), p.env.bndry(:,2)); % area of env
    for i = 1:length(p.env.obst)
        p.env.area = p.env.area - polyarea(p.env.obst{i}(:,1), p.env.obst{i}(:,2));
    end

    p.env.nAccess = 1;
    p.env.access_points = [0; 0];
    
    p.env.num_xarms = options.env_size(1)/(options.env_size(1)/ (1.4*p.sensor.Rs));
    p.env.num_yarms = options.env_size(1)/(options.env_size(2)/ (1.4*p.sensor.Rs));
    x = (p.sensor.Rs/2):p.env.num_xarms:options.env_size(1);
    y = (p.sensor.Rs/2):p.env.num_yarms:options.env_size(2);
    [X, Y] = meshgrid(x, y);
    p.env.ts_environment = [X(:) Y(:)];

    p.sensor.area = min(polyarea(p.sensor.footprint(:,1), p.sensor.footprint(:,2)), p.env.area);
    
    
% TARGET OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.env.nTargets = options.n_targets;

    if options.static
        p.target.v_max = 0; % max linear vel for targets
        p.target.w_max = 0; % max angular vel for targets
    else
        p.target.v_max = options.v_max; % max linear vel for targets
        p.target.w_max = 1; % max angular vel for targets
    end
    p.env.rate = 10; % Hz for target motion updates


% ROBOT OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.env.nBots = options.n_robots;


% SENSOR OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta = linspace(0, 2*pi, 20);
    p.sensor.Rs = 5; % select finite sensor footprint
    footprint = p.sensor.Rs*[cos(theta') sin(theta')];
    [x, y] = poly2cw(footprint(:,1), footprint(:,2));
    p.sensor.footprint = [x y];
    p.sensor.area = min(polyarea(p.sensor.footprint(:,1), p.sensor.footprint(:,2)), p.env.area);

    p.sensor.CovD = 0.25; % measurement covariance
    p.sensor.Sigma = Inf; %(0.4*h.sensor.Rs)^2; %(r.sensor.Rs/2)^2; %4; % detection covariance
    p.sensor.pk0 = 1; %exp(-h.phd.mu_c*r.sensor.area/h.env.area); % probability of no false positives
    p.sensor.Pfn = 0;%0.1; % probability of a false negative

    p.sensor.rate = 2; % Hz of sensor updates


% PHD FILTER OPTIONS %%%%s%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.phd.grid_size = 1; % spacing between particles
    p.env.size = ceil([(p.env.bbox(4)-p.env.bbox(2))/p.phd.grid_size,...
        (p.env.bbox(3)-p.env.bbox(1))/p.phd.grid_size]);
    
    p.phd.lambda = 1; % expected initial number of targets
    if options.static
        p.phd.ps0 = 1; % probability of target survival at boundary
        p.phd.ps1 = 1; % probability of target survival at infinity
        p.phd.ps_d = 0; % dropoff distance for ps
    else
        p.phd.ps0 = 1; % probability of target survival at boundary
        p.phd.ps1 = 1; % probability of target survival at infinity
        p.phd.ps_d = 2; % dropoff distance for ps
    end

    p.phd.mu_c = -log(p.sensor.pk0);
    p.phd.kappa = p.phd.mu_c / p.sensor.area;

    if options.static
        p.phd.mu_b = 0; % expected number of newborn targets
        p.phd.birth_d = 5; % only add weight within this dist of edges
    else
        p.phd.mu_b = 0.4; % expected number of newborn targets
        p.phd.birth_d = 5; % only add weight within this dist of edges
    end
    
    area = p.env.area - max(options.env_size(1) - 2*p.phd.birth_d, 0) * max(options.env_size(2) - 2*p.phd.birth_d, 0);
    p.phd.gamma = p.phd.mu_b / area; % density of births


% CONTROLLER OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.controller.vel = 10; % maximum robot velocity
    p.controller.checkin_time = Inf;
    p.controller.rate = 1; % Hz for control updates


% COMMUNICATION OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.comm.Rc_robot = Inf; % radius of communication
    p.comm.Rc_server = Inf; % radius of communication


% INITIALIZE SIMULATION PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p.sim.date = now;

    p.sim.linewidth = 2; % line width for plots
    p.sim.fs = 14; % font size
    
    p.sim.phd_grid_size = p.phd.grid_size; % size of grid for PHD estimate

    p.sim.plot_window = 50; % width of region around each bot

    p.sim.p_top = 0.04; % padding around animation windows
    p.sim.p_right = 0.02;

    % window of time to show check-ins
    p.sim.t_window = 50;
    
    

