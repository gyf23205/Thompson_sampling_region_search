function h = init_arrays(h, tMax)
% Initializes the arrays necessary to store simulation results and perform
% calculations

% DATA SAVED OVER TIME %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h.data.z = cell(1, tMax); % store measurement sets

h.data.lambda = zeros(1,tMax+1); % number of targets in voronoi cell
h.data.lambda_true = zeros(1,tMax+1); % true number of targets in voronoi cell

h.data.lambda_foot = zeros(1,tMax+1); % number of targets in footprint
h.data.lambda_foot_true = zeros(1,tMax+1); % number of targets in footprint

h.data.entropy = zeros(1,tMax+1); % entropy of PHD estimate
h.data.term_criterion = zeros(1,tMax+1); % termination criterion

h.data.q = zeros(length(h.q), tMax+1); % locations
h.data.path = cell(1, tMax); % paths travelled
h.data.active = true(1, tMax); % whether agent is active

h.data.ospa = zeros(1, tMax); % OSPA error
h.data.num_targets_found = zeros(1, tMax); % number of targets extracted from PHD
h.data.num_targets_matched = zeros(1, tMax); % number of targets extracted from PHD matched to true targets


% TEMPORARY DATA FOR CALCULATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h.temp.Group = h.id; % store robots in group
h.temp.path = []; % store path
h.temp.x_new_send = [];
h.temp.w_new_send = [];
