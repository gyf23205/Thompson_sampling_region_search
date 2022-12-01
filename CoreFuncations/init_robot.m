function r = init_robot(h, q, id)

% ENVIRONMENT OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r.env = h.params.env;
r.target = h.params.target;


% CONTROLLER OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r.controller = h.params.controller;
r.active = true;


% SENSOR OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r.sensor = h.params.sensor;


% INITIALIZE DATA STRUCTURES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r.id = id; % id for communication
r.q = q;
r.z = [];
r = init_arrays(r, h.params.sim.tMax);
r.arm_played = [];
r.temp.goal = q;
