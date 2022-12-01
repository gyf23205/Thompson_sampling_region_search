function t = init_target(h, q, id)

% This initializes a robot with initial state q using the parameters found
% in the structure h

t.id = id; % id for communication
t.q = q;
t.active = true;

% INITIALIZE DATA STRUCTURES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t.data.q = [t.q, zeros(length(t.q), h.params.sim.tMax)]; % locations
t.data.t_birth = 1;
t.data.t_death = Inf;
