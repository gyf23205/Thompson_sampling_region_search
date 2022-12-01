function h = init_targets(h)
% Randomize initial target locations

h.env.targets = h.env.targets + (rand(size(h.env.targets)) - 0.5);
