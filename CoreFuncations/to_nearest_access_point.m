function h = to_nearest_access_point(h)

% find control points nearest server
[~, goals] = pdist2(h.controller.control_points, h.server.access_points.', ...
                    'euclidean', 'Smallest', 1);

% find nearest server
dists = h.controller.dists;
dist_to_goal = dists(h.v, goals);
[~, idx] = min(dist_to_goal);
goal = goals(idx);

% find path to server
h.temp.path = reconstruct_path(h.controller.next, h.v, goal);
