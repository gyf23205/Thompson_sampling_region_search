function s = init_server(h)

% get robot params
s = init_robot(h, [], 0);

% remove unnecessary fields from server
s = rmfield(s, {'controller', 'env', 'q'});
s.data = rmfield(s.data, {'z', 'q', 'path'});
s.env.area = h.params.env.area;
s.env.bndry = h.params.env.bndry;
