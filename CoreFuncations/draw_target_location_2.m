function q = draw_target_location_2(h)

% bounding box
x0 = min(h.params.env.bndry(:,1))+max(h.params.env.bndry(:,1))/2;
x1 = max(h.params.env.bndry(:,1));
y0 = min(h.params.env.bndry(:,2))+max(h.params.env.bndry(:,2))/2;
y1 = max(h.params.env.bndry(:,2));

% calculate distance from edge
dist = min([h.server.state.x(1,:) - x0; x1 - h.server.state.x(1,:); ...
    h.server.state.x(2,:) - y0; y1 - h.server.state.x(2,:)]/2, [], 1);

wb = h.params.phd.birth_d - dist;
wb(wb < 0) = 0;
wb = cumsum(wb);

r = wb(end) * rand;
idx = find(r < wb, 1, 'first');

q = [h.server.state.x(:,idx)/2+max(h.params.env.bndry(:,1))/2; 2 * pi * rand+max(h.params.env.bndry(:,2))];