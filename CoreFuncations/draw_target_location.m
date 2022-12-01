function q = draw_target_location(h)

if rand < 0.666
    % bounding box
    x0 = min(h.params.env.bndry(:,1));
    x1 = max(h.params.env.bndry(:,1))/3;
    y0 = min(h.params.env.bndry(:,2));
    y1 = max(h.params.env.bndry(:,2))/3;


    % calculate distance from edge
    dist = min([h.server.state.x(1,:) - x0; x1 - h.server.state.x(1,:); ...
        h.server.state.x(2,:) - y0; y1 - h.server.state.x(2,:)]/3, [], 1);

    wb = h.params.phd.birth_d - dist;
    wb(wb < 0) = 0;
    wb = cumsum(wb);

    r = wb(end) * rand;
    idx = find(r < wb, 1, 'first');

    q = [h.server.state.x(:,idx); 2 * pi * rand]/3;
    
else

    % bounding box
    x0 = min(h.params.env.bndry(:,1))+2*max(h.params.env.bndry(:,1))/3;
    x1 = max(h.params.env.bndry(:,1));
    y0 = min(h.params.env.bndry(:,2))+2*max(h.params.env.bndry(:,2))/3;
    y1 = max(h.params.env.bndry(:,2));


    % calculate distance from edge
    dist = min([h.server.state.x(1,:) - x0; x1 - h.server.state.x(1,:); ...
        h.server.state.x(2,:) - y0; y1 - h.server.state.x(2,:)]/3, [], 1);

    wb = h.params.phd.birth_d - dist;
    wb(wb < 0) = 0;
    wb = cumsum(wb);

    r = wb(end) * rand;
    idx = find(r < wb, 1, 'first');

    q = [2*(h.server.state.x(:,idx)+max(h.params.env.bndry(:,1)))/3; 2 * pi * rand+max(h.params.env.bndry(:,2))/3];
end















