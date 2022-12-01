function h = calc_centroid(h)
nr = length(h);

[V, C] = VoronoiBounded([h.q]', h(1).env.bndry);

for i = 1:nr
    poly = V(C{i},:); % extract Voronoi region for robot
    idx = inpolygon(h(i).state.x(1,:), h(i).state.x(2,:), poly(:,1), poly(:,2)); % compute particles in region
%     pd = detect_prob(h(i), h(i).q, h(i).state.x(:,idx));
%     w = h(i).state.w(idx) .* pd;
    w = h(i).state.w(idx);
    cent = h(i).state.x(:,idx) * w' / sum(w); % compute weighted centroid of region
    if isnan(cent)
        cent = h(i).q;
%         keyboard
    end
    h(i).temp.goal = cent;
end
