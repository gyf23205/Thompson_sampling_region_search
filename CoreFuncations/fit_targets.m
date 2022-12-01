function h = fit_targets(h, t, trueTargets)
% Perform clustering to extract target locations from PHD
occ_thresh = h.phd.occ_thresh;
distThresh = h.phd.dist_thresh;

% keep all particles above threshold weight
state = h.state;
x = state.x(:, state.w > occ_thresh).';
w = state.w(state.w > occ_thresh);

% find pairwise distance between all remaining particles
d = pdist(x);
d = squareform(d);

% cluster particles within distance threshold
n = size(d, 1);
idx = zeros(n, 1);
id = 1;
while any(idx == 0)
    idx_old = idx;
    ind = find(idx == 0, 1, 'first');
    idx(d(:, ind) < distThresh) = id;
    while ~all(idx == idx_old)
        idx_old = idx;
        ind = find(idx == id, 1, 'first');
        idx(d(:, ind) < distThresh) = id;
    end
    id = id + 1;
end

% find weighted average location of particle clusters
targets = zeros(2, id-1);
weights = zeros(1, id-1);
keep = true(id-1, 1);
for i = 1:id-1
    targets(:, i) = w(idx==i) * x(idx==i,:) / sum(w(idx==i));
    weights(i) = sum(w(idx==i));
    if weights(i) < occ_thresh %0.5
        keep(i) = false;
    end
end
targets = targets(:, keep);
weights = weights(keep);

[dists, ind] = pdist2(trueTargets.', targets.', 'euclidean', 'Smallest', 1);
idx = dists < distThresh;

% save target locations
h.temp.targets = targets;
h.temp.weights = weights;

if t > 0
    h.data.num_targets_found(t) = sum(idx);
    h.data.num_false_targets(t) = sum(~idx);
    h.data.target_error(ind(idx), t) = dists(idx);
    h.data.target_weight(ind(idx), t) = weights(idx);
end

% TODO: track # of targets in env and plot
% TODO: track P(true target set)?
% TODO: track OSPA error?
