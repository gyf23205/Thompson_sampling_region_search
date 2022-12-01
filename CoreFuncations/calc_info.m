function h = calc_info(h, leader, t, nr)
% Calculates the mutual information
tic
r = h(leader);
order_approx = r.controller.order_approx;

pk0 = r.sensor.pk0; % probability of no false positive
mu = -log(pk0);

nrg = length(h); % number of robots in group

x0 = r.state.x;
w0 = r.state.w;

% find nbrs in graph for each robot
for i = 1:nrg
    if isempty(h(i).temp.path)
        nbrs = h(i).controller.neighbors{h(i).v, 1};
        % if too many neighbors, subsample according to distance
        nbrs = nbrs(h(i).controller.neighbors{h(i).v, 2} <= h(i).controller.maxStep);
    else
        nbrs = h(i).temp.path(1);
    end
    
    % build list of all possible locations for group
    if i == 1
        nodes = nbrs;
    else
        lx = length(nbrs);
        lZ = size(nodes,1);
        nodes = [kron(nodes,ones(lx,1)), kron(ones(lZ,1),nbrs)];
    end
end
Z = AllCombosBinary([false(1,nrg); true(1,nrg)]);

tLast = length(r.comm.out.msg); % number of time steps since last server comm
% use geometric cdf to describe return probability for each robot
p = r.comm.return_rate;% param of geometric dist
num_msg = 0;
for i = 0:tLast-1
    num_msg = num_msg + (tLast-i) * (1-p)^i * p;
end

% Find subset of footprint for each robot
nodes_unique = unique(nodes(:));
footprint = false(length(nodes_unique), length(w0));
for i = 1:length(nodes_unique)
    qi = r.controller.control_points(nodes_unique(i), :);
    fi = r.sensor.footprint;
    fi = bsxfun(@plus, fi, qi).';
    footprint(i,:) = inpolygon(x0(1,:), x0(2,:), fi(1,:), fi(2,:));
end

NoDetectAll = ones(length(nodes_unique), length(w0));
serverPd = zeros(length(nodes_unique), 1);
for i = 1:length(nodes_unique)
    qi = r.controller.control_points(nodes_unique(i), :).';
    foot = footprint(i,:);
    NoDetectAll(i,foot) = 1 - detect_prob_info(r, qi, x0(:,foot));
    
    serverPd(i) = detect_prob_server(r, qi, nr);
end

[goal, maxInfo] = calc_info_fast(nodes_unique, footprint, NoDetectAll, serverPd, ...
    nodes, Z, w0, mu, leader, order_approx, r.comm.return_rate, tLast);

if goal > 0 && length(nodes(goal,:))~=length(unique(nodes(goal,:)))
    keyboard
end

% find path to goal
for i = 1:nrg
    r = h(i);
    
    if isempty(r.temp.path)  && ~r.temp.checkin_mode
        if goal >0
            r.temp.path = reconstruct_path(r.controller.next, r.v, nodes(goal,i));
        else
            r.temp.path = reconstruct_path(r.controller.next, r.v, r.v);
        end
        
        % save # robots in group and computation time
        r.data.times(t) = toc;
        r.data.nBots(t) = nrg;
        r.data.Info(t) = maxInfo;
    end
    
    h(i) = r;
end
