function h = calc_info_old(h, leader, t, nr)
% Calculates the mutual information
tic
r = h(leader);
order_approx = r.controller.order_approx;
computeInfoTerms = false;

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

lambdaE = sum(r.state.w); % expected number of targets

% compute info for each config %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Info = zeros(1,length(nodes));
if computeInfoTerms
    InfoPlot = zeros(order_approx, length(nodes));
end
for k = 1:size(nodes,1)
    % ensure robots are not as same locations to avoid collision
    if nrg > 1 && length(unique(nodes(k,:))) ~= nrg
        Info(k) = -Inf;
        continue;
    end
    % find particles in union of robot footprints
    foot = false(1, length(w0));
    for i = 1:size(nodes,2)
        foot = foot | footprint(nodes(k,i)==nodes_unique,:);
    end
    w = w0(:, foot);
    lambda = sum(w);
    
    % compute probablilty of no detection for each robot
    NoDetect = zeros(nrg, length(w));
    for i = 1 : nrg
        NoDetect(i,:) = NoDetectAll(nodes(k,i)==nodes_unique, foot);
    end
    
    % compute alpha values
    alpha = zeros(size(Z,1), 1);
    for i = 1 : size(alpha,1)
        alpha(i) = sum(prod(NoDetect(~Z(i,:), :),1).*w);
    end
    
    % compute P(Z)
    Pz = zeros(1,size(Z,1));
    for i = 1:size(Z,1)-1
        z = Z(i,:);
        nz0 = sum(~z); % # not detections
        
        idx = all(~Z(:,~z), 2); % rows to count up
        nz1 = sum(~Z(idx,z), 2); % extra # not detections in each row
        
        Pz(i) = sum(((-1).^nz1) .* exp(-(lambda + mu*(nz0+nz1) - alpha(idx))));
    end
    Pz(end) = 1 - sum(Pz(1:end-1));
    if any( Pz < 0 | Pz > 1 )
        disp('Py not a probability')
        keyboard
        error('Py not a probability')
    end
    
    % compute H(Z)
    Hz = -sum(Pz .* log(Pz));
    
    if computeInfoTerms; InfoPlot(:, k) = Hz; end
    
    % compute H(Z | X)
    Hzx = 0;
    for i = 1:nrg
        foot = footprint(nodes(k,i)==nodes_unique,:);
        NoDetecti = NoDetectAll(nodes(k,i)==nodes_unique, foot);
        wi = w0(:, foot);
        lambda = sum(wi); % only look at footprint since outside of this
            % PHD will cancel in the lambda and alpha terms
        
        beta = -sum(wi .* NoDetecti .* log(NoDetecti));
        
        % calculate approximation terms
        alphaH = zeros(1, order_approx);
        temp = wi;
        for j = 1:order_approx
            temp = NoDetecti .* temp;
            alphaH(j) = sum(temp);
        end
        
        % calculate H(Zi | X)
        Hzix0 = exp(-(lambda + mu - alphaH(1))) * (beta + mu);
        Hzix1 = exp(-(lambda + mu - alphaH(1)));
        if computeInfoTerms; 
            InfoPlot(:, k) = InfoPlot(:, k) - Hzix0;
            InfoPlot(1, k) = InfoPlot(1, k) - Hzix1;
        end
        for l = 2:order_approx
            c_l = 1/(l*(l-1));
            Hzix1 = Hzix1 - c_l*exp(-(lambda + mu*l - alphaH(l)));
            if computeInfoTerms; InfoPlot(l, k) = InfoPlot(l, k) - Hzix1; end
        end
        % calculate H(Zi | X) using Monte Carlo integration
        if computeInfoTerms
            nSamples = 1e4;
            Hzix1 = 0;
%             Hplot = zeros(1, nSamples);
            nTar = poissrnd(lambda, [1, nSamples]); % draw # of targets
            wiNorm = cumsum(wi/lambda);
            for l = 1:nSamples
                p1 = pk0;
%                 pX = exp(-lambda);
                for m = 1:nTar(l)
                    % draw targets from D(x) and construct p(z=1)
                    vi = find(rand < wiNorm, 1, 'first');
                    p1 = p1*NoDetecti(vi);
%                     pX = pX*wi(vi);
                end
                p1 = 1 - p1; % get prob of a detection
                
                if p1 > 0
                    Hzix1 = Hzix1 - p1*log(p1); % divide by p(X) to weight properly
                end
%                 Hplot(l) = Hzix1/l;
            end
            Hzix1 = Hzix1/nSamples;

%             Hplot2 = zeros(1, order_approx);
%             Hzix1 = exp(-(lambda + mu - alphaH(1)));
%             Hplot2(1) = Hzix1;
%             for l = 2:order_approx
%                 c_l = 1/(l*(l-1));
%                 Hzix1 = Hzix1 - c_l*exp(-(lambda + mu*l - alphaH(l)));
%                 Hplot2(l) = Hzix1;
%             end
%             figure(1); plot(1:nSamples, Hplot)
%             figure(2); plot(1:order_approx, Hplot2)
%             if abs(Hplot(2000) - Hplot2(20)) > 0*.005
%                 figure(1); clf; 
%                 plot_options = {'LineWidth', 2, 'FontName', 'bitstream charter', 'FontSize', 16, 'FontWeight', 'bold'};
%                 plotxx(1:nSamples, Hplot, 1:order_approx, Hplot2, {'# particles', '# terms'}, 'Entropy', plot_options);
%             	pause
%             end
        end
        
        Hzx = Hzx + (Hzix0 + Hzix1);
    end
    Info(k) = Hz - Hzx;
    
    % Add info of message server if other bots %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nr == 1; continue; end % if only 1 robot, skip this step
    
    % find prob for each # detections
    pd = serverPd(nodes(k,leader) == nodes_unique);
    nZ = 1;%nr-1;
    pZ = zeros(1+nZ, 1); % each other robot generates msg each time step
    for m = 0 : nZ % |C^1| (so nZ-m = |C^0|)
        for n = 0 : m % |C'| (C' \subset C^1)
            pZ(m+1) = pZ(m+1) + nchoosek2(m, n) * (-1)^n * exp(-lambdaE * (1 - (1-pd)^(nZ-m+n)) - (nZ-m+n)*mu);
        end
    end
    
    % find prob for each measurement prob
    Hz = 0;
    pTotal = 0;
    for i = 0:nZ
        pTotal = pTotal + pZ(i+1)*nchoosek2(nZ,i);
        Hz = Hz - nchoosek2(nZ,i) * pZ(i+1) * log(pZ(i+1));
    end
    assert(abs(pTotal-1) < 1e-3);

    beta = -lambdaE * (1-pd) * log(1-pd);
    Hzx0 = exp(-lambdaE*(1 - (1-pd)) - mu) * (beta + mu);
    Hzx1 = exp(-lambdaE*(1 - (1-pd)) - mu);
    if computeInfoTerms
        InfoPlot(:, k) = InfoPlot(:, k) + (nr-1) * num_msg * Hz;
    	InfoPlot(:, k) = InfoPlot(:, k) - (nr-1) * num_msg * Hzix0;
        InfoPlot(1, k) = InfoPlot(1, k) - (nr-1) * num_msg * Hzix1;
    end
    for l = 2:order_approx
        c_l = 1/(l*(l-1));
        Hzx1 = Hzx1 - c_l * exp(-lambdaE * (1 - (1-pd)^l) - mu*l);
        if computeInfoTerms; InfoPlot(l, k) = InfoPlot(l, k) - (nr-1) * num_msg * Hzix1; end
    end
    Hzx = Hzx0 + Hzx1;
    
    % calculate H(Zi | X) using Monte Carlo integration
    if computeInfoTerms
        nSamples = 5e2;
        Hzx1 = 0;
%         Hplot = zeros(1, nSamples);
        nTar = poissrnd(lambdaE, [1, nSamples]);
        for l = 1:nSamples
            p1 = pk0*(1-pd)^nTar(l);
            p1 = 1 - p1;

            if p1 > 0
                Hzx1 = Hzx1 - p1*log(p1);
            end
%             Hplot(l) = Hzx1/l;
        end
        Hzx1 = Hzx1/nSamples;
        Hzx = Hzx0 + Hzx1;
%             plot(Hplot)
%             keyboard
        
%         Hplot2 = zeros(1, order_approx);
%         Hzx1 = exp(-lambdaE*(1 - (1-pd)) - mu);
%         Hplot2(1) = Hzx1;
%         for l = 2:order_approx
%             c_l = 1/(l*(l-1));
%             Hzx1 = Hzx1 - c_l * exp(-lambdaE * (1 - (1-pd)^l) - mu*l);
%             Hplot2(l) = Hzx1;
%         end
%         figure(1); plot(1:nSamples, Hplot)
%         figure(2); plot(1:order_approx, Hplot2)
%         Hplot2(end)/Hplot(end)
%         keyboard
    end
    
    Info(k) = Info(k) + (nr-1) * num_msg * (Hz - Hzx);
end

% find goal node
[maxInfo, goal] = max(Info);
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

%     % compare decisions made for different # terms in MI approx
%     if computeInfoTerms
%         [~, idx] = max(InfoPlot, [], 2);
%         nTerms = find(nodes(idx,ind) ~= goal, 1, 'last');
%         if isempty(nTerms)
%             h.data.nTermsNeeded(t) = 1;
%         else
%             h.data.nTermsNeeded(t) = nTerms + 1;
%         end
%         h.data.goal(t) = goal;
%         h.data.decisions(:,t) = nodes(idx,ind);
%     end
    h(i) = r;
end



function x = nchoosek2(n, k)

if k == 1 || k == n-1
    x = n;
elseif k == 0 || k == n
    x = 1;
else
    x = nchoosek2(n-1, k) + nchoosek2(n-1, k-1);
end




