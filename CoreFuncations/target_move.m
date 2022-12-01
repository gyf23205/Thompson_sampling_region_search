function h = target_move(h, t)

R = @(th) [cos(th) -sin(th); sin(th) cos(th)];
x0 = min(h.params.env.bndry(:,1));
x1 = max(h.params.env.bndry(:,1)/3);
y0 = min(h.params.env.bndry(:,2));
y1 = max(h.params.env.bndry(:,2)/3);

x0p = min(h.params.env.bndry(:,1))+2*max(h.params.env.bndry(:,1))/3;
x1p = max(h.params.env.bndry(:,1));
y0p = min(h.params.env.bndry(:,2))+2*max(h.params.env.bndry(:,2))/3;
y1p = max(h.params.env.bndry(:,2));

% move targets
for i = 1:length(h.targets)
    if h.targets(i).active
        q = h.targets(i).q;
        q(1:2) = q(1:2) + R(q(3)) * h.params.target.v_max * rand / h.params.sim.rate * [1; 0];
        q(3) = q(3) + h.params.target.w_max * randn / h.params.sim.rate;
        h.targets(i).q = q;
        h.targets(i).data.q(:,t+1) = q;
        dist1 = min([q(1) - x0; x1 - q(1); q(2) - y0; y1 - q(2)]);
        dist2 = min([q(1) - x0p; x1p - q(1); q(2) - y0p; y1p - q(2)]);
        if dist1 < 0 && dist2 < 0
            h.targets(i).active = false; % outside of env
        else
            if dist1 < h.params.phd.ps_d
                ps = h.params.phd.ps0 + ...
                    (h.params.phd.ps1 - h.params.phd.ps0) * dist1 / h.params.phd.ps_d;
            elseif dist2 < h.params.phd.ps_d
                ps = h.params.phd.ps0 + ...
                    (h.params.phd.ps1 - h.params.phd.ps0) * dist2 / h.params.phd.ps_d;
            else
                ps = h.params.phd.ps1;
            end
            if rand > ps
                h.targets(i).active = false;
            end
        end
        if ~h.targets(i).active
            h.targets(i).data.t_death = t;
        end
    end
end

% add targets
nt = poissrnd(h.params.phd.mu_b); % number of births
for i = 1:nt
    n = length(h.targets);
    h.targets(n+1) = init_target(h, draw_target_location(h), n+1);
    h.targets(n+1).data.t_birth = t;
end




















