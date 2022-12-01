function h = run_simulation(h)

nr = length(h.robots);
m = min([h.params.env.rate, h.params.sensor.rate, h.params.controller.rate]);
h.params.sim.rate = lcm(h.params.env.rate/m, lcm(h.params.sensor.rate/m, h.params.controller.rate/m)) * m;

h = log_data(h, 0);

% Run the simulation
iter = 0; % counter for iterations
t = 1; % counter for measurements

if h.params.sim.animate == 0
    h.viz.t0 = CTimeleft(h.params.sim.tMax);
    h.viz.t0.timeleft();
end

K = length(h.params.env.ts_environment); % Nbr Arms
alpha0 = 1;
beta0 = 1;
[alphas, betas, ~, ~] = ...
    ThompsonSampling_Initialize(K, alpha0, beta0);

done = false;
while ~done
    % if get measurements
    if mod(iter, h.params.sim.rate / h.params.sensor.rate) == 0       
        % get measurements
        qt = [h.targets.q];
        qt = qt(:, [h.targets.active]); % ignore inactive targets       
    end
    
    if mod(iter, h.params.sim.rate / h.params.controller.rate) == 0
        % for exploring group
        if ~isempty(h.robots)
           [h.robots, alphas, betas] = ts(h, h.robots, qt, t, alphas, betas);                
        end    
    end

    %h = tracking(h, iter, t);
    h.robots = move_robots(h.robots, h.params.sim.rate);  
    h = target_move(h, t);
    
    % draw latest estimate
    if h.params.sim.animate > 0
        h = animate(h, t);
    end
    
    done = false;
    if t == h.params.sim.tMax
        done = true;
    end
    if all(~[h.robots.active])
%         done = true;
    end
    
    iter = iter+1;
    if mod(iter, h.params.sim.rate / h.params.sensor.rate) == 0
        t = t+1;
        if ~h.params.sim.animate
            h.viz.t0.timeleft();
        end
    end
    
    if done
        break;
    end
end

end
