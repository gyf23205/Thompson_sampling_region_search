function [r, alphas, betas] = ts(h, r, qt, t, alphas, betas)

for i = 1:length(r)        
        r(i) = get_measurements(r(i), qt, t);
        if isempty(r(i).z)
            reward = 0;
        else
            reward = 1;             
        end
        
        C=20;
        dist_min = Inf;
        for j = 1:length(alphas)
            dist = norm(r(i).q - h.params.env.ts_environment(j,:)');
            if dist < dist_min
                dist_min = dist;
                idx_min = j;
            end
        end
        [alphas, betas, ~, ~] = ...
            ThompsonSampling_ReceiveReward(alphas, betas, ...
            reward, idx_min, [], []);
        for j = 1:length(alphas)
            if alphas(j)+betas(j)>C
                for k = 1:length(alphas)
                    alphas(k)=alphas(k)*C/(C+1);
                    betas(k)=betas(k)*C/(C+1);
                end
                break;
            end
        end

    if norm(r(i).q - r(i).temp.goal) < 0.01
        ArmToPlay = ThompsonSampling_RecommendArm(alphas, betas);
        r(i).arm_played = ArmToPlay;
        sample_next = h.params.env.ts_environment(ArmToPlay,:)';
        r(i).temp.goal = sample_next; 
    end
end

        
        
        
        
        
        
        