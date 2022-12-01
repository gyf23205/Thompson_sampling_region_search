function h = log_data(h, t)

targets = [h.targets.q];
targets = targets(:,[h.targets.active]);

% robots
for i = 1:length(h.robots)
    % pose
    h.robots(i).data.q(:,t+1) = h.robots(i).q;
end
