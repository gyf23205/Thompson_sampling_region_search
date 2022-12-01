function r = move_robots(r, rate)

for i = 1:length(r)
    dist = norm(r(i).temp.goal - r(i).q);
    dist_action = r(i).controller.vel / rate;
    if dist < dist_action
        r(i).q = r(i).temp.goal;
        r(i).temp.busy = false;
    else
        r(i).q = r(i).q + (r(i).temp.goal - r(i).q) / dist * dist_action;
    end
end
