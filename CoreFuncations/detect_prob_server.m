function pd = detect_prob_server(h, q, nr)

tau = h.server.comm.Rc;
s = h.server.access_points;

p_max = 0.8;

p0 = min((nr-1)*h.sensor.area/h.env.area, p_max);

pd = 0;
for i = 1:size(h.server.access_points,2)
    d = max(norm(q-s(:,i))-h.server.comm.Rc, 0);
    temp = p0*exp(-d/tau);
    if temp > pd
        pd = temp;
    end
end