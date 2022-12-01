function h = get_measurements(h, qt, t)

% Calc total detect prob for actual target locations
z = [];
for j = 1 : size(qt,2)
    xj = qt(1:2,j);
    TrueDetectProb = detect_prob(h.sensor, h.q, xj);
    if rand < TrueDetectProb
        z0 = xj + sqrt(h.sensor.CovD)*randn(2,1); % new measurement
        z = [z z0];
    end
end

h.data.z{t} = z;
h.z = z;
