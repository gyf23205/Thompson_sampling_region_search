function PDetect = detect_prob_info(h, qi, x)
% Returns the probability of a robot detecting 1 from event s=1 at qj

Sigma = h.sensor.Sigma;
Pfn = h.sensor.Pfn;

footprint = bsxfun(@plus, h.sensor.footprint.', qi);

idx = inpolygon(x(1,:), x(2,:), footprint(1,:), footprint(2,:));
idx(h.data.located) = false; % ignore spots where have detection

r = sum(bsxfun(@minus, x(:,idx), qi).^2, 1); % squared distance

J = size(x, 2); % number of particles

PDetect = zeros(1, J);
PDetect(idx) = (1-Pfn) * exp(-r / (2*Sigma^2));
