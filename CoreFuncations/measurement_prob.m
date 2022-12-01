function MeasurementProb = measurement_prob(sensor, z, x)
x = x.';

J = size(x, 1); % number of particles
M = size(z, 2); % number of detections

MeasurementProb = zeros(M, J);

for i = 1:M
    MeasurementProb(i, :) = mvnpdf(x, z(:,i).', sensor.CovD*eye(2));
end
