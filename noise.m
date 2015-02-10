function [ noise_points ] = noise( points ,sigma_1)
sigma = [sigma_1 0 0;0 sigma_1 0 ;0 0 sigma_1];
noise_points=mvnrnd(points,sigma);
end

