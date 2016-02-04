function [x, P] = UnscentedKalman(x_last, P_last, z_meas, delta_t, Q, R)

%% parameters and variables
n = length(x_last); % 7 in this homework
[covn, ~] = size(P_last); % 6 in this homework
W_sigma = zeros(covn, 2*covn); % difference to sigma points 6x6 in this homework
X_sigma = zeros(n, 2*covn); %sigma points 7x6 in this homework

%% generate sigma points
% first dilate covairance to obtain W_sigma
dilated_cov = 2*covn*(P_last + Q);
% do cholesky decomposition
S = chol(dilated_cov);
% put the columns into W_sigma
W_sigma(:, 1:covn) = S;
W_sigma(:, covn+1:end) = -S;
% calculate X_sigma from W_sigma





end