function [x, P] = UnscentedKalman(x_last, P_last, z_meas, delta_t, Q, R)

grav_len = norm((z_meas(1:3) - [-511; -500; 502])/10.5);
z_meas(1:3) = (z_meas(1:3) - [-511; -500; 502])/10.5/grav_len;
%% parameters and variables
x = zeros(10, 1);
n = length( x_last ); % 19 with calibration
[covn, ~] = size( P_last ); % 18 with calibration
W_sigma = zeros( covn, 2*covn ); % difference to sigma points 18x18 with calibration

%% generate sigma points
% first dilate covairance to obtain W_sigma
dilated_cov = 2*covn*( P_last + Q );
% do cholesky decomposition
S = chol( dilated_cov );
% put the columns into W_sigma
W_sigma( :, 1:covn ) = S;
W_sigma( :, covn+1:end ) = -S;
% calculate X_sigma from W_sigma
X_sigma = WtoX_quat( W_sigma, x_last );

%% Use process model to transform X_sigma into Y sigma
Y_sigma = A_process( X_sigma, delta_t );

%% estimate a priori x_a_priori with gradient descent
[x_a_priori, errors] = FindQuatMean( Y_sigma );

%% Transform Y_sigma to transformed error
W_sigma(1:3, :) = errors; %take the error straight from the last cycle of gradient descent
W_sigma(4:9, :) = bsxfun(@minus, Y_sigma(5:10, :), x_a_priori(5:10));

%% calculate P_a_priori from the new W_sigma
P_a_priori = 1/2/n*(W_sigma*W_sigma');

%% depending on delta_t, decide if I want to take in the measurement

if delta_t < 0.001 || grav_len > 10.2 || grav_len < 9.5
    x = x_a_priori;
    P = P_last;
    grav_len
else
    %% project measurements and find innovation
    
    % first, find the sigma points for measurements
    Z_sigma = H_measurement( Y_sigma );
    % take the mean
    z_a_priori = mean(Z_sigma, 2);
    % find innovation
    nu = z_meas - z_a_priori;
    
    %% find innovation's covariance
    Z_sigma_err = bsxfun(@minus, Z_sigma, z_a_priori);
    P_zz = 1/2/n * (Z_sigma_err * Z_sigma_err');
    P_nu = P_zz + R;
    
    %% find cross correlation P_xz
    P_xz = 1/2/n * W_sigma * Z_sigma_err';
    
    %% find kalman gain and update state vector/ covariance
    % kalman gain
    K = P_xz/P_nu;
    % update x
    x_delta = K*nu;
    x(1:4) = quatmult(x_a_priori(1:4), rotvec2quat(x_delta(1:3)));
    x(5:10) = x_a_priori(5:10) + x_delta(4:9);
    % update P
    P = P_a_priori - K * P_nu * K';  
end

end
