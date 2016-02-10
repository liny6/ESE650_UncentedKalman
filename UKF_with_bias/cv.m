%% load data
addpath ../calib
[Imu, Vicon] = loadfiles(1);
gravity = [1; 1; 1];

wa1 = [10.6526; 10.5767; 10.4773];
wa2 = [-511; -500; 502];

Z_calib = [bsxfun(@times, bsxfun(@minus, Imu.vals(1:3, :), wa2), 1./wa1); (Imu.vals(4:6, :) - 373)/69];
[~, nt] = size(Z_calib);

[~, ~, vicon_nt] = size(Vicon.rots);
Acc_GT = zeros(3, vicon_nt);

for i = 1:vicon_nt
    Acc_GT(:, i) = Vicon.rots(:, :, i)' * gravity;
end

X_hist = zeros(13, vicon_nt);


%% Declare Variables

% state vector
x = zeros(13,1);
x(8:10) = wa2;
x(11:13) = [373, 373, 370];
% measurement vector
z_meas = zeros(6, 1);
% state covariance
P = zeros(12);
% process model covariance
Q = zeros(12);
% measurement model covariance
R = zeros(6);



%% cross validation
q_best = 0;
w_best = 0;
err_best = inf;
count = 0;

for q_cov = linspace(5e-6, 5e-5, 10)
    for acc_cov = linspace(10, 20, 10)
        count = count+1;
        % initialize variables
        x(1:4) = [1; 0; 0; 0];
        w_cov = 3e-5;
        bias_cov = 1e-2*ones(1, 6);
        
        gyro_cov = 0.0035*ones(1, 3);
        
        P = 1e-2*diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, 1, 1, 1]);
        Q = diag([q_cov, q_cov, q_cov, w_cov, w_cov, w_cov, bias_cov]);
        R = diag([acc_cov*ones(1,3), gyro_cov]);
        
        %% run unscented kalman filter
        
        for i = 1:vicon_nt
            if i == 1;
                delta_t = 0.01;
            else
                delta_t = Imu.ts(i) - Imu.ts(i-1);
            end
            [x, P] = UnscentedKalman(x, P, Imu.vals(:, i), delta_t, delta_t*Q, R);
            x(1:4) = x(1:4)/norm(x(1:4));
            X_hist(:, i) = x;
        end
        
        err = norm(X_hist(1:3, :) - Acc_GT, 'fro');
        
        if err < err_best
            q_best = q_cov;
            w_best = w_cov;
            err_best = err;
        end
        fprintf('iteration: %d, q_cov = %.3e, acc_cov = %.3d, err = %.3f, err_best = %.3f\n', count, q_cov, acc_cov, err, err_best);
    end
end
