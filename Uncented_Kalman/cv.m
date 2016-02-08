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

X_hist = zeros(3, vicon_nt);


%% Declare Variables

% state vector
x = zeros(7,1);
% measurement vector
z_meas = zeros(6, 1);
% state covariance
P = zeros(6);
% process model covariance
Q = zeros(6);
% measurement model covariance
R = zeros(6);



%% cross validation
q_best = 0;
w_best = 0;
err_best = inf;
count = 0;

while( count < 1000 )
        count = count+1;
        % initialize variables
        x = [1; 0; 0; 0; 0; 0; 0];
        err = 0;
        q_cov = 1e-5;%(1e-3 - 1e-5)*rand + 1e-5;
        w_cov = 0.5;%(5e-1 - 1e-3)*rand + 1e-3;
        accel_cov = 0.0004;
        gyro_cov = 7.5e-4;
        
        P = 1e-3*diag([2*pi, 2*pi, 2*pi, pi, pi, pi]);
        Q = diag([q_cov, q_cov, q_cov, w_cov, w_cov, w_cov]);
        R = diag([accel_cov, accel_cov, accel_cov, gyro_cov, gyro_cov, gyro_cov]);
        
        %% run unscented kalman filter
        
        for i = 1:vicon_nt
            if i == 1;
                delta_t = 0.01;
            else
                delta_t = Imu.ts(i) - Imu.ts(i-1);
            end
            [x, P] = UnscentedKalman(x, P, Z_calib(:, i), delta_t, delta_t*Q, R);
            x(1:4) = x(1:4)/norm(x(1:4));
            X_hist(:, i) = quatrot(quatinv(x), gravity);
            err = err + norm(X_hist(:, i) - Acc_GT(:, i));
        end
        
        
        if err < err_best
            q_best = q_cov;
            w_best = w_cov;
            accel_best = accel_cov;
            gyro_best = gyro_cov;
            err_best = err;
            
            figure(1)
            
            plot(Vicon.ts, X_hist(1, :));
            %plot(Imu.ts, Z_calib(1, :));
            hold on
            plot(Vicon.ts, Acc_GT(1,:));
            hold off
            
            
            figure(2)
            
            plot(Vicon.ts, X_hist(2, :));
            hold on
            %plot(Imu.ts, Z_calib(2, :));
            plot(Vicon.ts, Acc_GT(2,:));
            hold off
            
            figure(3)
            
            plot(Vicon.ts, X_hist(3, :));
            hold on
            %plot(Imu.ts, Z_calib(3, :))\;
            plot(Vicon.ts, Acc_GT(3,:));
            hold off
        end
        pause(0.1)
        fprintf('count: %d, error: %.3f, q = %.3e, w = %.3e \n', count, err, q_cov, w_cov);
        fprintf('best_err: %.3f, best q = %.3e, w = %.3e \n', err_best, q_best, w_best);
end
