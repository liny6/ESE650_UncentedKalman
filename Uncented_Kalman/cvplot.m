
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

X_hist = zeros(7, nt);


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

%% initialize variables

x(1) = 1;

q_cov = 1e-5;
w_cov = 3e-5;

accel_cov = 0.0014;
gyro_cov = 7.5e-5;

P = 0.1*diag([2*pi, 2*pi, 2*pi, pi, pi, pi]);
Q = diag([q_cov, q_cov, q_cov, w_cov, w_cov, w_cov]);
R = diag([accel_cov, accel_cov, accel_cov, gyro_cov, gyro_cov, gyro_cov]);

%% run unscented kalman filter

for i = 1:nt
    if i == 1;
        delta_t = 0.01;
    else
        delta_t = Imu.ts(i) - Imu.ts(i-1);
    end
    [x, P] = UnscentedKalman(x, P, Z_calib(:, i), delta_t, delta_t*Q, R);
    x(1:4) = x(1:4)/norm(x(1:4));
    X_hist(:, i) = x;
end

%% get X_hist to rotate gravity

g_hist = zeros(3, nt);

for i = 1:nt
    g_hist(:, i) = quatrot(quatinv(X_hist(1:4, i)), gravity);
end

figure

plot(Imu.ts, g_hist(1, :));
hold on
%plot(Imu.ts, Z_calib(1, :));
plot(Vicon.ts, Acc_GT(1,:));


figure

plot(Imu.ts, g_hist(2, :));
hold on
%plot(Imu.ts, Z_calib(2, :));
plot(Vicon.ts, Acc_GT(2,:));

figure

plot(Imu.ts, g_hist(3, :));
hold on
%plot(Imu.ts, Z_calib(3, :));
plot(Vicon.ts, Acc_GT(3,:));
