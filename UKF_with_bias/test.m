close all
%% load data
addpath ../calib
[Imu, Vicon] = loadfiles(1);
gravity = [1; 1; 1];

wa1 = [10.6526; 10.5767; 10.4773];
wa2 = [-511; -500; 502];

[~, nt] = size(Imu.vals);

[~, ~, vicon_nt] = size(Vicon.rots);
Acc_GT = zeros(3, vicon_nt);

for i = 1:vicon_nt
    Acc_GT(:, i) = Vicon.rots(:, :, i)' * gravity;
end

[omega_t, OmegaGT] = getOmegaGT(Vicon);

%find covariance

R_cov = zeros(6, 1);

for i = 1:6
    R_cov(i) = cov(Imu.vals(i, 1:765));
end

X_hist = zeros(13, nt);


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

%% initialize variables

x(1) = 1;

q_cov = 8e-5;
w_cov = 5e-5;
bias_cov = [2e-3*ones(1, 3), 2e-3*ones(1, 3)];


acc_cov = 55*ones(1, 3);
gyro_cov = 8*ones(1, 3);

P = 1e-3*diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, 1, 1, 1]);
Q = diag([q_cov, q_cov, q_cov, w_cov, w_cov, w_cov, bias_cov]);
R = diag([acc_cov, gyro_cov]);

%% run unscented kalman filter

for i = 1:nt
    if i == 1;
        delta_t = 0.01;
    else
        delta_t = Imu.ts(i) - Imu.ts(i-1);
    end
    [x, P] = UnscentedKalman(x, P, Imu.vals(:, i), delta_t, delta_t*Q, R);
    x(1:4) = x(1:4)/norm(x(1:4));
    X_hist(:, i) = x;
    fprintf('%d\n', i);
end

%% get X_hist to rotate gravity

myrpy = quat2eul(X_hist(1:4, :)');
GTrpy = rotm2eul(Vicon.rots);

figure

plot(Imu.ts, myrpy(:, 1))
hold on
plot(Vicon.ts, GTrpy(:, 1))
%{
plot(Imu.ts, g_hist(1, :));
hold on
%plot(Imu.ts, Z_calib(1, :));
plot(Vicon.ts, Acc_GT(1,:));
%}

figure
plot(Imu.ts, myrpy(:, 2))
hold on
plot(Vicon.ts, GTrpy(:, 2))

%{
plot(Imu.ts, g_hist(2, :));
hold on
%plot(Imu.ts, Z_calib(2, :));
plot(Vicon.ts, Acc_GT(2,:));
%}

figure
plot(Imu.ts, myrpy(:, 2))
hold on
plot(Vicon.ts, GTrpy(:, 2))

figure
plot(Imu.ts, X_hist(5, :))
hold on
plot(omega_t, OmegaGT(1,:))