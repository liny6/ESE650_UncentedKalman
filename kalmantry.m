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

P = diag([2*pi, 2*pi, 2*pi, pi, pi, pi]);
Q = diag([0.05, 0.05, 0.05, 0.01, 0.01, 0.01]);
R = diag([0.05, 0.05, 0.05, 0.01, 0.01, 0.01]);

%% run unscented kalman filter