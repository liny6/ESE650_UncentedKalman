[Imu, Vicon] = loadfiles(1);
%{
%% condition data
Imu.vals(1, :) = - Imu.vals(1, :);
Imu.vals(2, :) = - Imu.vals(2, :);
temp = Imu.vals(4, :);
Imu.vals(4:5, :) = Imu.vals(5:6, :);
Imu.vals(6, :) = temp;
% inverting the signs of Ax and Ay; move Wz down two rows
%}
%% Get and Plot ground truth vs Measured value

%accelerometer part
Acc_GT = getAccGT(Vicon);
GT_vs_M_plot(Vicon.ts, Acc_GT, Imu.ts, Imu.vals, 'Gravity')

starttime = max(Imu.ts(1), Vicon.ts(1));
endtime = min(Imu.ts(end), Vicon.ts(end));

t_sample = starttime:0.01:endtime;

Acc_GT_int = zeros(3, length(t_sample)); %interpolated acceleration ground truth
Acc_meas_int = zeros(3, length(t_sample)); %interpolated acceleration measured

wa1 = zeros(3, 1);
wa2 = zeros(3, 1);

for i = 1:3
    Acc_GT_int(i, :) = interp1(Vicon.ts, Acc_GT(i, :), t_sample);
    Acc_meas_int(i, :) = interp1(Imu.ts, Imu.vals(i, :), t_sample);
    C = polyfit(Acc_GT_int(i,:), Acc_meas_int(i,:), 1);
    wa1(i) = C(1);
    wa2(i) = C(2);
end

Acc_meas_calib = bsxfun(@times, bsxfun(@minus, Acc_meas_int, wa2), 1./wa1);

%gyro part

[omega_t, OmegaGT] = getOmegaGT(Vicon);
GT_vs_M_plot(omega_t, OmegaGT, Imu.ts, Imu.vals(4:end, :), 'Angular Velocity')

starttime = max(Imu.ts(1), omega_t(1));
endtime = min(Imu.ts(end), omega_t(end));

t_sample = starttime:0.01:endtime;

Omega_GT_int = zeros(3, length(t_sample));
Omega_meas_int = zeros(3, length(t_sample));

ww1 = zeros(3, 1);
ww2 = zeros(3, 1);

for i = 1:3
    Omega_GT_int(i, :) = interp1(omega_t, OmegaGT(i, :), t_sample);
    Omega_meas_int(i, :) = interp1(Imu.ts, Imu.vals(i+3, :), t_sample);
    C = polyfit(Omega_GT_int(i,:), Omega_meas_int(i,:), 1);
    ww1(i) = C(1);
    ww2(i) = C(2);
end

omega_meas_calib = 1/69*(Omega_meas_int - 373);

%%


