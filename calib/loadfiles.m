function [imu_data, vicon_data] = loadfiles(num)

%num indicates which file the user wishes to load
imu_dataFileName = strcat(strcat('../imu/imuRaw', num2str(num)), '.mat');
ViconFileName = strcat(strcat('../vicon/viconRot', num2str(num)), '.mat');

imu_data = load (imu_dataFileName);
vicon_data = load (ViconFileName);

%% condition data
imu_data.vals(1, :) = - imu_data.vals(1, :);
imu_data.vals(2, :) = - imu_data.vals(2, :);
temp = imu_data.vals(4, :);
imu_data.vals(4:5, :) = imu_data.vals(5:6, :);
imu_data.vals(6, :) = temp;
% inverting the signs of Ax and Ay; move Wz down two rows

end