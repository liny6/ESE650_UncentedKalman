function [imu_data, vicon_data] = loadfiles(num)

%num indicates which file the user wishes to load
ImuFileName = strcat(strcat('imu/imuRaw', num2str(num)), '.mat');
ViconFileName = strcat(strcat('vicon/viconRot', num2str(num)), '.mat');

imu_data = load (ImuFileName);
vicon_data = load (ViconFileName);