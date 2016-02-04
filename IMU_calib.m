function z = IMU_calib(z_ADC)
%IMU_CALIB Summary of this function goes here
%   Detailed explanation goes here
[~, nt] = size(z_ADC);

Ka = 1/10;
Kw = 1/60;
Ba = [-500; -500; 500];
Bw = [375; 375; 375];

z = zeros(6, nt);

z(1:3, :) = Ka * (bsxfun(@minus, z_ADC(1:3, :), Ba));
z(4:6, :) = Kw * (bsxfun(@minus, z_ADC(4:6, :), Bw));
end

