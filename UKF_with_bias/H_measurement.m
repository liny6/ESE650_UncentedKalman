function Z_sigma = H_measurement( Y_sigma )
% this measurement suite contains acclerometer and gyro

gravity = [0; 0; 9.81];% expected gravity when it's at orientation [1; 0; 0; 0]
[~, nc] = size(Y_sigma);
Z_sigma = zeros(6, nc);
gravity_bias = Y_sigma(8:10, :);
omega_bias = Y_sigma(11:13, :);
gravity_sense = 10.5;
omega_sense = 70;

for i = 1:nc
    q = quatinv(Y_sigma(1:4, i));
    Z_sigma(1:3, i) = gravity_sense.*quatrot(q, gravity) + gravity_bias(:, i);
    Z_sigma(4:6, i) = (Y_sigma(5:7, i).*omega_sense + omega_bias(:, i));
end



end

