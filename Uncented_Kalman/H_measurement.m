function Z_sigma = H_measurement( Y_sigma )
% this measurement suite contains acclerometer and gyro

gravity = [0; 0; 1];% expected gravity when it's at orientation [1; 0; 0; 0]
[nr, nc] = size(Y_sigma);
Z_sigma = zeros(6, nc);

for i = 1:nc
    q = quatinv(Y_sigma(1:4, i));
    Z_sigma(1:3, i) = quatrot(q, gravity);
end

Z_sigma(4:6, :) = Y_sigma(5:7, :);

end

