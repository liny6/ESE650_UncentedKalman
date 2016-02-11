function Y_sigma = A_process( X_sigma, delta_t )
% use rotation process model to transform sigma points

% first calculate rotation occured based on angular velocity and deltat
[nr, nc] = size(X_sigma);
Y_sigma = zeros(nr, nc);

for i = 1:nc
    q_X = X_sigma(1:4, i);
    omega = X_sigma(5:7, i) * delta_t;
    q_diff = rotvec2quat(omega);
    Y_sigma(1:4, i) = quatmult(q_X, q_diff); 
end

Y_sigma(5:10, :) = X_sigma(5:10, :);
