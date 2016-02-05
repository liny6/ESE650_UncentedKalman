function [mu, errors] = FindQuatMean( Y_sigma )
%use gradient descent to find quaternion mean

[nr, nc] = size(Y_sigma);
mu = zeros(nr, 1);
%initialize q_t to be the first Y_sigma
q_t = Y_sigma(1:4, 1);
err = zeros(3, nc);
err_norm = 1;

%loop until convergence
while( err_norm > 0.001 )
    for i = 1:nc
        qi = Y_sigma(1:4, i);
        qe = quatmult(qi, quatinv(q_t)); %quaternion error
        %convert error quaternion to differential rotation vector
        err(:, i) = quat2rotvec(qe);
    end
    %take the average of all the error rotation vectors
    e_avg = 1/2/nc * (sum(err, 2));
    err_norm = norm(e_avg);
    %update the mean
    q_t = quatmult( rotvec2quat(e_avg), q_t);
end

%calculate the mean of the angular velocities
mu(5:7) = mean(Y_sigma(5:7, :), 2);
mu(1:4) = q_t;
%update errors for later use (to calculate covariance)
errors = err;

end

