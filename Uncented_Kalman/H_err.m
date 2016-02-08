function Z_err = H_err(W_sigma, Y_sigma, x_a_priori)

gravity = [0; 0; 1];% expected gravity when it's at orientation [1; 0; 0; 0]
[~, nc] = size(W_sigma);
Z_err = zeros(6, nc);

for i = 1:nc
    q = rotvec2quat(W_sigma(1:3, i));
    Z_err(1:3, i) = quatrot(q, gravity);
end

Z_err(4:6, :) = bsxfun(@minus, Y_sigma(4:6, :), x_a_priori(5:7));

end

