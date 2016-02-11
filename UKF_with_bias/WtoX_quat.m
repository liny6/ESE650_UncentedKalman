function X_sigma = WtoX_quat( W_sigma, x_last )

[~, nc] = size(W_sigma);
X_sigma = zeros(7, nc);
q = x_last(1:4);
w = x_last(5:7);
calib = x_last(8:10);

for i = 1:nc
    wqi = W_sigma(1:3, i);
    q_wi = rotvec2quat(wqi);
    X_sigma(1:4, i) = quatmult(q, q_wi);
end

X_sigma(5:10, :) = bsxfun(@plus, W_sigma(4:9, :), [w; calib]);
end

