steps = 1000;
x = [1; 0; 0; 0; 0; 0; 0.1];
x_hist = zeros(4, steps);

for i = 1:steps
    x_hist(:, i) = x(1:4);
    x = A_process(x, 1);
end

plot(x_hist(4, :))
    