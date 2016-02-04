nt = length(Vicon.ts);
omega = zeros(3, nt-1);
omega_t = 0.5* (Vicon.ts(2:end) + Vicon.ts(1:end-1));
[b, a] = butter(6, 0.1); % butterworth filter coefficients

for i = 1:nt-1
    del_R = Vicon.rots(:, :, i)' * Vicon.rots(:, :, i+1);
    del_v = vrrotmat2vec(del_R);
    omega(:, i) = Vicon.rots(:, :, i)'*del_v(1:3)'*del_v(4)/.01; % assume sampling time of 0.01s
end

omega = filter(b, a, omega, [], 2);