function [omega_t, OmegaGT] = getOmegaGT(Vicon)
%GETOMEGAGT Summary of this function goes here
%   Detailed explanation goes here
nt = length(Vicon.ts);
omega = zeros(3, nt-1);
omega_t = Vicon.ts(1:end-1);
[b, a] = butter(6, 0.1); % butterworth filter coefficients

for i = 1:nt-1
    del_R = Vicon.rots(:, :, i)' * Vicon.rots(:, :, i+1);
    del_v = vrrotmat2vec(del_R);
    del_t = Vicon.ts(i+1) - Vicon.ts(i);
    
    if del_t > 0.005
        omega(:, i) = Vicon.rots(:, :, i)'*vrrotvec2mat([del_v(1:3), del_v(4)/2])*del_v(1:3)'*del_v(4)/del_t;
    else
        omega(:, i) = omega(:, i-1); % if time stamp is too short, just assume previous step's omega in order to keep the estimate stable
    end
    
end

OmegaGT = filter(b, a, omega, [], 2);
%OmegaGT = omega;

end

