function Acc_GT = getAccGT(Vicon)
%GETGROUNDGRAVITY Summary of this function goes here
%   Detailed explanation goes here

G = [0; 0; 9.8];
[~, ~, nt] = size(Vicon.rots);
Acc_GT = zeros(3, nt);

for i = 1:nt
    Acc_GT(:, i) = Vicon.rots(:, :, i)'*G;
end

end

