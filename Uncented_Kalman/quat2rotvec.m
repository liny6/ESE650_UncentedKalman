function [ rotvec ] = quat2rotvec( q )
%QUAT2ROTVEC Summary of this function goes here
%   Detailed explanation goes here
sin_half_theta = norm(q(2:4));

if sin_half_theta == 0
    rotvec = zeros(3, 1);
else
    cos_half_theta = q(1);
    half_theta = atan2(sin_half_theta, cos_half_theta);
    vec_dir = q(2:4)/sin_half_theta;
    rotvec = vec_dir*half_theta*2;
end

