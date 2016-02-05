function q = rotvec2quat( rotvec )
%convert a differential rotation in vector form into a quaternion
rot_len = norm(rotvec);
rot_dir = rotvec/rot_len;
q = [cos(rot_len/2); rot_dir.*sin(rot_len/2)]; 
end

