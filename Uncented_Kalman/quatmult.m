function q_result = quatmult( q1, q2 )
%calculates the rotation q1 followed by q2
qr = q1(1);
qi = q1(2);
qj = q1(3);
qk = q1(4);

q1mat = [qr, -qi, -qj, -qk
        qi, qr -qk, qj;
        qj, qk, qr, -qi;
        qk, -qj, qi, qr];
q_result = q1mat*q2;
end

