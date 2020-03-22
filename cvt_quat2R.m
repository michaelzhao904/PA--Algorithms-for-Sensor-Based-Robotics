function R = cvt_quat2R(q)
%CVT_QUAT2R takes in the quaternion representation q and convet
%it into equivalent rotation matrix R representation

q_norm = norm(q,2);
q = q/q_norm;
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

R = [q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q0*q2+q1*q3);
    2*(q0*q3+q1*q2) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2) 2*(q0*q1+q2*q3) q0^2-q1^2-q2^2+q3^2];
end

