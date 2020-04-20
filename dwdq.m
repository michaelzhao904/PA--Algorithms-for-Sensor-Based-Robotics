function dwdq = dwdq(J_body,tab_body,config)
%DWDQ This function is part of the redundancy_resolution.m function
%   dwdq computes the manipulability w(q) and then computes it's derivative
%   wrt to q. J_body: Jacobian function; tab_body, config: parameters and
%   configuration for robot.

dwdq = zeros(size(config));

delta = 1e-4;

for i = 1:size(dwdq,1)
    v_delta = zeros(size(config));
    v_delta(i) = delta;
    q_plus = config + v_delta;
    q_minus = config - v_delta;
    J_plus = J_body(tab_body,q_plus);
    J_minus = J_body(tab_body,q_minus);
    dwdq(i) = (w(J_plus)-w(J_minus))/(2*delta);
end
end

function manipuMeasure = w(J)
manipuMeasure = sqrt(det(J*J')); % manipulability measure
end
