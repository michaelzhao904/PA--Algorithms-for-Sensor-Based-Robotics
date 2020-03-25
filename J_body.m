function [J_body] = J_body(tab_body,config)
%J_BODY computes the Jacobian matrix for open chain robot defined by screw
%axis table tab_body; config defines the set of joint angles. The output is
%the Jacobian matrix
%   tab_body: n-by-6 matrix, n: # of joints, with row: [w1 w2 w3 v1 v2 v3]
%   config: 1-by-n vector([joint 1 angle, joint 2 angle,...joint n angle])
n = size(tab_body,1);
J_body = zeros(6, n);

for i=1:n
    B_i = tab_body(i,:)';
    T = eye(4);
    if i < n
        for j = (i+1):n
            w = tab_body(j,1:3);
            v = tab_body(j,4:6);
            if w == zeros(1,3)
                R = eye(3);
                trans = -config(j)*v;
            else
                R = cvt_rotvec2R(w,-config(j));
                w_m = skewSymm(w);
                trans = (eye(3)*-config(j)+(1-cos(-config(j)))*w_m+(-config(j)...
                    -sin(-config(j)))*w_m^2)*v';
            end
            exp = [R, trans;
                zeros(1,3), 1];
            T = exp*T;
        end
    else
        T = eye(4);
    end
    J_body(:,i) = Ad_T(T)*B_i;
end