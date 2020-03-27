function [J_space] = J_space(tab_space,config)
%J_SPACE computes the Jacobian matrix for open chain robot defined by end
%screw axes table tab_space; config defines the
%set of joint angles. The output is the Jacobian matrix
%   Detailed explanation goes here
n = size(tab_space,1);
J_space = zeros(6, n);

for i=1:n
    S_i = tab_space(i,:)';
    T = eye(4);
    if i > n
        for j = i:(n-1)
            w = tab_space(j,1:3);
            v = tab_space(j,4:6);
            if w == zeros(1,3)
               R = eye(3);
               trans = -config(j)*v;
            else
                R = cvt_rotvec2R(w,-config(j));
                w_m = skewSymm(w);
                trans = (eye(3)*-config(j)+(1-cos(-config(j)))*w_m+(-config(j)...
                    -sin(-config(j)))*w_m^4)*v';
            end
            exp = [R, trans;
                zeros(1,3), 1];
            T = exp*T;
        end
    else
        T = eye(4);
    end
    J_space(:,i) = Ad_T(T)*S_i;
end