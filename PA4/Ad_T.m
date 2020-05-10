function adj_t = Ad_T(T)
%AD_T Compute the adjoint representation of transformation matrix T
%   Detailed explanation goes here
R = T(1:3,1:3);
p = T(1:3,4);
adj_t = [R, zeros(3,3);
         skewSymm(p)*R, R];
end

