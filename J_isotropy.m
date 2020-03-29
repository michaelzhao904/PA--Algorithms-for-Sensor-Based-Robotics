function mu1 = J_isotropy(J_body,type)
%J_ISOTROPY Summary of this function goes here
%   Detailed explanation goes here
switch type
    case 'w'
        J = J_body(1:3,:);
    case 'v'
        J = J_body(4:6,:);
    case 'all'
        J = J_body;
end
A = J*J';
[U,S,V] = svd(A);
mu1 = S(1,1)/S(end,end);
end

