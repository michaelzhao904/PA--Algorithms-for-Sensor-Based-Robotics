function [U,S] = ellipsoidData(J_body,type)
%ELLIPSOID Summary of this function goes here
%   Detailed explanation goes here
n = 3;
switch type
    case 'w'
        J = J_body(1:3,:);
    case 'v'
        J = J_body(4:6,:);
    case 'all'
        J = J_body;
        n = 6;
end
A = J*J';
[U,S,V] = svd(A);
U_3min = [U(:,n-2),U(:,n-1),U(:,n)];
S = diag(S);
end

