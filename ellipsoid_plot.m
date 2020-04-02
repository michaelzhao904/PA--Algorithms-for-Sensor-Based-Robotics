function [U,S] = ellipsoid_plot(J_body,type)
%ELLIPSOID_PLOT plots the ellipsoid of manipulability
%   J_body is a 6-by-n Jacobian Matrix
%   type: 'w','v','all' allow user to pick from J_w, J_v(both 3-by-n) and
%   J(6-by-n) to plot the ellipsoid. In the 'all' case, three smallest
%   singular values will be pick and the vectors will be compressed to
%   3-dimensional and make the plot
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
R = U_3min(any(U_3min,2),:);
%%
[x,y,z] = ellipsoid(0,0,0,S(n-2,n-2),S(n-1,n-1),S(n,n));
% [x,y,z] = ellipsoid(0,0,0,1,1,1);
% surf(x,y,z);

for i = 1:size(x,1)
    for j = 1:size(x,2)
        v = R*[x(i,j),y(i,j),z(i,j)]';
        x(i,j) = v(1);
        y(i,j) = v(2);
        z(i,j) = v(3);
    end
end
surf(x,y,z);
% axis equal;

end

