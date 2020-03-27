function vec = ellipsoid_plot(J_body,type)
%ELLIPSOID_PLOT Summary of this function goes here
%   Detailed explanation goes here
switch type
    case 'w'
        J = J_body(1:3,:)
    case 'v'
        J = J_body(4:6,:);
end
A = J*J';
[U,S,V] = svd(A);
[x,y,z] = ellipsoid(0,0,0,S(1,1),S(2,2),S(3,3));
% [x,y,z] = ellipsoid(0,0,0,1,1,1);
% surf(x,y,z);

for i = 1:size(x,1)
    for j = 1:size(x,2)
        v = U*[x(i,j),y(i,j),z(i,j)]';
        x(i,j) = v(1);
        y(i,j) = v(2);
        z(i,j) = v(3);
    end
end
surf(x,y,z);
% axis equal;
vec = S;
end

