function mu3 = J_ellipsoid_volume(J_body,type)
%J_ELLIPSOID_VOLUME computes the ellipsoid volume of a Jacobian matrix
%   J_body is a 6-by-n Jacobian Matrix
%   type: 'w','v','all' allow user to pick from J_w, J_v(both 3-by-n) and
%   J(6-by-n) to compute the ellipsoid volume
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
mu3 = det(S);
end

