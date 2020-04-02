function mu1 = J_isotropy(J_body,type)
%J_ISOTROPY computes the isotropy of a Jacobian matrix.
%   J_body is a 6-by-n Jacobian Matrix
%   type: 'w','v','all' allow user to pick from J_w, J_v(both 3-by-n) and
%   J(6-by-n) to compute the isotropy
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

