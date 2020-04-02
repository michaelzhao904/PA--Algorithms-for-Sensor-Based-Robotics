function mu2 = J_condition(J_body,type)
%J_CONDITION computes the condition number of a Jacobian matrix.
%   J_body is a 6-by-n Jacobian Matrix
%   type: 'w','v','all' allow user to pick from J_w, J_v(both 3-by-n) and
%   J(6-by-n) to compute
mu1 = J_isotropy(J_body,type);
mu2 = mu1^2;
end

