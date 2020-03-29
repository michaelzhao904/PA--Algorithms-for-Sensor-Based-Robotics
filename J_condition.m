function mu2 = J_condition(J_body,type)
%J_CONDITION Summary of this function goes here
%   Detailed explanation goes here
mu1 = J_isotropy(J_body,type);
mu2 = mu1^2;
end

