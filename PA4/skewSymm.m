function y = skewSymm(x)
%SKEWSYMM converts a R^3 vector to a 3x3 skew_symmetric matrix
y = [0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end

