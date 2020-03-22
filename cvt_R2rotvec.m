function [w,theta] = cvt_R2rotvec(R)
%ROTATIONCVT takes in the rotation matrix R and output representation in
%equivalent axis-angle representation [w,theta]

%% check dimension
if ~isequal(size(R),[3,3])
    error('R dimension is not (3,3)');
end
%% 
if isequal(R,eye(3))
    theta = 0;
    w = nan;
elseif trace(R) == -1
    theta = pi;
    if 1+R(3,3) ~= 0
        w = (1/sqrt(2*(1+R(3,3))))*[R(1,3);R(2,3);1+R(3,3)];
    elseif 1+R(2,2) ~= 0
        w = (1/sqrt(2*(1+R(2,2))))*[R(1,2);1+R(2,2);R(3,2)];
    else
        w = (1/sqrt(2*(1+R(1,1))))*[1+R(1,1);1+R(2,1);R(3,1)];
    end
else
   theta = acos((trace(R)-1)/2);
   w_matrix = (R-R')/(2*sin(theta));
   w = [w_matrix(3,2); w_matrix(1,3); w_matrix(2,1)];
end

end

