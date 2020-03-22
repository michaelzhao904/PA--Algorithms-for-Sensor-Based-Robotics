function [a,b,c] = cvt_R2ZYX(R)
%CVT_R2ZYX takes in the rotation matrix R and output representation in
%ZYX Euler angles(a,b,c) (roll:a; pitch:b, yall:c)
%% check dimension
if ~isequal(size(R),[3,3])
    error('R dimension is not (3,3)');
end
%%
a = atan2(R(2,1),R(1,1));
b = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
c = atan2(R(3,2),R(3,3));
end