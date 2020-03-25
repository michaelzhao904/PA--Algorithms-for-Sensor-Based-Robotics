function [a,b,c] = cvt_R2ZYZ(R)
%CVT_R2EULER takes in the rotation matrix R and output representation in
%ZYZ Euler angles(a,b,c)
%% check dimension
if ~isequal(size(R),[3,3])
    error('R dimension is not (3,3)');
end
%%
if R(3,3) == 1
    b = 0;
    a = atan2(R(2,1), R(2,2));
    c = 0;
elseif R(3,3) == -1
    b = pi;
    a = atan2(-R(2,1),R(2,2));
    c = 0;
else
    b = atan2(sqrt(R(1,3)^2+R(2,3)^2),R(3,3));
    a = atan2(R(2,3),R(1,3));
    c = atan2(R(3,2),-R(3,1));
end
end