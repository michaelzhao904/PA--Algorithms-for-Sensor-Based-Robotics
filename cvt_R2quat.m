function q = cvt_R2quat(R)
%CVT_R2QUAT takes in the rotation matrix R and output representation in
%quaternion representation
%% check dimension
if ~isequal(size(R),[3,3])
    error('R dimension is not (3,3)');
end
%% 
q0 = sqrt(1+trace(R))/2;
q = 1/(4*q0)*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
q = [q0; q];
end