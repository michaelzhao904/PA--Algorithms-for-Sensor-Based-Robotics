function R = cvt_rotvec2R(w,theta)
%CVT_ROTVEC2R takes in the axis-angle representation [w,theta] and convet
%it into equivalent rotation matrix R representation
%% check norm of w
if norm(w,2) ~= 1
    error('w norm is not 1');
end
%%
w_matrix = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
R = eye(3) + w_matrix*sin(theta) + w_matrix^2*(1-cos(theta));
end

