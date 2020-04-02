function [T] = FK_body(M,tab_b,config)
%FK_BODY takes in end effector configuration M and screw axes table
%tab_body and joints configurations and outputs the pose of the end
%effector
%   M:4-by-4 matrix
%   tab_body: n-by-6 matrix, n: # of joints, with row: [w1 w2 w3 v1 v2 v3]
%   config: 1-by-n vector([joint 1 angle, joint 2 angle,...joint n angle])
T = M;
for i = 1:size(tab_b,1)
    w = tab_b(i,1:3);
    v = tab_b(i,4:6);
    if w == zeros(1,3)
        R = eye(3);
        trans = config(i)*v;
    else
        R = cvt_rotvec2R(w,config(i));
        w_m = skewSymm(w);
        trans = (eye(3)*config(i)+(1-cos(config(i)))*w_m+(config(i)...
            -sin(config(i)))*w_m^2)*v';
    end
    exp = [R, trans;
        zeros(1,3), 1];
    T = T*exp;
end
%% uncomment code below for making plot
% figure(1);axis equal;hold on; grid on;
% xlim([-2,2]);ylim([-2,2]);zlim([-2,2]);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),1,'r','lineWidth',2);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),1,'g','lineWidth',2);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),1,'b','lineWidth',2);
% view(3);