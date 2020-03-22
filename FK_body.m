function [T] = FK_body(M,tab_body,config)
%FK_BODY takes in end effector configuration M and screw axes table
%tab_body and 
%   Detailed explanation goes here
T = M;
for i = 1:size(tab_body,1)
    w = tab_body(i,1:3);
    v = tab_body(i,4:6);
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
% figure(1);axis equal;hold on; xlim([0,10]);ylim([0,10]);zlim([0,10]);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),1,'r','lineWidth',2);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),1,'g','lineWidth',2);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),1,'b','lineWidth',2);