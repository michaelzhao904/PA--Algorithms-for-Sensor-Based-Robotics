function [T] = FK_space(M,tab_space,config)
%FK_SPACE takes in end effector configuration M and screw axes table
%tab_space and 
%   M is the position and orientation of end effector frame in base frame
%   when all angles are set to zero. tab_space are the twists of the robot.
%config is a matrix which describes the angles of the robot
T = M; 
n = size(tab_space,1);
for j = 1:n
    i = n+1-j;
    w = tab_space(i,1:3);
    v = tab_space(i,4:6);
    if w == zeros(1,3)                      %if the joint is prismatic
        R = eye(3);
        trans = config(i)*v;
    else
        R = cvt_rotvec2R(w,config(i));      %if it is a revolute joint
        w_m = skewSymm(w);
        trans = (eye(3)*config(i)+(1-cos(config(i)))*w_m+(config(i)...
            -sin(config(i)))*w_m^2)*v';
    end
    exp = [R, trans;                  %forming the product of exponentials 
        zeros(1,3), 1];
    T = exp*T;
end
figure(1);axis equal;hold on; grid on;%plotting the end-effector frame
xlim([-2,2]);ylim([-2,2]);zlim([-2,2]);
quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),1,'r','lineWidth',2);
quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),1,'g','lineWidth',2);
quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),1,'b','lineWidth',2);
view(3);
end                                    