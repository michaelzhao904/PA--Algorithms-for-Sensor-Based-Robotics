%% Homework2 Problem 4 (5.13)
syms theta1 theta2 theta3 theta4 theta5 theta6 L real
ws1 = [0, 0, 1]';
qs1 = [0, 0, 0]';
vs1 = -cross(ws1,qs1);

ws2 = cvt_rotvec2R([0,0,1],theta1)*[0, 1, 0]';
qs2 = [0, 0, 0]';
vs2 = -cross(ws2,qs2);

ws3 = cvt_rotvec2R([0,0,1],theta1)*cvt_rotvec2R([0,1,0],theta2)*[-1, 0, 0]';
qs3 = [0, 0, 0]';
vs3 = -cross(ws3,qs3);

ws4 = ws3;
qs4 = cvt_rotvec2R([0,0,1],theta1)*cvt_rotvec2R([0,1,0],theta2)*...
    cvt_rotvec2R([-1,0,0],theta3)*[0, L, 0]';
vs4 = -cross(ws4,qs4);

ws5 = ws3;
qs5 = cvt_rotvec2R([0,0,1],theta1)*cvt_rotvec2R([0,1,0],theta2)*...
    cvt_rotvec2R([-1,0,0],theta3)*[0, L*(1+cos(theta4)),-sin(theta4)*L]';
vs5 = -cross(ws5,qs5);

ws6 = cvt_rotvec2R([0,0,1],theta1)*cvt_rotvec2R([0,1,0],theta2)*...
    cvt_rotvec2R([-1,0,0],theta3)*cvt_rotvec2R([-1,0,0],theta4)*...
    cvt_rotvec2R([-1,0,0],theta5)*cvt_rotvec2R([0,1,0],theta6)*[0, 1, 0]';
qs6 = cvt_rotvec2R([0,0,1],theta1)*cvt_rotvec2R([0,1,0],theta2)*...
    cvt_rotvec2R([-1,0,0],theta3)*[0, L*(1+cos(theta4)+cos(theta4+theta5)),...
    -L*(sin(theta4)+sin(theta4+theta5))]';
vs6 = -cross(ws6,qs6);

Js = [ws1 ws2 ws3 ws4 ws5 ws6;
      vs1 vs2 vs3 vs4 vs5 vs6];
f(theta1,theta2,theta3,theta4,theta5,theta6)=Js;
f_full(theta1,theta2,theta3,theta4,theta5,theta6,L)=Js;
J  = f_full(0,pi/2,0,pi,pi,0,1); % Assume L = 1
J  = f_full(0,0,-pi/2,0,0,0,0); % Assume L = 1
J = double(J)
% [v,d] = eig(J)