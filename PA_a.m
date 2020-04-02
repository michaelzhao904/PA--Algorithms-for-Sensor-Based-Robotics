%% PA2 Problem b)--find the forward kinematics using space form...
%% This part finds the forward kinematics of the robot using the space form
R = [1, 0, 0;
     0, -1, 0;
     0, 0, -1];
p = [0.088, 0, 0.333+0.316+0.384-0.107]';
M = [R, p;
    zeros(1,3), 1];

w1 = [0, 0, 1];
q1 = [0, 0, 0.3330];
v1 = -cross(w1,q1);

w2 = [0, 1, 0];   
q2 = [0, 0, 0.3330];
v2 = -cross(w2,q2);

w3 = [0, 0, 1];
q3 = [0, 0, 0.316+0.333];
v3 = -cross(w3,q3);

w4 = [0, -1, 0];
q4 = [0.0825, 0, 0.316+0.333];
v4 = -cross(w4,q4);

w5 = [0, 0, 1];
q5 = [0, 0, 0.384+0.316+0.3330];
v5 = -cross(w5,q5);

w6 = [0, -1, 0];
q6 = [0, 0, 0.384+0.316+0.3330];
v6 = -cross(w6,q6);

w7 = [0, 0, -1];
q7 = [0.088, 0, 0.3330+0.3160+0.384];
v7 = -cross(w7,q7);

w8 = [0,0,-1];
q8 = [0.088, 0, 0.3330+0.3160+0.384-0.107];
v8 = -cross(w8,q8);

tab_s = [w1, v1;
       w2, v2;
       w3, v3;
       w4, v4;
       w5, v5;
       w6, v6;
       w7, v7;
       w8, v8];

