%% PA2 Problem a)--find the forward kinematics using space form
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
q4 = [0.088, 0, 0.316+0.333];
v4 = -cross(w4,q4);

w5 = [0, 0, 1];
q5 = [0, 0, 0.384+0.316+0.3330];
v5 = -cross(w5,q5);

w6 = [0, -1, 0];
q6 = [0, 0, 0.384+0.316+0.3330];
v6 = -cross(w6,q6);

w7 = [0, 0, -1];
q7 = [0.088, 0, 0.330+0.3160+0.384];
v7 = -cross(w7,q7);

%w8 = [0,0,-1];
%q8 = [0.088, 0, 0.330+0.3160+0.384-0.107];
%v8 = -cross(w8,q8);

tab_space = [w1, v1;
       w2, v2;
       w3, v3;
       w4, v4;
       w5, v5;
       w6, v6;
       w7, v7];
%%Testing the codes
%config = zeros(8,1); %set all joint angles to be zero
%config = [pi/2; 0; pi/2; 0; 0; 0; 0]
config = [pi/4; pi/4; pi/4; pi/4; 0; 0; 0]
T = FK_space(M,tab_space,config)
T1 = FK_body(M,tab_body,config)
transform = getTransform(Panda,[pi/4 pi/4 pi/4 pi/4 0 0 0 0],'base', 'body8')
J_space1 = J_space(tab_space,config)
jacobian = geometricJacobian(Panda,[pi/4 pi/4 pi/4 0 0 0 0 0],'body8')
   