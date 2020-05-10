%% PA4 problems solution

%% Define robot
dhparams = [0 0 0.333 0;
            0 -pi/2 0 0;
            0 pi/2 0.316 0;
            0.0825 pi/2 0 0;
            -0.0825 -pi/2 0.384 0;
            0 pi/2 0 0;
            0.088 pi/2 0 0;
            0 0 0.207 0]; %mdh parameter for Panda robot

Panda = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'mdh');
body1.Joint = jnt1;
addBody(Panda,body1,'base');

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
body8 = robotics.RigidBody('body8');
jnt8 = robotics.Joint('jnt8','revolute');

setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');
setFixedTransform(jnt7,dhparams(7,:),'mdh');
setFixedTransform(jnt8,dhparams(8,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
body8.Joint = jnt8;

addBody(Panda,body2,'body1')
addBody(Panda,body3,'body2')
addBody(Panda,body4,'body3')
addBody(Panda,body5,'body4')
addBody(Panda,body6,'body5')
addBody(Panda,body7,'body6')
addBody(Panda,body8,'body7')

showdetails(Panda)

show(Panda);
% axis off
Panda.DataFormat = 'row';

T = getTransform(Panda,zeros(1,8),'body8','base');

%
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
q8 = [0.088, 0, 0.3330+0.3160+0.384-0.207];
v8 = -cross(w8,q8);

tab_s = [w1, v1;
       w2, v2;
       w3, v3;
       w4, v4;
       w5, v5;
       w6, v6;
       w7, v7;
       w8, v8];
   
%% configuration settings
p_goal = [0.2036, 0.5487, 0.7682]';
% FK_space(M, tab_s, config_init);
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 3.8973, 0];
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0];
config_init = [0, 0, 0, -0.0698, 0, 0, 0, 0];
%% (a)

%% (b)
w1 = 1;
w2 = 1;
err = 1e10; % initailize error
jointsData = config_init;
i = 1;
while true
    T = FK_space(M, tab_s, jointsData(i,:));
    R = T(1:3,1:3);
    t = T(1:3,4);
    err = norm(t-p_goal,2);
    if err < 1e-4
        break
    end
    J = J_space(tab_s, jointsData(i,:));
    J_alpha = J(1:3,:);
    J_epsilon = J(4:6,:);
    
    C1 = sqrt(w1)*(-skewSymm(t)*J_alpha + J_epsilon);
    d1 = p_goal - t;
    C2 = sqrt(w2)*(-skewSymm(R*[0, 0, 0]')*J_alpha);
    d2 = zeros(3,1);
    
    C = [C1; C2];
    d = [d1; d2];
    
    dq = lsqlin(C, d, [], [], [], [], (q_min-jointsData(i,:))'/100,...
        (q_max-jointsData(i,:))'/100);
    jointsData(i+1,:) = jointsData(i,:) + dq';
    i=i+1;
end
% T = FK_space(M,tab_s,jointsData(1,:))