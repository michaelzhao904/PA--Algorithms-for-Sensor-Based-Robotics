%% This code defines and simulates the robot

dhparams = [0 0 0.333 0;
            0 -pi/2 0 0;
            0 pi/2 0.316 0;
            0.0825 pi/2 0 0;
            -0.0825 -pi/2 0.384 0;
            0 pi/2 0 0;
            0.088 pi/2 0 0;
            0 0 0.107 0]; %mdh parameter for Panda robot

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

% showdetails(Panda)

% show(Panda);
% axis off

Panda.DataFormat = 'row';

% T_tb = getTransform(Panda,[pi/4,pi/4,pi/4,0,0,0,0,0],'body8');

% geoJacob = geometricJacobian(Panda,homeConfiguration(Panda),'body8');
% show(Panda,homeConfiguration(Panda));hold on;axis off

%% Predefined configurations
run('PA_a.m'); % obtain M and tab_space
run('PA_ca.m'); % obtain M and tab_b
homeConfig = zeros(8,1); % robot home configuration
config = [0,0,0,0,0,0,0,0]'; % test config
config_2 = [0,pi/2,pi/4,pi/4,pi/4,pi/4,0,0]';
config = config_2;
%% Tests for FK_space, FK_body

% test for FK
FK_s = FK_space(M,tab_s,config);
FK_b = FK_body(M,tab_b,config);
FK_m = getTransform(Panda,config','body8');

% % test for Jacobian
% J_b = J_body(tab_b,config);
% % J_s = J_space(tab_space,config);
% J_s2b = Ad_T(FK_b)*J_b;

%% Tests for J_space, J_body

J_s = J_space(tab_s,config)
J_b = J_body(tab_b,config);
T_sb = FK_body(M,tab_b,config);
J_b2s = Ad_T(T_sb)*J_b
%% Test for J_inverse_kinematics.m

T_sd = FK_body(M,tab_b,config_2); %set desired configuration
delta = [1e-3,1e-3]; % orientation and position error tolerance
dt = 1e-4;
% solve for inverse kinematic motion for different methos
inverse_solution = J_inverse_kinematics(T_sd,M,tab_b,homeConfig,delta,dt);
% transpose_solution = J_transpose_kinematics(T_sd,M,tab_b,homeConfig,eye(6),delta,dt);
% redundancy_solution = redundancy_resolution(T_sd, M, tab_b,...
%     homeConfig, delta, 0.1, dt);

% end pose with different methods
T_end_inverse = FK_body(M,tab_b,inverse_solution(end,:));
% T_end_transpose = FK_body(M,tab_b,transpose_solution(end,:));
% T_end_redundancy = FK_body(M,tab_b,redundancy_solution(end,:));

%% Simulation
visualRobot(Panda,T_sd,inverse_solution,100);
visualRobot(Panda,T_sd,transpose_solution,100);
visualRobot(Panda,T_sd,redundancy_solution,100);
