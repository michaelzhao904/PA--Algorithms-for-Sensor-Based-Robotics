%% This code defines and simulates the robot and nests tests for functions
%% Define robot using Robotics System Toolbox
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

%% Predefined configurations
run('PA_a.m'); % obtain M and tab_s(M :home configuration pose, tab_s: screw axis info for space frame)
run('PA_ca.m'); % obtain M and tab_b(M :home configuration pose, tab_b: screw axis info for space frame)
homeConfig = zeros(8,1); % robot home configuration
% config = [0,0,0,0,0,0,0,0]'; % test config
config = [0,pi/2,pi/4,pi/4,pi/4,pi/4,pi/2,0]'; % test config for report
config = config; % user can change the config value for different tests

%% Tests for FK_space, FK_body function
% test for FK
FK_s = FK_space(M,tab_s,config); % Forward kinematics with space frame
FK_b = FK_body(M,tab_b,config); % Forward kinematics with body frame
FK_m = getTransform(Panda,config','body8'); % Forward kinematics with MATLAB built-in function
% The above three values should be the same.

%% Tests for J_space, J_body function

J_s = J_space(tab_s,config); % Space Jacobian at test config
J_b = J_body(tab_b,config); % Boby Jacobian at test config
T_sb = FK_body(M,tab_b,config); % pose of end effector
J_b2s = Ad_T(T_sb)*J_b; % Convert body Jacobian to space Jacobian
% J_b2s should be the same with J_s
%% Tests for ellipsoid_plot, J_isotropy, J_condition, J_ellipsoid_volume
% 'w' 'v' 'all' specify which kind of Jacobian used.
% 'w': rotational Jacobian  'v':velocity Jacobian  'all':whole Jacobian
figure;
ellipsoid_plot(J_b,'w');
figure;
ellipsoid_plot(J_b,'v');
J_isotropy_w = J_isotropy(J_b,'w');
J_isotropy_v = J_isotropy(J_b,'v');
J_isotropy_all = J_isotropy(J_b,'all');
J_condition_w = J_condition(J_b,'w');
J_condition_v = J_condition(J_b,'v');
J_condition_all = J_condition(J_b,'all');
J_ellipsoid_volume_w = J_ellipsoid_volume(J_b,'w');
J_ellipsoid_volume_v = J_ellipsoid_volume(J_b,'v');
J_ellipsoid_volume_all = J_ellipsoid_volume(J_b,'all');

%% Test for J_inverse_kinematics.m

T_sd = FK_body(M,tab_b,config); %set desired configuration
delta = [1e-3,1e-3]; % orientation and position error tolerance
dt = 1e-4; % specify time step
% solve for inverse kinematic motion for different methos
% get inverse kinematics solution data
inverse_solution = J_inverse_kinematics(T_sd,M,tab_b,homeConfig,delta,dt);
% get transpose kinematics solution data
transpose_solution = J_transpose_kinematics(T_sd,M,tab_b,homeConfig,eye(6),delta,dt);
% get redundancy resolution solution data
redundancy_solution = redundancy_resolution(T_sd, M, tab_b,...
    homeConfig, delta, 0.1, dt);

% end pose with different methods
T_end_inverse = FK_body(M,tab_b,inverse_solution(end,:));
T_end_transpose = FK_body(M,tab_b,transpose_solution(end,:));
T_end_redundancy = FK_body(M,tab_b,redundancy_solution(end,:));
% T_end_inverse, T_end_transpose, T_end_redundancy should be close to T_sd
%% Simulation
% simulate each cases
frames = 100; %specify simultaion frames number
visualRobot(Panda,T_sd,inverse_solution,frames);
visualRobot(Panda,T_sd,transpose_solution,frames);
visualRobot(Panda,T_sd,redundancy_solution,frames);
