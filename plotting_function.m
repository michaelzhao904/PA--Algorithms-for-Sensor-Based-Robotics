%%This code defines the robot and plots the graphs for different
%%configurations. This code assumes that we already have the data available
%%for plotting
%%Loading the data from .mat file
%Joint_angles = load('data.mat','joint_angles');
%Condition_numbers = load('data.mat','Condition_numbers');
%Isotropy = load('data.mat','Isotropy');
%frames = size(Joint_angles,1);
%joints = size(Joint_angles,2);
%%Plotting the graphs
subplot(1,2,1);
%Loading the rigid body tree diagram of the robot
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
Panda.DataFormat = 'row';

%Plotting the robot configuration framewise
config = homeConfiguration(Panda)
show(Panda)
for i = 1:frames
   show(Panda,Joint_angles(i,:),'PreservePlot',true);
   hold on;
end
width=550;    %set the window width
height=400;   %set the window height
title('Subplot 1: Robotic manipulation of Panda robot')

subplot(1,2,2);
%x = 1:frames;
%for j = 1:frames
%    y =  Condition_numbers(j)
%end
x = [5 4 3 2 1 0];
y = [0 1 2 3 4 5];
width = 300;
height = 150;
xlabel('time frames') 
ylabel('Condition numbers') 
plot(x,y,'g');
hold on;
title('Subplot 2: Condition Number')

