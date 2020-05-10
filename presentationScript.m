%% Script for presentation
% Note
% dt selection for inverse, large dt oscilation, small dt slow but good convergence
% K matrix selection for transpose
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

%% Import inbuilt robot
% Panda = loadrobot("frankaEmikaPanda");
% Panda.DataFormat = 'row';
% show(Panda,zeros(1,9),'PreservePlot',false);
% T = getTransform(Panda,zeros(1,9),'panda_link8')
%% Define parameters and configurations for robot
run('PA_ca.m');
homeConfig = zeros(8,1);
config = [0,pi/2,pi/4,pi/4,pi/4,pi/4,pi/2,0]';
% config = ones(8,1);

% configB = [pi/2,pi/2,pi/4,pi/4,pi/4,pi/4,pi/2,0]';

%% Simulate

T_sd = FK_body(M,tab_b,config);
% T_sd = [eye(3) [0.3;0.3;0.3];
%         zeros(1,3) 1];
% T_sdB = FK_body(M,tab_b,configB);
delta = [1e-2, 1e-2];
dt = .01;

%% inverse solution
inverse_solution = J_inverse_kinematics(T_sd,M,tab_b,homeConfig,delta,dt);
% inverse_solution_B = J_inverse_kinematics(T_sdB,M,tab_b,...
%     inverse_solution_A(end,:)',delta,dt);
T_end_inverse = FK_body(M,tab_b,inverse_solution(end,:));
% J_inverse_solution_A = 

%% transpose solution
K = eye(6);
K(4:6,4:6) = 10*eye(3);
transpose_solution = J_transpose_kinematics(T_sd,M,tab_b,homeConfig,K,delta,dt);
% transpose_solution_B = J_transpose_kinematics(T_sdB,M,tab_b,...
%     transpose_solution_A(end,:)',eye(6),delta,dt);
T_end_transpose = FK_body(M,tab_b,transpose_solution(end,:));

%% redundancy solution
redundancy_solution = redundancy_resolution(T_sd, M, tab_b,...
    homeConfig, delta, 10000, dt);
% redundancy_solution_B = redundancy_resolution(T_sdB, M, tab_b,...
%     redundancy_solution_A(end,:), delta, 0.1, dt);
T_end_redundancy = FK_body(M,tab_b,redundancy_solution(end,:));

%% Generate Data for Simulation
num = 1;
switch num
    case 1
        data = inverse_solution;
        name = 'Inverse Kinematics Solution';
    case 2
        data = transpose_solution;
        name = 'Jacobian Transpose Solution';
    case 3
        data = redundancy_solution;
        name = 'Redundancy Resolution Solution';
end
% data = inverse_solution;
n = size(data,1);
frames = 101;
seq = 1:round(n/frames):n;
iso_w = zeros(size(seq));
iso_v = zeros(size(seq));
cond_w = zeros(size(seq));
cond_v = zeros(size(seq));
manip_v = zeros(size(seq));
U_w = zeros(3,3,length(seq));
U_v = zeros(size(U_w));
S_w = zeros(3,length(seq));
S_v = zeros(3,length(seq));
ellip_x = zeros(21,21,length(seq));
ellip_y = zeros(21,21,length(seq));
ellip_z = zeros(21,21,length(seq));
T = zeros(4,4,length(seq));
robotConfig = zeros(length(seq),size(data,2));
j = 1;

for i = seq

%     plot3(T_sd(1,4),T_sd(2,4),T_sd(3,4),'r.','MarkerSize',20);
    T(:,:,j) = getTransform(Panda,data(i,:),'body8','base');
    robotConfig(j,:) = data(i,:);

    % draw robot

    % plot isotropy and condition number
    J = J_body(tab_b,data(i,:));
    iso_w(j) = J_isotropy(J,'w');
    iso_v(j) = J_isotropy(J,'v');
    cond_w(j) = J_condition(J,'w');
    cond_v(j) = J_condition(J,'v');
    manip_v(j) = J_ellipsoid_volume(J,'all');
    
%     dwq = dwdq(@J_body,tab_b,data(i,:)')
%     pause(0.01);
%     plot(1:j,cond_w(1:j),'k');
%     plot(1:j,cond_v(1:j),'m');
%     legend('iso_w','iso_v')

    % ellipsoid data
    [U_w(:,:,j),S_w(:,j)] = ellipsoidData(J,'w');
    [U_v(:,:,j),S_v(:,j)] = ellipsoidData(J,'v');
    scaleFac = 0.5;
        [xv_temp,yv_temp,zv_temp] = ellipsoid(0,0,0,S_v(1,j)*scaleFac,...
            S_v(2,j)*scaleFac,S_v(3,j)*scaleFac);
    xv_temp = reshape(xv_temp,[1,21*21]);
    yv_temp = reshape(yv_temp,[1,21*21]);
    zv_temp = reshape(zv_temp,[1,21*21]);
    xyz_temp = [xv_temp;yv_temp;zv_temp];
    R2 = T(1:3,1:3,j);
    R1 = U_v(:,:,j);
    p = T(1:3,4,j);
    xyz_trans = R2*R1*xyz_temp + p;
    xv_revover = reshape(xyz_trans(1,:),[21,21]);
    yv_revover = reshape(xyz_trans(2,:),[21,21]);
    zv_revover = reshape(xyz_trans(3,:),[21,21]);
    
    ellip_x(:,:,j) = xv_revover;
    ellip_y(:,:,j) = yv_revover;
    ellip_z(:,:,j) = zv_revover;
        
%     % plot
%     subplot(1,2,1)
%     show(Panda,data(i,:),'PreservePlot',false);
%     hold on;
%     xlim([-1,1]);ylim([-1,1]);zlim([0,1.5]);
%     quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,1),T_sd(2,1),T_sd(3,1),.1,'r','lineWidth',2);
%     quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,2),T_sd(2,2),T_sd(3,2),.1,'g','lineWidth',2);
%     quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,3),T_sd(2,3),T_sd(3,3),.1,'b','lineWidth',2);
%     h = surf(ellip_x(:,:,j),ellip_y(:,:,j),ellip_z(:,:,j),...
%         'FaceAlpha',0.2,'edgecolor','none');
%     
%     plot3(T(1,4,j),T(2,4,j),T(3,4,j),'r.','MarkerSize',20);
%     pause(0.1);
%     
%     subplot(1,2,2);
%     hold on;
%     plot(1:j,iso_w(1:j),'b','lineWidth',2);
%     plot(1:j,iso_v(1:j),'r','lineWidth',2);
% 	delete(h);

    % next iteration
    j = j+1;

end

figure('DefaultAxesFontSize',12);

set(gcf, 'WindowState', 'maximized');
subplot(2,3,3);
title('Manipulability Volume');
subplot(2,3,6);
title('Isotropy');

for ii = 1:length(seq)
        % plot
    subplot(2,3,[1 2 4 5]);
    
    show(Panda,robotConfig(ii,:),'PreservePlot',false);
    hold on;
    xlim([-.8,.8]);ylim([-.8,.8]);zlim([0,1.2]);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,1),T_sd(2,1),T_sd(3,1),.1,'r','lineWidth',2);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,2),T_sd(2,2),T_sd(3,2),.1,'g','lineWidth',2);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,3),T_sd(2,3),T_sd(3,3),.1,'b','lineWidth',2);
    h = surf(ellip_x(:,:,ii),ellip_y(:,:,ii),ellip_z(:,:,ii),...
        'FaceAlpha',0.2,'edgecolor','none');
    
    plot3(T(1,4,ii),T(2,4,ii),T(3,4,ii),'r.','MarkerSize',20);
    pause(0.1);
    
    subplot(2,3,3);
    hold on;
    plot(1:ii,manip_v(1:ii),'k','lineWidth',2);
	delete(h);
    
    subplot(2,3,6);
    hold on;
    plot(1:ii,iso_w(1:ii),'b','lineWidth',2);
    plot(1:ii,iso_v(1:ii),'r','lineWidth',2);
    

end
        legend('isotropy w','isotropy v');
        xlabel('iterations');

% Save data for plots
switch num
    case 1
        data = inverse_solution;
        name = 'Inverse Kinematics Solution';
        iso_w_inverse = iso_w;
        iso_v_inverse = iso_v;
        manip_v_inverse = manip_v;
    case 2
        data = transpose_solution;
        name = 'Jacobian Transpose Solution';
        iso_w_transpose = iso_w;
        iso_v_transpose = iso_v;
        manip_v_transpose = manip_v;
    case 3
        data = redundancy_solution;
        name = 'Redundancy Resolution Solution';
        iso_w_redundancy = iso_w;
        iso_v_redundancy = iso_v;
        manip_v_redundancy = manip_v;
end

%% plot for presentation
figure('DefaultAxesFontSize',12);
plot(1:100,iso_w_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,iso_w_transpose(1:100),'g','lineWidth',3);
% plot(seq,iso_v,'b');
xlim([0 100]);
xlabel('iterations');
legend('inverse isotropy w','transpose isotropy w')

figure('DefaultAxesFontSize',12);
plot(1:100,iso_v_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,iso_v_transpose(1:100),'g','lineWidth',3);
% plot(seq,iso_v,'b');
xlim([0 100]);
xlabel('iterations');
legend('inverse isotropy v','transpose isotropy v')

figure('DefaultAxesFontSize',12);
plot(1:100,manip_v_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,manip_v_transpose(1:100),'g','lineWidth',3);
xlim([0 100]);
xlabel('iterations');
legend('inverse manipulability volume','transpose manipulability volume');

%% inverse vs redundancy
figure('DefaultAxesFontSize',12);
plot(1:100,iso_w_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,iso_w_redundancy(1:100),'b','lineWidth',3);
% plot(seq,iso_v,'b');
xlim([0 100]);
xlabel('iterations');
legend('inverse isotropy w','redundancy isotropy w')

figure('DefaultAxesFontSize',12);
plot(1:100,iso_v_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,iso_v_redundancy(1:100),'b','lineWidth',3);
% plot(seq,iso_v,'b');
xlim([0 100]);
xlabel('iterations');
legend('inverse isotropy v','redundancy isotropy v')

figure('DefaultAxesFontSize',12);
plot(1:100,manip_v_inverse(1:100),'r','lineWidth',3);hold on;
plot(1:100,manip_v_redundancy(1:100),'b','lineWidth',3);
xlim([0 100]);
xlabel('iterations');
legend('inverse manipulability volume','redundancy manipulability volume');