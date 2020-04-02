function visualRobot(robot,T_sd,data,frames)
%VISUALROBOT Visualize the robot and simulate the movement with data input
%   robot: robot tree model; T_sd: desired end-effector pose; data:times
%   series data:(time step,joint); steps: totoal frames
n = size(data,1);
for i = 1:round(n/frames):n
    show(robot,data(i,:),'PreservePlot',false);
    hold on; 
    xlim([-1,1]);ylim([-1,1]);zlim([0,1.5]);
%     plot3(T_sd(1,4),T_sd(2,4),T_sd(3,4),'r.','MarkerSize',20);
    T = getTransform(robot,data(i,:),'body8','base');
    plot3(T(1,4),T(2,4),T(3,4),'r.','MarkerSize',20);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,1),T_sd(2,1),T_sd(3,1),.1,'r','lineWidth',2);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,2),T_sd(2,2),T_sd(3,2),.1,'g','lineWidth',2);
    quiver3(T_sd(1,4),T_sd(2,4),T_sd(3,4),T_sd(1,3),T_sd(2,3),T_sd(3,3),.1,'b','lineWidth',2);
    drawnow;
end
end
