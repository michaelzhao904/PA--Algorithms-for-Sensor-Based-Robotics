function simMotion(robot,p_goal,data,frames,wall)
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
    plot3(p_goal(1),p_goal(2),p_goal(3),'b.','MarkerSize',20);
    drawnow;
end
end

