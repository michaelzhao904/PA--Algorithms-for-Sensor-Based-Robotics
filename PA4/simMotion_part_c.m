function simMotion(robot,p_goal,data,frames,wall)
%VISUALROBOT Visualize the robot and simulate the movement with data input
%   robot: robot tree model; T_sd: desired end-effector pose; data:times
%   series data:(time step,joint); steps: totoal frames
d = norm(wall);
n_vec = (wall/d)';
w = null(n_vec); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-50:50); % Provide a gridwork (you choose the size)
X = 1+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = 1+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = 1+w(3,1)*P+w(3,2)*Q;
% Take wall to be [0.3,-0.8,0.4]
n = size(data,1);
for i = 1:round(n/frames):n
    show(robot,data(i,:),'PreservePlot',false);
    surf(X,Y,Z);
    hold on;
    xlim([-1,1]);ylim([-1,1]);zlim([0,1.5]);
    %     plot3(T_sd(1,4),T_sd(2,4),T_sd(3,4),'r.','MarkerSize',20);
    T = getTransform(robot,data(i,:),'body8','base');
    plot3(T(1,4),T(2,4),T(3,4),'r.','MarkerSize',20);
    plot3(p_goal(1),p_goal(2),p_goal(3),'b.','MarkerSize',20);
    drawnow;
end
end

