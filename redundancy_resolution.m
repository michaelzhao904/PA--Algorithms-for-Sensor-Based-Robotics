function [redundancy_resolution] = redundancy_resolution(T_sd, M, tab_body,...
    config,angle_err, vel_err, k_o,q)
%This function is for calculating the inverse kinematics with an added aim 
%of maximizing the manipulability measure and aim at exploiting 
%redundancy by moving away from singulariites.
%T_sd is the final desired configuration of the robot manipulator and it
%should be within the velocity and angular errors specified by the user.
%M and tab_body describe the geometry of the robot. config is the initial
%guess we make for iterations to the required configuration. 
n2 = size(tab_body,1);
w = ones(1,3);      %used for initialising the variables n and n1
v = ones(1,3);
n = norm(w);        
n1 = norm(v);
redundancy_resolution = config;
while n > angle_err || n1 > vel_err
    T_sb = FK_body(M,tab_body,config);
    T_bs = [T_sb(1:3,1:3)' -T_sb(1:3,1:3)'*T_sb(1:3,4);
        zeros(1,3), 1];
    T_bd = T_bs*T_sd;
    R = T_bd([1:3],[1:3]);
    p = T_bd([1:3],4);
    [w, theta] = cvt_R2rotvec(R);
    [W] = skewSymm(w);
    G_inv =  eye(3)/theta - 0.5*[W] + (1/theta-0.5*cot(theta/2))*[W]^2;
    v = G_inv*p;
    V_twist = [w ; v];
    n = norm(w);
    n1 = norm(v);
    J(q)  = J_body(tab_body, config);
    w(q) = sqrt(det(J(q)*transpose(J(q)))) %maximize this function
    qo_dot = k_o*transpose(diff(w(q),q));
    q_dot = pinv(J(q))*V_twist + (eye(n2)-pinv(J(q))*J(q))*qo_dot;
    config = config + q_dot*dt;
    redundancy_resolution = [redundancy_resolution;config];
end

