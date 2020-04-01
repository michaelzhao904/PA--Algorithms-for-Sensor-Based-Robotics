function redundancy_solution = redundancy_resolution(T_sd, M, tab_body,...
    config,angle_err, vel_err, k0, dt)
%This function is for calculating the inverse kinematics with an added aim 
%of maximizing the manipulability measure and aim at exploiting 
%redundancy by moving away from singulariites.
%T_sd is the final desired configuration of the robot manipulator and it
%should be within the velocity and angular errors specified by the user.
%M and tab_body describe the geometry of the robot. config is the initial
%guess we make for iterations to the required configuration
%k0: a positive constant coefficient for gradient ascend
%dt: time step of simultaion
%redundancy_solution: time series of joint angles. 1st dimension for time,
%2nd dimension for joint angles

p_dim = size(tab_body,1);
w = ones(1,3);      %used for initialising the variables n and n1
v = ones(1,3);
n = norm(w);        
n1 = norm(v);
redundancy_solution = config';
while n > angle_err || n1 > vel_err
    T_sb = FK_body(M,tab_body,config);
    T_bs = [T_sb(1:3,1:3)' -T_sb(1:3,1:3)'*T_sb(1:3,4);
        zeros(1,3), 1];
    T_bd = T_bs*T_sd;
    R = T_bd(1:3,1:3);
    p = T_bd(1:3,4);
    [w, theta] = cvt_R2rotvec(R);
    [W] = skewSymm(w);
    G_inv =  eye(3)/theta - 0.5*W + (1/theta-0.5*cot(theta/2))*W^2;
    v = G_inv*p;
    V_twist = [w ; v];
    n = norm(w);
    n1 = norm(v);
    
    J = J_body(tab_body, config);
    J_pinv = pinv(J);
    
    w_q = dwdq(@J_body,tab_body,config);
    q0_dot = k0*w_q';
    
    q_dot = J_pinv*V_twist+(eye(p_dim)-J_pinv*J)*q0_dot;
    config = config + q_dot*dt;
    redundancy_solution = [redundancy_solution;config'];
end

