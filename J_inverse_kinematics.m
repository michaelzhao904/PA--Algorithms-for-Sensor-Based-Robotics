function inverse_solution = J_inverse_kinematics(T_sd, M, tab_body, config,delta,dt)
%This function is for calculating the inverse kinematics from the final...
% config and make sure the errors are in limit. dt assumes to be 1 sec
%i = 0;
w = ones(1,3);      %used for initialising the variables n and n1
v = ones(1,3);
n = norm(w);        
n1 = norm(v);
angle_err = delta(1);
vel_err = delta(2);
inverse_solution = config';
while n > angle_err || n1 > vel_err
    T_sb = FK_body(M,tab_body,config);
    T_bs = [T_sb(1:3,1:3)' -T_sb(1:3,1:3)'*T_sb(1:3,4);
        zeros(1,3), 1];
    T_mul = T_bs*T_sd;
    R = T_mul(1:3,1:3);
    p = T_mul(1:3,4);
    [w, theta] = cvt_R2rotvec(R);
    W = skewSymm(w);
    G_inv =  eye(3)/theta - 0.5*W + (1/theta-0.5*cot(theta/2))*W^2;
    v = G_inv*p;
    V_twist = [w ; v];
    n = norm(w*theta);
    n1 = norm(v*theta);
    config = config + pinv(J_body(tab_body, config))*V_twist*dt;
    inverse_solution = [inverse_solution;config'];
end
end
