function tab_config = J_transpose_kinematics(T_sd, M, tab_body, config, K, delta, dt)
%J_TRANSPOSE_KINEMATICS uses the Lyapunov direct method
%   T_sd, M, tab_body, config is the same as J_inverse_kinematics.
%   K: symmetric positive definite matrix(6-by-6)
%   delta:[w_tolerance, v_tolerance]; dt:timestep
tab_config = config';
while 1
    T_sb = FK_body(M,tab_body,config);
    T_bs = [T_sb(1:3,1:3)' -T_sb(1:3,1:3)'*T_sb(1:3,4);
        zeros(1,3), 1];
    T_bd = T_bs*T_sd;
    R = T_bd(1:3,1:3);
    p = T_bd(1:3,4);
    [w, theta] = cvt_R2rotvec(R);
    W = skewSymm(w);
    G_inv =  eye(3)/theta - 0.5*W + (1/theta-0.5*cot(theta/2))*W^2;
    v = G_inv*p;
    
    err_w = norm(w*theta);
    err_v = norm(v*theta);
    
    S_theta = theta*[w; v];
    
    J_A = J_body(tab_body, config);
    config_dot = J_A'*K*S_theta;
    config = config + config_dot*dt;
    tab_config = [tab_config; config'];
    if err_w < delta(1) && err_v < delta(2) % check if within tolerance
        break
    end
end
end