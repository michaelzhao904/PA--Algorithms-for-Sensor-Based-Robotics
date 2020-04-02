% T_sd=M;
% tab_body=tab_b;
% angle_err=1e-1;
% vel_err = 1e-1;
% 
% w = ones(1,3);      %used for initialising the variables n and n1
% v = ones(1,3);
% n = norm(w);        
% n1 = norm(v);
% inverse_solution = config';
% while n > angle_err || n1 > vel_err
%     %T_sb(i) = FK_body(M,tab_body,config(i));
%     %T_mul = inv(T_sb(i))*T_sd;
%     T_sb = FK_body(M,tab_body,config);
%     T_bs = [T_sb(1:3,1:3)' -T_sb(1:3,1:3)'*T_sb(1:3,4);
%         zeros(1,3), 1];
%     T_mul = T_bs*T_sd;
%     R = T_mul(1:3,1:3);
%     p = T_mul(1:3,4);
%     [w, theta] = cvt_R2rotvec(R);
%     W = skewSymm(w);
%     G_inv =  eye(3)/theta - 0.5*W + (1/theta-0.5*cot(theta/2))*W^2;
%     v = G_inv*p;
%     V_twist = [w ; v];
%     n = norm(w);
%     n1 = norm(v);
%     config = config + pinv(J_body(tab_body, config))*V_twist;
%     %config(i+1) = config(i) + pinv(J_body(tab_body, config(i)))*V_twist 
%     %i = i+1;
%     inverse_solution = [inverse_solution;config'];
% end

while 1
    pause(1);
    w7
end