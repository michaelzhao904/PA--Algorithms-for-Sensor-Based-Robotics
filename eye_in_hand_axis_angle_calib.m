function [R_axis,T_axis] = eye_in_hand_axis_angle_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
%%This algorithm is for hand-eye calibration for eye in hand problem using axis-angle approach%%
n = size(q_Robot_config,1);
E = zeros(4,4,n);
S = zeros(4,4,n);
for i = 1:n
    E((1:3),(1:3),i)= cvt_quat2R(q_Robot_config(i,:));     %converting the quaternion representation to rotation matrix R
    S((1:3),(1:3),i)= cvt_quat2R(q_camera_config(i,:));
    E((1:3),4,i)= t_Robot_config(i,:)';
    S((1:3),4,i)= t_camera_config(i,:)';
    E(4,:,i)= [0 0 0 1];
    S(4,:,i)= [0 0 0 1];
end
A = zeros(4,4,(n-1));
B = zeros(4,4,(n-1)); 
for k = 1:(n-1)
     A(:,:,k)= inv_T(E(:,:,k))*E(:,:,(k+1));   %Building the A and B matrices from two consecutive configurations
     B(:,:,k)= S(:,:,k)*inv_T(S(:,:,(k+1)));
end     
beta = zeros(3,1,(n-1));
alpha = zeros(3,1,(n-1));
for j = 1:(n-1)
    beta(:,:,j)  =  cvt_R2rotvec(B((1:3),(1:3),j));     %Taking matrix logarithm of matrices A and B
    alpha(:,:,j) =  cvt_R2rotvec(A((1:3),(1:3),j));
end
M_dash = zeros(3,3,(n-1));
for j = 1:(n-1)
    M_dash(:,:,j) = beta(:,:,j)*transpose(alpha(:,:,j));  %Calculating the product of alpha and beta
end
M = zeros(3,3);
D = zeros(3,3);
for j = 1:(n-1)
    M = D + M_dash(:,:,j);   %adding all the M_dash                               
    D = M;
end
R_axis = (transpose(M)*M)^(-0.5)*transpose(M)  %calculating the value of R
%%This part of the code calculates the translation vector using axis angle method %%
I = eye(3);  
K = zeros(3,3,(n-1));
for j = 1:(n-1)
    K(:,:,j) = I-A((1:3),(1:3),j);         %Calculating the value of K as I-A
end
W_1 = num2cell(K,[1,2]);
W = vertcat(W_1{:});                %concatenating all the matrices vertically
Z = zeros(3,1,(n-1));         
for j =1:(n-1)
    Z(:,:,j) = A((1:3),4,j)-R_axis*B((1:3),4,j);
end
Y_1 = num2cell(Z,[1,2]);
Y = vertcat(Y_1{:});                              %concatenating all the matrices vertically
T_axis = inv(transpose(W)*W)*transpose(W)*Y      %This is the solution for AX=B if the matrix A is tall (least squares method)
end