function [R_axis,T_axis] = eye_in_hand_axis_angle_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
%%This algorithm is for hand-eye calibration for eye in hand problem using axis-angle approach%%
n = size(q_Robot_config,1);
E = zeros(3,3,n);
S = zeros(3,3,n);
for i = 1:n
    E(:,:,i)= cvt_quat2R(q_Robot_config(i,:));      %converting the quaternion representation to rotation matrix R
    S(:,:,i)= cvt_quat2R(q_camera_config(i,:));
end
A = zeros(3,3,(n-1));
B = zeros(3,3,(n-1)); 
 for j = 1:(n-1)
     A(:,:,j)= inv(E(:,:,j))*E(:,:,(j+1));   %Building the A and B matrices from two consecutive configurations
     B(:,:,j)= S(:,:,j)*inv(S(:,:,(j+1)));
 end       
beta = zeros(3,1,(n-1));
alpha = zeros(3,1,(n-1));
for j = 1:(n-1)
    beta(:,:,j)  =  cvt_R2rotvec(B(:,:,j));     %Taking matrix logarithm of matrices A and B
    alpha(:,:,j) =  cvt_R2rotvec(A(:,:,j));
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
R_axis = (transpose(M)*M)^(-0.5)*transpose(M);   %calculating the value of R
%%This part of the code calculates the translation vector using axis angle method %%
I = eye(3);  
K = zeros(3,3,(n-1));
for j = 1:(n-1)
    K(:,:,j) = I-A(:,:,j);         %Calculating the value of K as I-A
end
W_1 = num2cell(K,[1,2]);
W = vertcat(W_1{:});                %concatenating all the matrices vertically
Z = zeros(3,1,(n-1));         
for j =1:(n-1)
    Z(:,:,j) = transpose(t_Robot_config(j,:))-R_axis*transpose(t_camera_config(j,:));
end
Y_1 = num2cell(Z,[1,2]);
Y = vertcat(Y_1{:});                              %concatenating all the matrices vertically
T_axis = inv(transpose(W)*W)*transpose(W)*Y;      %This is the solution for AX=B if the matrix A is tall (least squares method)
end



