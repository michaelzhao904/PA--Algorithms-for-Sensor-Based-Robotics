function [R_quat,T_quat] = eye_in_hand_quat_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
%%This algorithm is for hand-eye calibration for eye in hand problem using quaternion approach%%
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
A_quat = zeros(1,4,(n-1));
B_quat = zeros(1,4,(n-1));
for k =1:(n-1)
    A_quat(:,:,k) = cvt_R2quat(A((1:3),(1:3),k));      %Quaternion represenataions of matrices
    B_quat(:,:,k) = cvt_R2quat(B((1:3),(1:3),k));
end
M_quat = zeros(4,4,(n-1));
for i = 1:(n-1)
    M_quat(1,1,i) = A_quat(1,1,i)-B_quat(1,1,i);   % Building the M matrix
    M_quat(1,(2:4),i) = A_quat(1,(2:4),i)- B_quat(1,(2:4),i);
    M_quat((2:4),1,i) = transpose(A_quat(1,(2:4),i)-B_quat(1,(2:4),i));
    M_quat((2:4),(2:4),i) = (A_quat(1,1,i)-B_quat(1,1,i))*eye(3) + skewSymm(A_quat(1,(2:4),i)+B_quat(1,(2:4),i));
end
M_quat_1 = num2cell(M_quat,[1,2]);
M_quat_2 = vertcat(M_quat_1{:});
[U,S,V] = svd(M_quat_2);                                 
Quat_rot = V(:,4);                    % SVD decompostion of matrix M
R_quat = cvt_quat2R(Quat_rot)    %Converting to R matrix
%%This part of the code calculates the translational vector using quaternion approach %%
C = zeros(3,3,(n-1));
for i= 1:(n-1)
    C(:,:,i) = A((1:3),(1:3),i)-eye(3); %Calculating the translational vector
end
C_1 = num2cell(C,[1,2]);
C_quat = vertcat(C_1{:});
F = zeros(3,1,(n-1));
for i =1:(n-1)
    F(:,:,i) = R_quat*B((1:3),4,i)-A((1:3),4,i);
end
F_1 = num2cell(F,[1,2]);
F_quat = vertcat(F_1{:});         %concatenating all the matrices vertically
T_quat = inv(transpose(C_quat)*C_quat)*transpose(C_quat)*F_quat
end