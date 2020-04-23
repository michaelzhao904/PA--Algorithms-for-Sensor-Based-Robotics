%% This algorithm is for hand-eye calibration for eye in hand problem using axis-angle approach%%
E = zeros(3,3,10);
S = zeros(3,3,10);
for i = 1:10
    E(:,:,i)= cvt_quat2R(q_Robot_config(i,:));
    S(:,:,i)= cvt_quat2R(q_camera_config(i,:));
end
A = zeros(3,3,9);
B = zeros(3,3,9); 
 for j = 1:9
     A(:,:,j)= inv(E(:,:,j))*E(:,:,(j+1));
     B(:,:,j)= S(:,:,j)*inv(S(:,:,(j+1)));
 end       
beta = zeros(3,1,9);
alpha = zeros(3,1,9);
for i = 1:9
    beta(:,:,i)  =  cvt_R2rotvec(B(:,:,i));
    alpha(:,:,i) =  cvt_R2rotvec(A(:,:,i));
end
M_dash = zeros(3,3,9);
for k = 1:9
    M_dash(:,:,k) = beta(:,:,k)*transpose(alpha(:,:,k));
end
M = zeros(3,3);
D = zeros(3,3);
for k = 1:9
    M = D + M_dash(:,:,k);
    D = M;
end
R_axis = (transpose(M)*M)^(-0.5)*transpose(M) 
%% This part of the code calculates the translation vector using axis angle method%%
I = eye(3);
K = zeros(3,3,9);
for i = 1:9
    K(:,:,i) = I-A(:,:,i);
end
%W = vertcat(K(:,:,1),K(:,:,2),K(:,:,3),K(:,:,4),K(:,:,5),K(:,:,6),K(:,:,7),K(:,:,8),K(:,:,9));
W_1 = num2cell(K,[1,2]);
W = vertcat(W_1{:});
Z = zeros(3,1,9);
for i =1:9
    Z(:,:,i) = transpose(t_Robot_config(i,:))-R_axis*transpose(t_camera_config(i,:));
end
%Y = vertcat(Z(:,:,1),Z(:,:,2),Z(:,:,3),Z(:,:,4),Z(:,:,5),Z(:,:,6),Z(:,:,7),Z(:,:,8),Z(:,:,9));
Y_1 = num2cell(Z,[1,2]);
Y = vertcat(Y_1{:});
%T = mldivide(Y,W)      %check this step if the function used is correct
T = inv(transpose(W)*W)*transpose(W)*Y      %This is the solution for AX=B if the matrix A is tall
%% Calculating value of R for using the quaternion approach%%
A_quat = zeros(1,4,9);
B_quat = zeros(1,4,9);
for k =1:9
    A_quat(:,:,k) = cvt_R2quat(A(:,:,k));
    B_quat(:,:,k) = cvt_R2quat(B(:,:,k));
end
M_quat = zeros(4,4,9);
for i = 1:9
    M_quat(1,1,i) = A_quat(1,1,i)-B_quat(1,1,i);
    %M_quat(1,(2:4),i) = transpose(A_quat(1,(2:4),i))-transpose(B_quat(1,(2:4),i))
    M_quat(1,(2:4),i) = A_quat(1,(2:4),i)- B_quat(1,(2:4),i);
    M_quat((2:4),1,i) = transpose(A_quat(1,(2:4),i)-B_quat(1,(2:4),i));
    M_quat((2:4),(2:4),i) = (A_quat(1,1,i)-B_quat(1,1,i))*eye(3) + skewSymm(A_quat(1,(2:4),i)+B_quat(1,(2:4),i));
end
M_quat_1 = num2cell(M_quat,[1,2]);
M_quat_2 = vertcat(M_quat_1{:});
[U,S,V] = svd(M_quat_2);
V_transpose = transpose(V);
Quat_rot = V_transpose(:,4);
R_quat = cvt_quat2R(Quat_rot)
norm1 = norm(Quat_rot);
%% This part of the code calculates the translational vector using quaternion approach %%
C = zeros(3,3,9);
for i= 1:9
    C(:,:,i) = A(:,:,i)-eye(3);
end
C_1 = num2cell(C,[1,2]);
C_quat = vertcat(C_1{:});
F = zeros(3,1,9);
for i =1:9
    F(:,:,i) = R_quat*transpose(t_camera_config(i,:))-transpose(t_Robot_config(i,:));
end
F_1 = num2cell(F,[1,2]);
F_quat = vertcat(F_1{:});   
T_quat = inv(transpose(C_quat)*C_quat)*transpose(C_quat)*F_quat















