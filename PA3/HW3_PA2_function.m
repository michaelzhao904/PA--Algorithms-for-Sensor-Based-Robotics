%% Eye in hand calibration using axis-angle and quaternion approach
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config] = data_quaternion();
[R_axis,T_axis] = eye_in_hand_axis_angle_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
[R_quat,T_quat] = eye_in_hand_quat_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
%% Error calculation for rotation and translation matrices
error_test = R_axis - R_quat;
error_R_percentage = norm(error_test)*100    %between axis-angle and quaternion approaches
error_test1 = T_axis-T_quat;
error_T_percentage = norm(error_test1)*100   %between axis-angle and quaternion approaches
%% Bonus question calculation Part a (taking all the ten observations) noisy data
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config] = data_quaternion_noisy();
[R_axis_noisy,T_axis_noisy] = eye_in_hand_axis_angle_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
[R_quat_noisy,T_quat_noisy] = eye_in_hand_quat_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
%% Comparing the results of noisy and noiseless outputs
error_test2 = R_axis - R_axis_noisy;
error_R_axis_noise_vs_noiseless_percent = norm(error_test2)*100    %between noisy and noise free data
error_test3 = T_axis - T_axis_noisy;
error_T_axis_noise_vs_noiseless_percent = norm(error_test3)*100    %between noisy and noise free data
error_test4 = R_quat - R_quat_noisy;
error_R_quat_noise_vs_noiseless_percent = norm(error_test4)*100    %between noisy and noise free data
error_test5 = T_quat - T_quat_noisy;
error_T_quat_noise_vs_noiseless_percent = norm(error_test5)*100    %between noisy and noise free data
%%  Bonus question calculation Part a (taking five observations)
[R_axis_noisy_5,T_axis_noisy_5] = eye_in_hand_axis_angle_calib(q_Robot_config((1:5),:),q_camera_config((1:5),:),t_Robot_config((1:5),:),t_camera_config((1:5),:))
[R_quat_noisy_5,T_quat_noisy_5] = eye_in_hand_quat_calib(q_Robot_config((1:5),:),q_camera_config((1:5),:),t_Robot_config((1:5),:),t_camera_config((1:5),:))
%% Comparing the results of 5 observations and noiseless data
error_test6 = R_axis - R_axis_noisy_5;
error_percent_R_axis_noiseless_vs_5 = norm(error_test6)*100
error_test7 = T_axis - T_axis_noisy_5;
error_percent_T_axis_noiseless_vs_5 = norm(error_test7)*100
error_test8 = R_quat - R_quat_noisy_5;
error_percent_R_quat_noiseless_vs_5 = norm(error_test8)*100
error_test9 = T_quat - T_quat_noisy_5;
error_percent_T_quat_noiseless_vs_5 = norm(error_test9)*100
%% Comparing the resuts of noisy data 10 vs 5
error_test10 = R_axis_noisy - R_axis_noisy_5;
error_percent_R_axis_10_vs_5 = norm(error_test10)*100
error_test11 = T_axis_noisy - T_axis_noisy_5;
error_percent_T_axis_10_vs_5 = norm(error_test11)*100
error_test12 = R_quat_noisy - R_quat_noisy_5;
error_percent_R_quat_10_vs_5 = norm(error_test12)*100
error_test13 = T_quat_noisy - T_quat_noisy_5;
error_percent_T_quat_10_vs_5 = norm(error_test13)*100






