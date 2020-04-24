%%This algorithm uses the functions which we have developed for calculation
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion()
[R_axis,T_axis] = eye_in_hand_axis_angle_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)
[R_quat,T_quat] = eye_in_hand_quat_calib(q_Robot_config,q_camera_config,t_Robot_config,t_camera_config)