# Programming Assignment 3.1.3

import numpy as np
import RegisCalib

calbody_data = np.loadtxt("HW3-PA1\pa1-debug-a-calbody.txt", delimiter=',', skiprows=1)
calreadings_data = np.loadtxt("HW3-PA1\pa1-debug-a-calreadings.txt", delimiter=',', skiprows=1)
N_D, N_A, N_C = 8, 8, 27
N_frame = 8
N_tol = N_D + N_A + N_C
C_expected = np.zeros((N_C * N_frame, 3))
pts_d = calbody_data[0:N_D, :]
pts_a = calbody_data[N_D:(N_A + N_D), :]
for i in range(N_frame):

    pts_D = calreadings_data[(i * N_tol):(i * N_tol + N_D), :]
    F_D = RegisCalib.registration3D(pts_d, pts_D)

    pts_A = calreadings_data[(N_D + i * N_tol):(N_D + i * N_tol + N_A), :]
    F_A = RegisCalib.registration3D(pts_a, pts_A)

    for j in range(N_C):
        C_expected[(i * N_C + j), :] = np.linalg.inv(F_D).dot(F_A).dot(np.append(calbody_data[N_D + N_A + j, :], 1))[
                                       0:3]
print(C_expected)