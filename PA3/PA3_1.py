# Programming Assignment 3.1 by Qishen

import numpy as np
import RegisCalib

# specify data source. e.g. "debug-a" or "unknown-j"
dataSource = "debug-a"

# load data
calbody_data = np.loadtxt("HW3-PA1\pa1-{}-calbody.txt".format(dataSource), delimiter=',', skiprows=1)
calreadings_data = np.loadtxt("HW3-PA1\pa1-{}-calreadings.txt".format((dataSource)), delimiter=',', skiprows=1)

# problem PA 3.1.3
N_D, N_A, N_C = 8, 8, 27
N_frame = 8
N_tol = N_D + N_A + N_C
C_expected = np.zeros((N_C * N_frame, 3))
pts_d = calbody_data[0:N_D, :]
pts_a = calbody_data[N_D:(N_A + N_D), :]

for i in range(N_frame):

    pts_D = calreadings_data[(i * N_tol):(i * N_tol + N_D), :]
    # compute transformation F_D
    F_D = RegisCalib.registration3D(pts_d, pts_D)

    pts_A = calreadings_data[(N_D + i * N_tol):(N_D + i * N_tol + N_A), :]
    # compute transformation F_A
    F_A = RegisCalib.registration3D(pts_a, pts_A)

    # compute C_i^expected
    for j in range(N_C):
        C_expected[(i * N_C + j), :] = np.linalg.inv(F_D).dot(F_A).dot(np.append(calbody_data[N_D + N_A + j, :], 1))[
                                       0:3]
# print(C_expected)

# problem PA 3.1.4
# load EM tracking data
empivot_data = np.loadtxt("HW3-PA1\pa1-{}-empivot.txt".format(dataSource), delimiter=',', skiprows=1)
N_G, N_frame_pivot = 6, 12
# compute midpoint and the translated point coordinates
G_0 = np.mean(empivot_data[0:N_G, :], 0)
G_0_rep = np.tile(G_0, (N_G*N_frame_pivot, 1))
g = empivot_data - G_0_rep

A_em = np.zeros((N_frame_pivot*3, 6))
b_em = np.zeros((N_frame_pivot*3, 1))
# compute transformation F_G and formulate least square problem
for i in range(N_frame_pivot):
    pts_g = g[i*N_G:(i+1)*N_G, :]
    pts_G = g[0 * N_G:(0 + 1) * N_G, :]
    F_G = RegisCalib.registration3D(pts_g, pts_G)

    A_em[3*i:(3*i+3), :] = np.append(F_G[0:3, 0:3], -np.eye(3), axis=1)
    b_em[3*i:(3*i+3), 0] = -F_G[0:3, 3]

x_em = np.linalg.lstsq(A_em, b_em, rcond=None)[0]
# estimated post position with EM probe pivot calibration
P_xyz_em = x_em[3:]+G_0[:, None]

# problem PA 3.1.5
# load optical tracking data
optpivot_data = np.loadtxt("HW3-PA1\pa1-{}-optpivot.txt".format(dataSource), delimiter=',', skiprows=1)
N_H = 6

A_opt = np.zeros((N_frame_pivot*3, 6))
b_opt = np.zeros((N_frame_pivot*3, 1))

for i in range(N_frame_pivot):
    # compute transformation F_D and transform the observation to EM tracker coordinates
    pts_D_opt = optpivot_data[i*(N_H+N_D): (i*(N_H+N_D) + N_D), :]
    F_D_opt = RegisCalib.registration3D(pts_d, pts_D_opt)
    pts_h = np.append(optpivot_data[(i*(N_H+N_D) + N_D):(i+1)*(N_H+N_D), :], np.ones((N_H, 1)), 1)
    pts_h = np.linalg.inv(F_D_opt).dot(pts_h.T).T[:, 0:3]

    if i == 0:
        # find the midpoint of transformed h in the first frame
        G_0_opt = np.mean(pts_h, 0)
        pts_H = pts_h - G_0_opt
    # compute transformation F_H and formulate least square problem
    F_H = RegisCalib.registration3D(pts_h, pts_H)
    A_opt[3*i:(3*i+3), :] = np.append(F_H[0:3, 0:3], -np.eye(3), axis=1)
    b_opt[3*i:(3*i+3), 0] = -F_H[0:3, 3]

x_opt = np.linalg.lstsq(A_opt, b_opt, rcond=None)[0]
# estimated post position with optical probe pivot calibration
P_xyz_opt = x_opt[3:] + G_0_opt[:, None]

# Output text file
headers = "{}, {}, solution output for PA1-{}".format(N_C, N_frame, dataSource)
output = np.concatenate((P_xyz_em.T, P_xyz_opt.T, C_expected),axis=0)
np.savetxt('PA3-{}-output.txt'.format(dataSource), output, delimiter=',',header=headers, fmt='%10.5f')