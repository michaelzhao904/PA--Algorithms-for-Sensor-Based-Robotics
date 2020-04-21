# function package for PA3 by Qishen
import numpy as np


def registration3D(pts_a, pts_b):
    # ptsA: (n,3) 3D data points
    # ptsB: (n,3) 3D data points
    # return T: (4,4) [R p;0 1] transformation matrix
    ave_a = np.mean(pts_a, 0)
    ave_b = np.mean(pts_b, 0)
    n = np.shape(pts_a)[0]
    ave_rep_a = np.tile(ave_a, (n, 1))
    ave_rep_b = np.tile(ave_b, (n, 1))
    np.shape(ave_rep_a)
    tilde_a = pts_a - ave_rep_a
    tilde_b = pts_b - ave_rep_b
    # find R using SVD approach
    H = np.zeros((3, 3))
    for i in range(n):
        a_i = tilde_a[i, :][:, None]
        b_i = tilde_b[i, :][None, :]
        H_i = a_i.dot(b_i)
        H += H_i
    u, s, vt = np.linalg.svd(H)
    R = vt.T.dot(u.T)
    # find p vector
    p = ave_b - R.dot(ave_a)
    p = p[:, None]
    T = np.c_[R, p]
    T = np.concatenate((T, np.array([[0, 0, 0, 1]])))
    return T