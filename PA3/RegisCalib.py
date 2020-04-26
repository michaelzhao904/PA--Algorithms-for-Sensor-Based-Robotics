# function package for PA3 by Qishen
import numpy as np

# define function for Correspondance-based Method Algorithm
def registration3D(pts_a, pts_b):
    # ptsA: (n,3) 3D data points
    # ptsB: (n,3) 3D data points
    # return T: (4,4) [R p;0 1] transformation matrix
    # This is an implementation of Correspondence-based Methods algorithm in page 12-16 in W7-L1

    # Step 1: compute the mean value of data set a and b
    ave_a = np.mean(pts_a, 0)
    ave_b = np.mean(pts_b, 0)
    # Step 2: compute a and b tilde
    n = np.shape(pts_a)[0]
    ave_rep_a = np.tile(ave_a, (n, 1))
    ave_rep_b = np.tile(ave_b, (n, 1))
    np.shape(ave_rep_a)
    tilde_a = pts_a - ave_rep_a
    tilde_b = pts_b - ave_rep_b
    # Step 3: find R using SVD approach
    H = np.zeros((3, 3))
    for i in range(n):
        a_i = tilde_a[i, :][:, None]
        b_i = tilde_b[i, :][None, :]
        H_i = a_i.dot(b_i)
        H += H_i
    u, s, vt = np.linalg.svd(H)
    R = vt.T.dot(u.T)
    # Step 4: find p vector
    p = ave_b - R.dot(ave_a)
    p = p[:, None]
    # Step 5： find desired transformation
    T = np.c_[R, p]
    T = np.concatenate((T, np.array([[0, 0, 0, 1]])))
    return T