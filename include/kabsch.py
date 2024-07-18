import numpy as np
# Credit: Mike OShea
# https://gist.github.com/oshea00/dfb7d657feca009bf4d095d4cb8ea4be

# Implements Kabsch algorithm - best fit.
# Supports scaling (umeyama)
# Compares well to SA results for the same data.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns s,R,t
# s = scale B to A
# R = 3x3 rotation matrix (B to A)
# t = 3x1 translation vector (B to A)


def kabsch(A, B, scale=False):
    # Nx3
    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    if scale:
        H = BB.T.dot(AA) / N
    else:
        H = BB.T.dot(AA)

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T.dot(U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        # print "Reflection detected"
        Vt[2, :] *= -1
        R = Vt.T.dot(U.T)

    if scale:
        varA = np.var(A, axis=0).sum()
        c = 1 / (1 / varA * np.sum(S))  # scale factor
        t = -R.dot(centroid_B.T * c) + centroid_A.T
    else:
        c = 1
        t = -R.dot(centroid_B.T) + centroid_A.T

    return R, t, c
