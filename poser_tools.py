from numpy import *
import numpy as np
import scipy.optimize


def rotmat2euler(mat, rads = 1):  # Function that computes roll pitch and yaw from rotation matrix

    phi = arctan2(mat[2, 0], mat[2, 1])
    theta = arccos(mat[2, 2])
    gamma = -1 * arctan2(mat[0, 2], mat[1, 2])

    if rads == 1:
        return phi, theta, gamma

    else:
        phi = (phi/3.14159265359) * 180
        theta = (theta/3.14159265359) * 180
        gamma = (gamma/3.14159265359) * 180

        return phi, theta, gamma


def rotmat2quat(mat): # Function that computes quaternions from rotation matrix
    q = empty((1, 4))
    q[0] = sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2])/2
    q[1] = (mat[2][1] - mat[1][2])/(4 * q[0])
    q[2] = (mat[0][2] - mat[2][0])/(4 * q[0])
    q[3] = (mat[1][0] - mat[0][1])/(4 * q[0])
    return q


def rigid_transform_3D(A, B):  # Function the solves translation vector and rotation matrix given 2 data sets.
    assert len(A) == len(B)
    N = A.shape[0]  # total points
    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array

    H = transpose(AA) * BB
    U, S, Vt = linalg.svd(H)
    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
        # print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    t = -R * centroid_A.T + centroid_B.T

    return R, t


def calc_pose(m_points, o_points, s_id, guess=array([3, 3, 3, 3]), verbose=False):
    if len(s_id) < 5:  # Ignores data with less than 4 points.
        print("Not enough points")
        return None

    # K is the square distance between 2 sensors, k1-k4 are used for the non-linear solver
    k = empty(4)
    k[0] = ((m_points[s_id[0]][0] - m_points[s_id[1]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[1]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[1]][2])**2)
    k[1] = ((m_points[s_id[2]][0] - m_points[s_id[1]][0])**2) + ((m_points[s_id[2]][1] - m_points[s_id[1]][1])**2) + ((m_points[s_id[2]][2] - m_points[s_id[1]][2])**2)
    k[2] = ((m_points[s_id[0]][0] - m_points[s_id[2]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[2]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[2]][2])**2)
    k[3] = ((m_points[s_id[0]][0] - m_points[s_id[3]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[3]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[3]][2])**2)

    phi = []
    theta = []
    for i in range(len(o_points)):
        phi.append(np.pi - o_points[i][1])
        theta.append(o_points[i][0])

    h = []
    for i in range(len(o_points)):
        h.append(cos(arctan(sin(theta[i])*tan(phi[i]))))

    def equations(z):  # Equation used by scipy
        r1 = z[0]
        r2 = z[1]
        r3 = z[2]
        r4 = z[3]

        F = empty(4)
        F[0] = ((r1*h[0]*cos(theta[0])-r2*h[1]*cos(theta[1]))**2) + ((r1*h[0]*sin(theta[0])-r2*h[1]*sin(theta[1]))**2) + ((r1*h[0]*sin(theta[0])*tan(phi[0])-r2*h[1]*sin(theta[1])*tan(phi[1]))**2)-k[0]
        F[1] = ((r2*h[1]*cos(theta[1])-r3*h[2]*cos(theta[2]))**2) + ((r2*h[1]*sin(theta[1])-r3*h[2]*sin(theta[2]))**2) + ((r2*h[1]*sin(theta[1])*tan(phi[1])-r3*h[2]*sin(theta[2])*tan(phi[2]))**2)-k[1]
        F[2] = ((r3*h[2]*cos(theta[2])-r4*h[3]*cos(theta[3]))**2) + ((r3*h[2]*sin(theta[2])-r4*h[3]*sin(theta[3]))**2) + ((r3*h[2]*sin(theta[2])*tan(phi[2])-r4*h[3]*sin(theta[3])*tan(phi[3]))**2)-k[2]
        F[3] = ((r4*h[3]*cos(theta[3])-r1*h[0]*cos(theta[0]))**2) + ((r4*h[3]*sin(theta[3])-r1*h[0]*sin(theta[0]))**2) + ((r4*h[3]*sin(theta[3])*tan(phi[3])-r1*h[0]*sin(theta[0])*tan(phi[0]))**2)-k[3]
        return F

    # Need to find the best nonlinear solver for this
    # z = scipy.optimize.fsolve(equations, zGuess, xtol=1.5e-05)
    z = scipy.optimize.least_squares(equations, guess, max_nfev=10000, method='lm')
    r = []
    for radius in z.x:
        r.append(radius)

    if not z.success:
        print("Convergence Error")

    for i in range(4, len(o_points)): # Finds the remaining radii of detected sensors using r[0].
        k = (m_points[s_id[0]][0] - m_points[s_id[i]][0])**2 + (m_points[s_id[0]][1] - m_points[s_id[i]][1])**2 + (m_points[s_id[0]][2] - m_points[s_id[i]][2])**2

        def find_r(ro):
            return((r[0]*h[0]*cos(theta[0]) - ro*h[i]*cos(theta[i]))**2) + ((r[0]*h[0]*sin(theta[0]) - ro*h[i]*sin(theta[i]))**2) + ((r[0]*h[0]*sin(theta[0])*tan(phi[0]) - ro*h[i]*sin(theta[i])*tan(phi[i]))**2) - k

        r.append(scipy.optimize.least_squares(find_r, r[0]).x)

    B_2 = []
    A_2 = []
    for i in range(len(r)):
        A_2.append([m_points[s_id[i]][0], m_points[s_id[i]][1], m_points[s_id[i]][2]])
        B_2.append([r[i]*h[i]*cos(theta[i]), r[i]*h[i]*sin(theta[i]), r[i]*h[i]*sin(theta[i])*tan(phi[i])])

    A_2 = np.matrix(A_2)
    B_2 = np.matrix(B_2)
    ret_R, ret_t = rigid_transform_3D(A_2, B_2)

    if verbose:
        print("Translation Vector:")
        print(ret_t)

        print("Rotation Matrix:")
        print(ret_R)

        dist_r = sqrt(ret_t[0]**2 + ret_t[1]**2 + ret_t[2]**2)
        print("Distance:", dist_r)

    return ret_R, ret_t, z.x