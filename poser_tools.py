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


def rigid_transform_3D(A, B):  # Function the solves translation vector and rotation matrix given 2 views
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
    k1 = pow((m_points[s_id[0]][0] - m_points[s_id[1]][0]), 2) + \
         pow((m_points[s_id[0]][1] - m_points[s_id[1]][1]), 2) + \
         pow((m_points[s_id[0]][2] - m_points[s_id[1]][2]), 2)
    k2 = pow((m_points[s_id[2]][0] - m_points[s_id[1]][0]), 2) + \
         pow((m_points[s_id[2]][1] - m_points[s_id[1]][1]), 2) + \
         pow((m_points[s_id[2]][2] - m_points[s_id[1]][2]), 2)
    k3 = pow((m_points[s_id[0]][0] - m_points[s_id[2]][0]), 2) + \
         pow((m_points[s_id[0]][1] - m_points[s_id[2]][1]), 2) + \
         pow((m_points[s_id[0]][2] - m_points[s_id[2]][2]), 2)
    k4 = pow((m_points[s_id[0]][0] - m_points[s_id[3]][0]), 2) + \
         pow((m_points[s_id[0]][1] - m_points[s_id[3]][1]), 2) + \
         pow((m_points[s_id[0]][2] - m_points[s_id[3]][2]), 2)

    phi_1 = 3.14159265359 - o_points[0][1]
    phi_2 = 3.14159265359 - o_points[1][1]
    phi_3 = 3.14159265359 - o_points[2][1]
    phi_4 = 3.14159265359 - o_points[3][1]
    theta_1 = o_points[0][0]
    theta_2 = o_points[1][0]
    theta_3 = o_points[2][0]
    theta_4 = o_points[3][0]
    h1 = cos(arctan(sin(theta_1)*tan(phi_1)))
    h2 = cos(arctan(sin(theta_2)*tan(phi_2)))
    h3 = cos(arctan(sin(theta_3)*tan(phi_3)))
    h4 = cos(arctan(sin(theta_4)*tan(phi_4)))

    def equations(z):  # Equation used by scipy
        r1 = z[0]
        r2 = z[1]
        r3 = z[2]
        r4 = z[3]

        F = empty(4)
        F[0] = ((r1*h1*cos(theta_1)-r2*h2*cos(theta_2))**2) + ((r1*h1*sin(theta_1)-r2*h2*sin(theta_2))**2) + ((r1*h1*sin(theta_1)*tan(phi_1)-r2*h2*sin(theta_2)*tan(phi_1))**2)-k1
        F[1] = ((r2*h2*cos(theta_2)-r3*h3*cos(theta_3))**2) + ((r2*h2*sin(theta_2)-r3*h3*sin(theta_3))**2) + ((r2*h2*sin(theta_2)*tan(phi_2)-r3*h3*sin(theta_3)*tan(phi_3))**2)-k2
        F[2] = ((r3*h3*cos(theta_3)-r4*h4*cos(theta_4))**2) + ((r3*h3*sin(theta_3)-r4*h4*sin(theta_4))**2) + ((r3*h3*sin(theta_3)*tan(phi_3)-r4*h4*sin(theta_4)*tan(phi_4))**2)-k3
        F[3] = ((r4*h4*cos(theta_4)-r1*h1*cos(theta_1))**2) + ((r4*h4*sin(theta_4)-r1*h1*sin(theta_1))**2) + ((r4*h4*sin(theta_4)*tan(phi_4)-r1*h1*sin(theta_1)*tan(phi_1))**2)-k4
        return F

    # Need to find the best nonlinear solver for this
    # z = scipy.optimize.fsolve(equations, zGuess, xtol=1.5e-05)
    z = scipy.optimize.least_squares(equations, guess, max_nfev=10000, method='lm')

    r1 = z.x[0]
    r2 = z.x[1]
    r3 = z.x[2]
    r4 = z.x[3]

    if not z.success:
        print("Convergence Error")


    A_2 = [[m_points[s_id[0]][0], m_points[s_id[0]][1], m_points[s_id[0]][2]],
                     [m_points[s_id[1]][0], m_points[s_id[1]][1], m_points[s_id[1]][2]],
                     [m_points[s_id[2]][0], m_points[s_id[2]][1], m_points[s_id[2]][2]],
                     [m_points[s_id[3]][0], m_points[s_id[3]][1], m_points[s_id[3]][2]]]

    B_2 = [[r1*h1*cos(theta_1), r1*h1*sin(theta_1), r1*h1*sin(theta_1)*tan(phi_1)],
           [r2*h2*cos(theta_2), r2*h2*sin(theta_2), r2*h2*sin(theta_2)*tan(phi_2)],
           [r3*h3*cos(theta_3), r3*h3*sin(theta_3), r3*h3*sin(theta_3)*tan(phi_3)],
           [r4*h4*cos(theta_4), r4*h4*sin(theta_4), r4*h4*sin(theta_4)*tan(phi_4)]]

    A_2 = np.matrix(A_2)
    B_2 = np.matrix(B_2)
    ret_R, ret_t = rigid_transform_3D(A_2, B_2)

    if verbose:
        print("Translation Vector")
        print(ret_t)

        print("Rotation Matrix")
        print(ret_R)

        #print("Euler Angles")
        #print(phi, theta, gamma)
        # print(z)
        dist_r = sqrt(ret_t[0]**2 + ret_t[1]**2 + ret_t[2]**2)
        print("Distance", dist_r)
        # print("RMSE:", rmse)

    return ret_R, ret_t, z.x