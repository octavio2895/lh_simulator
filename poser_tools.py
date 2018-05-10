from numpy import *
import numpy as np
import scipy.optimize
from sim_tools import lighthouse2cartesian
import random


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
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    t = -R * centroid_A.T + centroid_B.T

    return R, t


def rigid_transform_2(A, B, origin):  # Function the solves translation vector and rotation matrix given 2 data sets.
    assert len(A) == len(B)
    N = A.shape[0]  # total points
    centroid_A = asmatrix([0, 0, 0])
    centroid_B = asmatrix(origin)

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

    t = centroid_B.T
    return R, t


def calc_pose(m_points, o_points, s_id, guess=array([3, 3, 3, 3]), verbose=False):

    if len(s_id) < 4:  # Ignores data with less than 4 points.
        print("Not enough points")
        return None
    # K is the square distance between 2 sensors, k1-k5 are used for the non-linear solver
    k = empty(6)
    k[0] = ((m_points[s_id[0]][0] - m_points[s_id[1]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[1]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[1]][2])**2)
    k[1] = ((m_points[s_id[2]][0] - m_points[s_id[1]][0])**2) + ((m_points[s_id[2]][1] - m_points[s_id[1]][1])**2) + ((m_points[s_id[2]][2] - m_points[s_id[1]][2])**2)
    k[2] = ((m_points[s_id[3]][0] - m_points[s_id[2]][0])**2) + ((m_points[s_id[3]][1] - m_points[s_id[2]][1])**2) + ((m_points[s_id[3]][2] - m_points[s_id[2]][2])**2)
    k[3] = ((m_points[s_id[0]][0] - m_points[s_id[3]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[3]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[3]][2])**2)
    k[4] = ((m_points[s_id[0]][0] - m_points[s_id[2]][0])**2) + ((m_points[s_id[0]][1] - m_points[s_id[2]][1])**2) + ((m_points[s_id[0]][2] - m_points[s_id[2]][2])**2)
    k[5] = ((m_points[s_id[3]][0] - m_points[s_id[1]][0])**2) + ((m_points[s_id[3]][1] - m_points[s_id[1]][1])**2) + ((m_points[s_id[3]][2] - m_points[s_id[1]][2])**2)

    phi = []
    theta = []
    for i in range(len(o_points)):
        phi.append((np.pi/2) - o_points[i][1])
        theta.append((np.pi/2) - o_points[i][0])

    h = []
    for i in range(len(o_points)):
        h.append(cos(arctan(cos(theta[i])*tan(phi[i]))))

    def equations(z):

        """
         Equations used to find the radii of the first 4 data points
        """

        r1 = z[0]
        r2 = z[1]
        r3 = z[2]
        r4 = z[3]


        F = empty(6)
        F[0] = ((r1*h[0]*sin(theta[0])-r2*h[1]*sin(theta[1]))**2) + ((r1*h[0]*cos(theta[0])-r2*h[1]*cos(theta[1]))**2) + ((r1*h[0]*cos(theta[0])*tan(phi[0])-r2*h[1]*cos(theta[1])*tan(phi[1]))**2)-k[0]
        F[1] = ((r2*h[1]*sin(theta[1])-r3*h[2]*sin(theta[2]))**2) + ((r2*h[1]*cos(theta[1])-r3*h[2]*cos(theta[2]))**2) + ((r2*h[1]*cos(theta[1])*tan(phi[1])-r3*h[2]*cos(theta[2])*tan(phi[2]))**2)-k[1]
        F[2] = ((r3*h[2]*sin(theta[2])-r4*h[3]*sin(theta[3]))**2) + ((r3*h[2]*cos(theta[2])-r4*h[3]*cos(theta[3]))**2) + ((r3*h[2]*cos(theta[2])*tan(phi[2])-r4*h[3]*cos(theta[3])*tan(phi[3]))**2)-k[2]
        F[3] = ((r4*h[3]*sin(theta[3])-r1*h[0]*sin(theta[0]))**2) + ((r4*h[3]*cos(theta[3])-r1*h[0]*cos(theta[0]))**2) + ((r4*h[3]*cos(theta[3])*tan(phi[3])-r1*h[0]*cos(theta[0])*tan(phi[0]))**2)-k[3]
        F[4] = ((r4*h[2]*sin(theta[2])-r1*h[0]*sin(theta[0]))**2) + ((r4*h[2]*cos(theta[2])-r1*h[0]*cos(theta[0]))**2) + ((r4*h[2]*cos(theta[2])*tan(phi[2])-r1*h[0]*cos(theta[0])*tan(phi[0]))**2)-k[4]
        F[5] = ((r2*h[1]*sin(theta[1])-r3*h[3]*sin(theta[3]))**2) + ((r2*h[1]*cos(theta[1])-r3*h[3]*cos(theta[3]))**2) + ((r2*h[1]*cos(theta[1])*tan(phi[1])-r3*h[3]*cos(theta[3])*tan(phi[3]))**2)-k[5]
        return F

    # Need to find the best nonlinear solver for this

    z = scipy.optimize.least_squares(equations, guess, max_nfev=10000, method='lm')
    # print(z)

    r = []
    for radius in z.x:
        r.append(radius)

    if not z.success:
        print("Convergence Error")

    # k_0 = []
    # for i in range(3):
    #     k_0.append(((m_points[s_id[i]][0])**2) + ((m_points[s_id[i]][1])**2) + ((m_points[s_id[i]][2])**2))
    #
    # origin_guess = array([r[0], theta[0], phi[0]])
    #
    # def find_origin(z): # These equations are used to compute the position of the devices origin relative to LH.
    #     r_0 = z[0]
    #     theta_0 = z[1]
    #     phi_0 = z[2]
    #
    #     F = empty(3)
    #     F[0] = ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*sin(theta_0)-r[0]*h[0]*sin(theta[0]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)-r[0]*h[0]*cos(theta[0]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)*tan(phi_0)-r[0]*h[0]*cos(theta[0])*tan(phi[0]))**2)-k_0[0]
    #     F[1] = ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*sin(theta_0)-r[1]*h[1]*sin(theta[1]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)-r[1]*h[1]*cos(theta[1]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)*tan(phi_0)-r[1]*h[1]*cos(theta[1])*tan(phi[1]))**2)-k_0[1]
    #     F[2] = ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*sin(theta_0)-r[2]*h[2]*sin(theta[2]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)-r[2]*h[2]*cos(theta[2]))**2) + ((r_0*cos(arctan(cos(theta_0)*tan(phi_0)))*cos(theta_0)*tan(phi_0)-r[2]*h[2]*cos(theta[2])*tan(phi[2]))**2)-k_0[2]
    #
    #     return F
    #
    # origin = scipy.optimize.least_squares(find_origin, origin_guess, max_nfev=10000, method='lm')

    possible_radii = []
    for index in range(4, len(o_points)):
        '''
            Calculates the radii of the remaining sensors using the previously calculated radii.
            The equation yields 2 possible radii. It sometimes generate complex radii.
        '''
        c1 = h[index]*sin(theta[index])
        c2 = r[0]*h[0]*sin(theta[0])
        c3 = h[index]*cos(theta[index])
        c4 = r[0]*h[0]*cos(theta[0])
        c5 = h[index]*cos(theta[index])*tan(phi[index])
        c6 = r[0]*h[0]*cos(theta[0])*tan(phi[0])
        c7 = (m_points[s_id[0]][0] - m_points[s_id[index]][0])**2 + (m_points[s_id[0]][1] - m_points[s_id[index]][1])**2 + (m_points[s_id[0]][2] - m_points[s_id[index]][2])**2

        i = (-(sqrt(abs((-(c1**2))*((c4**2) + (c6**2) - c7) + 2*c1*c2*(c3*c4 + c5*c6) - (c2**2) * ((c3**2) + (c5**2)) - (c3**2) * ((c6**2) - c7) + 2*c3*c4*c5*c6 - ((c4**2) - c7) * c5**2)) - c1*c2 - c3*c4 - c5*c6)/((c1**2) + (c3**2) + (c5**2)))
        j = ((sqrt(abs((-(c1**2))*((c4**2) + (c6**2) - c7) + 2*c1*c2*(c3*c4 + c5*c6) - (c2**2) * ((c3**2) + (c5**2)) - (c3**2) * ((c6**2) - c7) + 2*c3*c4*c5*c6 - ((c4**2) - c7) * c5**2)) + c1*c2 + c3*c4 + c5*c6)/((c1**2) + (c3**2) + (c5**2)))
        possible_radii.append([i, j, index])

    for i in range(len(possible_radii)):
        '''
            This chooses which of the 2 radii fits the system better.
        '''
        rand_sensor = random.randint(1, 3)
        dist_i = ((r[rand_sensor]*h[rand_sensor]*sin(theta[rand_sensor]) - possible_radii[i][0]*h[possible_radii[i][2]]*sin(theta[possible_radii[i][2]]))**2) + ((r[rand_sensor]*h[rand_sensor]*cos(theta[rand_sensor]) - possible_radii[i][0]*h[possible_radii[i][2]]*cos(theta[possible_radii[i][2]]))**2) + ((r[rand_sensor]*h[rand_sensor]*cos(theta[rand_sensor])*tan(phi[rand_sensor]) - possible_radii[i][0]*h[possible_radii[i][2]]*cos(theta[possible_radii[i][2]])*tan(phi[possible_radii[i][2]]))**2)
        dist_j = ((r[rand_sensor]*h[rand_sensor]*sin(theta[rand_sensor]) - possible_radii[i][1]*h[possible_radii[i][2]]*sin(theta[possible_radii[i][2]]))**2) + ((r[rand_sensor]*h[rand_sensor]*cos(theta[rand_sensor]) - possible_radii[i][1]*h[possible_radii[i][2]]*cos(theta[possible_radii[i][2]]))**2) + ((r[rand_sensor]*h[rand_sensor]*cos(theta[rand_sensor])*tan(phi[rand_sensor]) - possible_radii[i][1]*h[possible_radii[i][2]]*cos(theta[possible_radii[i][2]])*tan(phi[possible_radii[i][2]]))**2)
        error_i = abs((m_points[s_id[rand_sensor]][0] - m_points[possible_radii[i][2]][0])**2 + (m_points[s_id[rand_sensor]][1] - m_points[possible_radii[i][2]][1])**2 + (m_points[s_id[rand_sensor]][2] - m_points[possible_radii[i][2]][2])**2 - dist_i)
        error_j = abs((m_points[s_id[rand_sensor]][0] - m_points[possible_radii[i][2]][0])**2 + (m_points[s_id[rand_sensor]][1] - m_points[possible_radii[i][2]][1])**2 + (m_points[s_id[rand_sensor]][2] - m_points[possible_radii[i][2]][2])**2 - dist_j)
        if np.isnan(error_i) or np.isnan(error_j):
            continue
        if error_i > error_j:
            r.append(possible_radii[i][1])
        else:
            r.append(possible_radii[i][0])

    # for i in range(4, len(o_points)): # Finds the remaining radii of detected sensors using r[0].
    #     k = (m_points[s_id[0]][0] - m_points[s_id[i]][0])**2 + (m_points[s_id[0]][1] - m_points[s_id[i]][1])**2 + (m_points[s_id[0]][2] - m_points[s_id[i]][2])**2
    #
    #     def find_r(ro):
    #         return((r[0]*h[0]*sin(theta[0]) - ro*h[i]*sin(theta[i]))**2) + ((r[0]*h[0]*cos(theta[0]) - ro*h[i]*cos(theta[i]))**2) + ((r[0]*h[0]*cos(theta[0])*tan(phi[0]) - ro*h[i]*cos(theta[i])*tan(phi[i]))**2) - k
    #
    #     r_new = scipy.optimize.least_squares(find_r, r[0])
    #     print(r_new)
    #     r.append(r_new.x)

    B_2 = []
    A_2 = []
    for i in range(len(r)):
        A_2.append([m_points[s_id[i]][0], m_points[s_id[i]][1], m_points[s_id[i]][2]])
        B_2.append([r[i]*h[i]*sin(theta[i]), r[i]*h[i]*cos(theta[i]), r[i]*h[i]*cos(theta[i])*tan(phi[i])])

    A_2 = np.matrix(A_2)
    B_2 = np.matrix(B_2)
    # ret_R, ret_t = rigid_transform_2(A_2, B_2, lighthouse2cartesian(origin.x))
    ret_R, ret_t = rigid_transform_3D(A_2, B_2)


    if verbose:
        print("Translation Vector:")
        print(ret_t)

        print("Rotation Matrix:")
        print(ret_R)

        dist_r = sqrt(ret_t[0]**2 + ret_t[1]**2 + ret_t[2]**2)
        print("Distance:", dist_r)

    return ret_R, ret_t, z.x