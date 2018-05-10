import numpy as np


def cartesian2lighthouse(point):     # takes an array of cartesian coordinates (x, y, z) and returns a numpy array of
    rtp = [0, 0, 0]                 # equivalent spherical coordinates (rho, theta, phi).
    rtp[0] = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
    rtp[1] = np.arctan(point[0]/point[1])
    rtp[2] = np.arctan(point[2]/point[1])
    return np.asarray(rtp)


def lighthouse2cartesian(point):
    xyz = [0, 0, 0]
    xyz[0] = point[0]*np.cos(np.arctan(np.cos(point[1])*np.tan(point[2])))*np.sin(point[1])
    xyz[1] = point[0]*np.cos(np.arctan(np.cos(point[1])*np.tan(point[2])))*np.cos(point[1])
    xyz[2] = point[0]*np.cos(np.arctan(np.cos(point[1])*np.tan(point[2])))*np.cos(point[1])*np.tan(point[2])
    return np.asarray(xyz)
