import numpy as np


def cartesian2lighthouse(point):     # takes an array of cartesian coordinates (x, y, z) and returns a numpy array of
    rtp = [0, 0, 0]                 # equivalent spherical coordinates (rho, theta, phi).
    rtp[0] = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
    rtp[1] = np.arctan(point[0]/point[1])
    rtp[2] = np.arctan(point[2]/point[1])
    return np.asarray(rtp)