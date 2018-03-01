import numpy as np
import json
import csv
from matplotlib import pyplot as plt



def open_config(file_name, j_file=True):  # Function that loads the model points from file into numpy array
    model = []
    normals = []
    if j_file:
        with open(file_name) as input_file:
            json_decode = json.load(input_file)
            for j in range(32):
                xyz = np.asarray(json_decode['lighthouse_config']['modelPoints'][j])
                ijk = np.asarray(json_decode['lighthouse_config']['modelNormals'][j])
                model.append(xyz)
                normals.append(ijk)
    else:
        with open(file_name) as input_file:
            sensors = csv.reader(input_file, delimiter=' ', quotechar='|')
            for sensor in sensors:
                xyz = [float(sensor[0]), float(sensor[1]), float(sensor[2])]
                model.append(xyz)
    return np.asarray(model), np.asarray(normals)


def calculate_translation(point, gradients, time=(1 / 60)):  # function that works with a simple translation
    translation = point + (gradients * time)
    return np.asarray(translation)


def cartesian2spherical(point):     # takes an array of cartesian coordinates (x, y, z) and returns a numpy array of
    rtp = [0, 0, 0]                 # equivalent spherical coordinates (rho, theta, phi).
    rtp[0] = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
    rtp[1] = np.arctan(point[1]/point[0])
    rtp[2] = np.arctan((np.sqrt((point[0]**2)+(point[1]**2))/point[2]))
    return np.asarray(rtp)

model_points, normal_points = open_config("config.json")

first_position = np.array([1, 0, 5])
second_position = np.array([1, 0, 5])
time = 2
current_time = 0
gradients = (second_position - first_position)/time

captured_position_theta = []
captured_position_phi = []

for sensor in model_points:
    cartesian_starting_position = sensor + first_position
    spherical_starting_position = cartesian2spherical(cartesian_starting_position)
    new_time = spherical_starting_position[1] / (120 * np.pi)

    for i in range(1000):
        new_position = cartesian2spherical(calculate_translation(cartesian_starting_position, gradients, time=new_time))
        next_time = new_position[1] / (120 * np.pi)
        diff = abs(new_time-next_time)
        new_time = next_time
        if diff < (1/120)/400000:   # if the error is smaller than the precision of the LH then the solution is
            break                   # good enough
    captured_position_theta.append(new_position[1])

    new_time = new_time + (1/120)
    for i in range(1000):
        new_position = cartesian2spherical(calculate_translation(cartesian_starting_position, gradients, time=new_time))
        next_time = new_position[2] / (120 * np.pi)
        diff = abs(new_time-next_time)
        new_time = next_time
        if diff < (1/120)/400000:   # if the error is smaller than the precision of the LH then the solution is
            break                   # good enough
    captured_position_phi.append(new_position[2])

captured_position_theta = np.asarray(captured_position_theta)
captured_position_phi = np.asarray(captured_position_phi)
print(captured_position_theta)
print(captured_position_phi)


plt.scatter(captured_position_phi, captured_position_theta, s=10, c='b', marker="s", label='Static')
plt.legend(loc='upper left')
plt.title('Simulation')
plt.axis((.1, .3, -.15, .15))
plt.show()