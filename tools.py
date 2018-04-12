from numpy import *
import numpy as np
import scipy.optimize
import json
import csv


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


def open_config_old(file_name, j_file=True):  # Function that loads the model points from file
    model = []
    normals = []
    if j_file:
        with open(file_name) as input_file:
            json_decode = json.load(input_file)
            for j in range(32):
                xyz = json_decode['lighthouse_config']['modelPoints'][j]
                ijk = json_decode['lighthouse_config']['modelNormals'][j]
                model.append(xyz)
                normals.append(ijk)
    else:
        with open(file_name) as input_file:
            sensors = csv.reader(input_file, delimiter=' ', quotechar='|')
            for sensor in sensors:
                xyz = [float(sensor[0]), float(sensor[1]), float(sensor[2])]
                model.append(xyz)
    return model, normals


def load_points(file_name, lighthouse="L", count=1, steps=1, dlimiter=' '):  # Returns angles from csv file for any
    act = ""                                                                 # specified lighthouse and number of poses
    i = 0                                                                    # count will determine how many to poses to
    location_x = np.zeros(32)                                                # skip
    location_y = np.zeros(32)
    xyflag = 0
    parseflag = 0
    all_points = []
    all_ids = []
    with open(file_name, newline='') as csvfile:  # Loads csv
        track = csv.reader(csvfile, delimiter=dlimiter, quotechar='|')
        for sweep in track:
            if sweep[0] != lighthouse:  # Discards any data that's not related to lighthouse
                continue
            if act != sweep[1]:  # Will wait until second column changes X-Y. Buggy.
                i += 1
                act = sweep[1]
            if i > steps*2:  # Will wait until X and Y are collected and count the number of poses parsed
                xyflag = 1
                parseflag += 1
                i = 1
            if xyflag == 1:  # Will store X-Y values in list with IDs in order
                sensor_ids = []
                sensor_points = []
                for k in range(0, 31):
                    if location_x[k] != 0 and location_y[k] != 0:  # Rejects data without a pair
                        x = (location_x[k]/400000)*3.14159265359
                        y = (location_y[k]/400000)*3.14159265359
                        point = [x, y]
                        sensor_points.append(point)
                        sensor_ids.append(k)
                xyflag = 0
                all_points.append(sensor_points)
                all_ids.append(sensor_ids)
                location_x = np.zeros(32)
                location_y = np.zeros(32)
            if parseflag == count:  # Will break when number of poses are parsed
                break
            if sweep[1] == 'X':
                location_x[int(sweep[4])] = int(sweep[6].strip())
            else:
                location_y[int(sweep[4])] = int(sweep[6].strip())
    return all_points, all_ids

def load_points_raw(file_name, lighthouse="L", count=1, steps=1, dlimiter=' '):  # Returns angles from csv file for any
    act = ""                                                                 # specified lighthouse and number of poses
    i = 0                                                                    # count will determine how many to poses to
    location_x = np.zeros(32)                                                # skip
    location_y = np.zeros(32)
    xyflag = 0
    parseflag = 0
    all_points = []
    all_ids = []
    with open(file_name, newline='') as csvfile:  # Loads csv
        track = csv.reader(csvfile, delimiter=dlimiter, quotechar='|')
        for sweep in track:
            if sweep[0] != lighthouse:  # Discards any data that's not related to lighthouse
                continue
            if act != sweep[1]:  # Will wait until second column changes X-Y. Buggy.
                i += 1
                act = sweep[1]
            if i > steps*2:  # Will wait until X and Y are collected and count the number of poses parsed
                xyflag = 1
                parseflag += 1
                i = 1
            if xyflag == 1:  # Will store X-Y values in list with IDs in order
                sensor_ids = []
                sensor_points = []
                for k in range(0, 31):
                    if location_x[k] != 0 and location_y[k] != 0:  # Rejects data without a pair
                        x = (location_x[k])
                        y = (location_y[k])
                        point = [x, y]
                        sensor_points.append(point)
                        sensor_ids.append(k)
                xyflag = 0
                all_points.append(sensor_points)
                all_ids.append(sensor_ids)
                location_x = np.zeros(32)
                location_y = np.zeros(32)
            if parseflag == count:  # Will break when number of poses are parsed
                break
            if sweep[1] == 'X':
                location_x[int(sweep[4])] = int(sweep[6].strip())
            else:
                location_y[int(sweep[4])] = int(sweep[6].strip())
    return all_points, all_ids


def load_points_new(file_name, lighthouse="L", count=1, steps=1, dlimiter=' '):  # Returns angles from csv file for any
    act = ""                                                                 # specified lighthouse and number of poses
    i = 0                                                                    # count will determine how many to poses to
    location_x = np.zeros(32)                                                # skip
    location_y = np.zeros(32)
    pulse_x = np.zeros(32)
    pulse_y = np.zeros(32)
    xyflag = 0
    parseflag = 0
    all_points = []
    all_ids = []
    with open(file_name, newline='') as csvfile:  # Loads csv
        track = csv.reader(csvfile, delimiter=dlimiter, quotechar='|')
        for sweep in track:
            if sweep[0] != lighthouse:  # Discards any data that's not related to lighthouse
                continue
            if act != sweep[1]:  # Will wait until second column changes X-Y. Buggy.
                i += 1
                act = sweep[1]
            if i > steps*2:  # Will wait until X and Y are collected and count the number of poses parsed
                xyflag = 1
                parseflag += 1
                i = 1
            if xyflag == 1:  # Will store X-Y values in list with IDs in order
                sensor_ids = []
                sensor_points = []
                pulses = []
                for k in range(0, 31):
                    if location_x[k] != 0 and location_y[k] != 0:  # Rejects data without a pair
                        x = (location_x[k]/400000)*3.14159265359
                        y = (location_y[k]/400000)*3.14159265359
                        point = [x, y]
                        pulses.append(pulse_x[k] + pulse_y[k])
                        sensor_points.append(point)
                        sensor_ids.append(k)
                xyflag = 0
                sorted_pulse = sorted(range(len(pulses)), key=lambda i: pulses[i], reverse=True)[:4]
                valid_points = []
                valid_ids = []
                for k in sorted_pulse:
                    valid_points.append(sensor_points[k])
                    valid_ids.append(sensor_ids[k])
                all_points.append(valid_points)
                all_ids.append(valid_ids)
                location_x = np.zeros(32)
                location_y = np.zeros(32)
            if parseflag == count:  # Will break when number of poses are parsed
                break
            if sweep[1] == 'X':
                pulse_x[int(sweep[4])] = int(sweep[7].strip())
                location_x[int(sweep[4])] = int(sweep[6].strip()) #+ int(sweep[7])/2
            else:
                pulse_y[int(sweep[4])] = int(sweep[7].strip())
                location_y[int(sweep[4])] = int(sweep[6].strip()) #+ int(sweep[7])/2
    return all_points, all_ids