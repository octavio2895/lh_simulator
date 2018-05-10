from tools import *
from poser_tools import calc_pose, rotmat2quat
from sim_tools import cartesian2lighthouse
import time
from matplotlib import pyplot as plt
import cv2
import os

dirname="poses"
model_pointsa = open_config_old("data/config.json", j_file=True)
poses, ids = new_parser("data/data_for_albino", count=5000, lighthouse="L")
poses_raw, ids_raw = new_parser_raw("data/data_for_albino", count=5000, lighthouse="L")
print(len(poses))

blank = cv2.imread("blank.jpg")
rms = []
compute_times = []
guess_z = array([3, 3, 3, 3])
for pose in range(len(poses)):
    filename = "pose_" + str(pose) + ".jpg"
    print(filename)
    im = blank.copy()
    start_time = time.time()
    if len(poses[pose]) < 4:
        continue
    rotation_matrix, translation_vector, guess_z = calc_pose(model_pointsa[0], poses[pose], ids[pose], guess=guess_z, verbose=True)
    end_time = time.time()

    model_points = model_pointsa[0]

    A_2 = []
    for i in range(len(poses[pose])):
        A_2.append([model_points[ids[pose][i]][0], model_points[ids[pose][i]][1], model_points[ids[pose][i]][2]])

    A_2 = np.matrix(model_points)
    C = (rotation_matrix * A_2.T)
    C = C.T
    translation_vector = translation_vector.T

    for i in range(len(C)):
        C[i] = C[i] + translation_vector[0]

    reprojection = zeros((len(C), 3))
    for i in range(len(C)):
        reprojection[i] = cartesian2lighthouse(np.squeeze(np.asarray(C[i])))
        reprojection[i][1] = 200000 - reprojection[i][1]/np.pi * 400000
        reprojection[i][2] = 200000 - reprojection[i][2]/np.pi * 400000



    reprojection_2d_x = zeros((len(reprojection) - len(ids[pose])))
    reprojection_2d_y = zeros((len(reprojection) - len(ids[pose])))
    sensor_hit_x = zeros((len(ids[pose])))
    sensor_hit_y = zeros((len(ids[pose])))

    for i in range(len(ids[pose])):
        sensor_hit_x[i] = reprojection[ids[pose]][i][1]
        sensor_hit_y[i] = reprojection[ids[pose]][i][2]
    j = 0
    for i in range(len(reprojection)):
        if i in ids[pose]:
            continue
        else:
            reprojection_2d_x[j] = reprojection[i][1]
            reprojection_2d_y[j] = reprojection[i][2]
            j += 1

    poses_raw_fixed_x = zeros((len(poses_raw[pose])))
    poses_raw_fixed_y = zeros((len(poses_raw[pose])))

    for i in range(0, len(poses_raw[pose])):
        poses_raw_fixed_x[i] = poses_raw[pose][i][0]
        poses_raw_fixed_y[i] = poses_raw[pose][i][1]



    sum = 0
    for i in range(len(ids[pose])):
        sum += (sensor_hit_x[i] - poses_raw_fixed_x[i])**2 + (sensor_hit_y[i] - poses_raw_fixed_y[i])**2
    rmse = sqrt(sum/len(ids[pose]))
    print("RMS error:")
    print(rmse)
    rms.append(rmse)

    print("Pose compute time (s):")
    print(end_time-start_time)
    compute_times.append(end_time-start_time)

    for p in range(len(poses_raw_fixed_y)):
        cv2.circle(im, (int(poses_raw_fixed_x[p] / 200), int(poses_raw_fixed_y[p] / 200)), 1, (0, 255, 0), -1)

    for p in range(len(sensor_hit_y)):
        cv2.circle(im, (int(sensor_hit_x[p] / 200), int(sensor_hit_y[p] / 200)), 1, (0, 0, 255), -1)

    for p in range(len(reprojection_2d_y)):
        cv2.circle(im, (int(reprojection_2d_x[p] / 200), int(reprojection_2d_y[p] / 200)), 1, (255, 0, 0), -1)

    im = im[650:1350, 650:1350]
    im = cv2.resize(im, (480, 480))
    cv2.imwrite(os.path.join(dirname, filename), im)
    # plt.scatter(poses_raw_fixed_x, poses_raw_fixed_y, s=10, c='g', marker="s", label='Raw Data')
    # plt.scatter(sensor_hit_x, sensor_hit_y, s=10, c='b', marker="s", label='Sensor Hit')
    # plt.scatter(reprojection_2d_x, reprojection_2d_y, s=10, c='r', marker="s", label='Reprojection')
    # plt.legend(loc='upper left')
    # plt.title('Reprojection test')
    # #plt.axis((200000, 220000, 115000, 135000))
    # plt.axis((0, 400000, 0, 400000))
    # plt.show()
print("Average RMS error:", mean(rms))
print("Average compute time per frame", mean(compute_times))