from tools import *
from poser_tools import calc_pose, rotmat2quat
from sim_tools import cartesian2lighthouse
import time
from matplotlib import pyplot as plt

model_points = open_config_old("data/WM0_points.csv", j_file=False)
poses, ids = load_points("data/output.csv", count=1, lighthouse="L")
poses_raw, ids_raw = load_points_raw("data/output.csv", count=1, lighthouse="L")

for i in range(len(poses)):
    start_time = time.time()
    rotation_matrix, translation_vector, z = calc_pose(model_points[0], poses[i], ids[i], verbose=True)
    end_time = time.time()
ids = ids[0]
model_points = model_points[0]

A_2 = []
for i in range(len(poses[0])):
    A_2.append([model_points[ids[i]][0], model_points[ids[i]][1], model_points[ids[i]][2]])

A_2 = np.matrix(model_points)
C = (rotation_matrix * A_2.T)
C = C.T
translation_vector = translation_vector.T

for i in range(len(C)):
    C[i] = C[i] + translation_vector[0]


reprojection = zeros((len(C), 3))
for i in range(len(C)):
    reprojection[i] = cartesian2lighthouse(np.squeeze(np.asarray(C[i])))
    reprojection[i][1] = reprojection[i][1]/np.pi *400000
    reprojection[i][2] = reprojection[i][2]/np.pi *400000

reprojection_2d_x = zeros((len(reprojection) - len(ids)))
reprojection_2d_y = zeros((len(reprojection) - len(ids)))
sensor_hit_x = zeros((len(ids)))
sensor_hit_y = zeros((len(ids)))

for i in range(len(ids)):
    sensor_hit_x[i] = 200000 - reprojection[ids[i]][1]
    sensor_hit_y[i] = -reprojection[ids[i]][2]

j = 0
for i in range(len(reprojection)):
    if i in ids:
        continue
    else:
        reprojection_2d_x[j] =200000 - reprojection[i][1]
        reprojection_2d_y[j] = -reprojection[i][2]
        j += 1

poses_raw_fixed_x = zeros((len(poses_raw[0])))
poses_raw_fixed_y = zeros((len(poses_raw[0])))

for i in range(0, len(poses_raw[0])):
    poses_raw_fixed_x[i] = poses_raw[0][i][0]
    poses_raw_fixed_y[i] = poses_raw[0][i][1]

sum = 0
for i in range(len(ids)):
    sum += (sensor_hit_x[i] - poses_raw_fixed_x[i])**2 + (sensor_hit_y[i] - poses_raw_fixed_y[i])**2
rmse = sqrt(sum/len(ids))
print("RMS error:")
print(rmse)


plt.scatter(poses_raw_fixed_x, poses_raw_fixed_y, s=10, c='g', marker="s", label='Raw Data')
plt.scatter(sensor_hit_x, sensor_hit_y, s=10, c='b', marker="s", label='Sensor Hit')
plt.scatter(reprojection_2d_x, reprojection_2d_y, s=10, c='r', marker="s", label='Reprojection')
plt.legend(loc='upper left')
plt.title('Reprojection test')
plt.axis((200000, 220000, 115000, 135000))
#plt.axis((0, 400000, 0, 400000))
plt.show()
print("Pose compute time (s):")
print(end_time-start_time)
