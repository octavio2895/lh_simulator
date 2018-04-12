from tools import *
from poser_tools import calc_pose
from sim_tools import cartesian2lighthouse
import time

model_points = open_config_old("WM0_points.csv", j_file=False)
poses, ids = load_points("output.csv", count=1, lighthouse="L")

for i in range(len(poses)):
    rotation_matrix, translation_vector, z = calc_pose(model_points[0], poses[i], ids[i], verbose=True)

ids = ids[0]
model_points = model_points[0]
print(ids)
A_2 = [[model_points[ids[0]][0], model_points[ids[0]][1], model_points[ids[0]][2]],
       [model_points[ids[1]][0], model_points[ids[1]][1], model_points[ids[1]][2]],
       [model_points[ids[2]][0], model_points[ids[2]][1], model_points[ids[2]][2]],
       [model_points[ids[3]][0], model_points[ids[3]][1], model_points[ids[3]][2]]]
A_2 = np.matrix(A_2)
C = (rotation_matrix * A_2.T)
C = C.T
print(C)
