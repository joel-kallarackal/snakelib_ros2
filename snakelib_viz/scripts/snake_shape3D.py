import numpy as np
import matplotlib.pyplot as plt
# import mpl_toolkits.mplot3d  # just to register the '3d' projection
import matplotlib.animation as animation
import csv
import yaml

# -----------------------------
# DH Transform
# -----------------------------
def dh_transform(theta, d, a, alpha):
    """Single DH transform matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def forward_kinematics(dh_params, joint_angles):
    """Compute all joint positions in 3D"""
    T = np.eye(4)
    points = [(0,0,0)]
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        th = theta + joint_angles[i]   # revolute joint
        T = T @ dh_transform(th, d, a, alpha)
        points.append((T[0,3], T[1,3], T[2,3]))
    return points

params_path1 = '/home/joel/biorobotics_lab/snake_ws/src/snakelib_state/param/estimation_params.yaml'
        
with open(params_path1, "r") as file:
    data = yaml.safe_load(file)

dh_params = [[0, 0, 0.06577, 0]]
dh_params = np.concatenate((dh_params, data.get("dh")))

angles = []
with open("/home/joel/biorobotics_lab/snake_ws/src/snakelib_viz/data/2DMapping/Jungle_Gym_2D_Mapping_0.400_0.500.csv", newline="") as f:
    reader = csv.reader(f)
    for row in reader:
        angles.append(np.concatenate(([0],[float(val) for val in row])))
angles = np.array(angles)*4

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

tick_spacing = 0.1   # adjust to your robot size

ax.set_box_aspect([1,1,0.2])  # equal aspect ratio

ticks = np.arange(-1.1, 1.1+tick_spacing, tick_spacing)

ax.set_xticks(ticks)
ax.set_yticks(ticks)
ax.set_zticks(ticks)


# Auto-scale axes based on link lengths
total_length = sum([p[2] for p in dh_params]) + sum([p[1] for p in dh_params])
ax.set_xlim(-0.3, total_length)
ax.set_ylim(-total_length, total_length)
ax.set_zlim(-0.1, 0.3)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

line, = ax.plot([], [], [], "o-", lw=3)

final_x, final_y, final_z = [float(i) for i in forward_kinematics(dh_params, angles[-1])[-1]]
print(f"x : {final_x}\ny : {final_y}\nz : {final_z}")
def update(frame):
    pts = forward_kinematics(dh_params, angles[frame])
    xs, ys, zs = zip(*pts)
    line.set_data(xs, ys)
    line.set_3d_properties(zs)
    return line,



ani = animation.FuncAnimation(fig, update, frames=len(angles), blit=True, interval=100)
plt.show()
