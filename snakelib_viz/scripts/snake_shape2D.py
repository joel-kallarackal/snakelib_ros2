import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import yaml

# -----------------------------
# Forward kinematics with DH
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
    """Compute all joint positions in 2D (x,y)"""
    T = np.eye(4)
    points = [(0,0)]
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        th = theta + joint_angles[i]   # add joint variable
        T = T @ dh_transform(th, d, a, alpha)
        points.append((T[0,3], T[1,3]))  # x,y position
    return points

params_path1 = '/home/joel/biorobotics_lab/snake_ws/src/snakelib_state/param/estimation_params.yaml'
        
with open(params_path1, "r") as file:
    data = yaml.safe_load(file)

dh_params = data.get("dh")


# -----------------------------
# Load joint angles from CSV
# CSV format: each row = joint angles for one timestep
# -----------------------------
angles = []
# with open("joint_angles.csv", newline="") as f:
#     reader = csv.reader(f)
#     for row in reader:
#         angles.append([float(val) for val in row])
# angles = np.array(angles)
for i in range(100):
    angles.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 0, 0])


# -----------------------------
# Animate
# -----------------------------
fig, ax = plt.subplots()
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-sum(p[2] for p in dh_params)-0.5, sum(p[2] for p in dh_params)+0.5)
ax.set_ylim(-sum(p[2] for p in dh_params)-0.5, sum(p[2] for p in dh_params)+0.5)

line, = ax.plot([], [], "o-", lw=3)

def update(frame):
    pts = forward_kinematics(dh_params, angles[frame])
    xs, ys = zip(*pts)
    line.set_data(xs, ys)
    return line,

ani = animation.FuncAnimation(fig, update, frames=len(angles), blit=True, interval=200)
plt.show()
