import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

# Load CSV file
df = pd.read_csv('/home/joel/biorobotics_lab/snake_ws/src/snakelib_debug/logs/joint_commands_lateral_undulation.csv', header=None)
df2 = pd.read_csv('/home/joel/biorobotics_lab/snake_ws/src/snakelib_debug/logs/joint_states_lateral_undulation.csv', header=None)

time = df.iloc[:, 16].to_numpy()  
time2 = df2.iloc[:, 16].to_numpy()  
num_plots = df.shape[1] - 1  
ncols = 4
nrows = math.ceil(num_plots / ncols)

fig, axes = plt.subplots(nrows, ncols, figsize=(15, nrows * 3), sharex=True)
axes = axes.flatten()  

for i in range(num_plots):
    col_data = df.iloc[:, i].to_numpy()*180/np.pi
    col_data2 = df2.iloc[:, i].to_numpy()*180/np.pi
    axes[i].plot(time, col_data, marker='', linestyle='-', label="commanded")
    axes[i].legend("")
    axes[i].plot(time2, col_data2, marker='', linestyle='-', label="achieved")
    axes[i].set_title(f'Joint {i+1} vs Time')
    axes[i].set_ylabel("Angle (degrees)")
    axes[i].set_xlabel("Time (seconds)")
    axes[i].legend(loc='best')
    axes[i].grid(True)


fig.suptitle('Lateral Undulation', fontsize=16)

for j in range(num_plots, len(axes)):
    axes[j].axis('off')

fig.tight_layout()
plt.xlabel('Time')
plt.show()
