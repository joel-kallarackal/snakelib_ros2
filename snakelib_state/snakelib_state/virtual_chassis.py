import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import yaml
import os
from ament_index_python.packages import get_package_share_directory

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import copy

import threading

class VirtualChassisVisualizer(Node):
    def __init__(self):
        super().__init__('virtual_chassis_node')
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/snake/joint_states',
            self.joint_state_cb,
            10 
        )
        self.joint_state_sub
    
        params_path1 = os.path.join(get_package_share_directory('snakelib_state'), 'param', 'estimation_params.yaml')
        
        with open(params_path1, "r") as file:
            self.data = yaml.safe_load(file)

        visualization_enabled = self.data.get("visualization")
        self.dh = self.data.get("dh")
        self.l = self.data.get("l")

        self.current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        if visualization_enabled:
            self.initialised = False
            ###
            # fig = plt.figure()
            # self.ax = fig.add_subplot(111, projection='3d')
            # ani = FuncAnimation(fig, self.update_plot, interval=100)
            # plt.tight_layout()
            # # plt.show()
            # threading.Thread(target=plt.show, daemon=True).start()
            ###
            plt.ion()  # Interactive mode on
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.timer = self.create_timer(0.1, self.update_plot)

    def joint_state_cb(self, msg):
        # self.get_logger().info("updating joint angles....")
        self.current_joint_angles = list(msg.position)

    def compute_com(self, T_list, l):
        coms = []
        for i in range(0, len(T_list)):
            T_link = T_list[i]
            # Local COM position in link frame
            com_local = np.array([-l/2, 0, 0, 1])
            # Transform to world frame
            com_world = T_link @ com_local
            coms.append(com_world[:3])
        coms = np.array(coms)
        overall_com = np.mean(coms, axis=0)  # Assuming uniform mass
        return coms, overall_com
    
    def T(self, theta, d, a, alpha):
        """
        Returns transformation matrix from k to k-1
        """
        c, s = np.cos(theta), np.sin(theta)
        c_al, s_al = np.cos(alpha), np.sin(alpha)

        return np.array([[c, -c_al*s, s_al*s, a*c],
                        [s, c_al*c, -s_al*c, a*s],
                        [0, s_al, c_al, d],
                        [0, 0, 0, 1]])
    
    def compute_fk(self, dh, joint_angles):
        T0 = np.eye(4)
        Ts = []
        Ts.append(T0)
        for i in range(len(dh)):
            Ts.append(Ts[i]@self.T(joint_angles[i], dh[i][1], dh[i][2], dh[i][3]))

        return Ts
    
    def update_plot(self):
        self.ax.clear()
        T_list = self.compute_fk(self.dh, self.current_joint_angles) # Step 1
        coms, overall_com = self.compute_com(T_list, self.l) # Step 2

        # Data Matrix
        P = np.array(coms) - overall_com # Step 3
        U, S, VT = np.linalg.svd(P) # Step 4
        V = VT.T

        # Virtual Chassis with respect to the initial body frame
        T_vc = np.eye(4)
        T_vc[:3, :3] = V
        T_vc[:3, 3] = overall_com

        # Enforce no sign flips
        v1 = T_vc[:3, 0]
        v2 = T_vc[:3, 1]
        if self.initialised==False:
            self.v1_before = copy.copy(v1)
            self.v2_before = copy.copy(v2)
            self.initialised = True

        if np.dot(v1, self.v1_before) < 0:
            v1 = -v1
        
        self.v1_before = copy.copy(v1)

        if np.dot(v2, self.v2_before) < 0:
            v2 = -v2
        
        self.v2_before = copy.copy(v2)
        
        T_vc[:3, 0] = v1
        T_vc[:3, 1] = v2

        # Make the virtual chassis always right handed
        T_vc[:3, 2] = np.cross(T_vc[:3, 0], T_vc[:3, 1])

        # Frames with respect to virtual chassis
        T_new = []
        T_vc_inv = np.linalg.inv(T_vc)
        for t in T_list:
            T_new.append(T_vc_inv @ t)
        
        coms_vc, overall_com_vc = self.compute_com(T_new, self.l) # Step 2
        
        self.ax.scatter(coms_vc[:, 0], coms_vc[:, 1], coms_vc[:, 2], c='m', label='Link COMs')
        self.ax.scatter(*overall_com_vc, c='cyan', s=100, label='Virtual Chassis')
        self.ax.text(*overall_com_vc, '', color='cyan', fontsize=10)
        plt.legend()

        T_vc_in_vc = T_vc_inv @ T_vc # Virtual chassis frame in virtual chassis frame

        self.ax.quiver(*T_vc_in_vc[:3, 3], *T_vc_in_vc[:3, 0], color='r')  # X axis
        self.ax.quiver(*T_vc_in_vc[:3, 3], *T_vc_in_vc[:3, 1], color='g')  # Y axis
        self.ax.quiver(*T_vc_in_vc[:3, 3], *T_vc_in_vc[:3, 2], color='b')  # Z axis
        # ax.text(*T_vc_in_vc[:3, 3], '', fontsize=10)

        origins_vc = np.array([T[:3, 3] for T in T_new])
        head_vc = (T_vc_inv @ T_list[0]) @ [-self.l, 0, 0, 1] # Convert head tip to VC frame
        origins_vc = np.concatenate(([head_vc[:3]], origins_vc))

        self.ax.text(*head_vc[:3], 'Head', fontsize=10)

        # Plot links
        self.ax.plot(origins_vc[:, 0], origins_vc[:, 1], origins_vc[:, 2], 'k-o', linewidth=2)

        # Plot frames
        for i, T in enumerate(T_new):
            origin = T[:3, 3]
            self.ax.quiver(*origin, *T[:3, 0]*0.1, color='r')  # X axis
            self.ax.quiver(*origin, *T[:3, 1]*0.1, color='g')  # Y axis
            self.ax.quiver(*origin, *T[:3, 2]*0.1, color='b')  # Z axis
            if i!=16:
                self.ax.text(*origin, f'J{i}', fontsize=10)
            else:
                self.ax.text(*origin, 'Tail', fontsize=10)

        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Snake Shape With Respect To Virtual Chassis')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = VirtualChassisVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
