import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import yaml
import os
from ament_index_python.packages import get_package_share_directory

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import copy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tf_transformations import quaternion_from_matrix



class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator_node')
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/snake/joint_states',
            self.joint_state_cb,
            10 
        )
        self.joint_state_sub

        self.jam_feedback_sub = self.create_subscription(
            String,
            '/snake/jam_feedback',
            self.jam_feedback_cb,
            10 
        )
        self.jam_feedback_sub

        self.frame_pub = self.create_publisher(Marker, 'visualization_marker', 10)
    
        params_path1 = os.path.join(get_package_share_directory('snakelib_state'), 'param', 'estimation_params.yaml')
        
        with open(params_path1, "r") as file:
            self.data = yaml.safe_load(file)

        self.dh = self.data.get("dh")
        self.head_len = self.data.get("head_len")
        self.module_lens = [self.dh[i][2] for i in range(len(self.dh))]
        self.module_lens.append(self.head_len)

        self.l = self.data.get("l")

        self.current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.jam_initialised = False

        self.module_mapping = {"head": 16, "tail": 1, "tail_tip": 0}

        self.head_tip_in_head_frame = [self.head_len, 0, 0, 1]

    def joint_state_cb(self, msg):
        self.current_joint_angles = list(msg.position)[::-1]

        if self.jam_initialised:
            jam_module = self.module_mapping[self.jam_type]
            T_frames_in_head = self.compute_fk(self.dh, self.current_joint_angles)
            T_body_in_world = self.body_frame_in_world
            T_body_in_head = T_frames_in_head[self.module_mapping[self.jam_type]]
            T_frames_in_body = self.convert_frames(T_frames_in_head, T_body_in_head)
            self.T_frames_in_world = [T_body_in_world @ T_frames_in_body[i] for i in range(len(T_frames_in_body))]
 
            if self.jam_type=="tail" or self.jam_type=="tail_tip":
                # self.unjammed_tip_location_in_world =  ((T_body_in_world @ (np.linalg.inv(T_body_in_head) @ T_frames_in_head[16])) @ self.head_tip_in_head_frame)[:3]
                # self.unjammed_tip_location_in_world =  ((T_body_in_world @ (np.linalg.inv(T_body_in_head) @ T_frames_in_head[16])))[:3, 3]
                self.unjammed_tip_location_in_world = (self.T_frames_in_world[self.module_mapping["head"]] @ self.head_tip_in_head_frame)[:3]
            elif self.jam_type=="head":
                self.unjammed_tip_location_in_world = self.T_frames_in_world[self.module_mapping["tail_tip"]][:3, 3]
            else:
                self.get_logger().error(f"{self.jam_type} is an unrecognized jam type.")

            x, y, z = self.unjammed_tip_location_in_world
            self.get_logger().info(f"World Frame >> X : {str(x)}, Y : {str(y)}, Z : {str(z)}")
            self.get_logger().info(f"Distance : {np.sqrt(x**2+y**2+z**2)}")

            matrices = [frame for frame in self.T_frames_in_world]
            matrices.append(np.eye(4)) # World Frame
            matrices.append(T_body_in_world) # Body Frame
            for i, mat in enumerate(matrices):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "frames"
                marker.id = i
                marker.type = Marker.ARROW  # AXES not available in some versions
                marker.action = Marker.ADD

                marker.pose.position.x = mat[0, 3]
                marker.pose.position.y = mat[1, 3]
                marker.pose.position.z = mat[2, 3]
                quat = quaternion_from_matrix(mat)
                marker.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

                if i<len(matrices)-2:
                    marker.scale.x = self.module_lens[i]  # shaft length
                else:
                    marker.scale.x = 0.05 # For world and body frame
                
                marker.scale.y = 0.02  # shaft diameter
                marker.scale.z = 0.02  # head diameter

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                if i==(len(matrices)-2):
                    # World Frame
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                if i==(len(matrices)-1):
                    # Body Frame
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0

                self.frame_pub.publish(marker)


    def jam_feedback_cb(self, msg):
        # change body frame on signal
        self.jam_type = msg.data
        self.get_logger().info(f"Jam Signal Received : {self.jam_type}")

        # Set world frame on first successful jam
        if self.jam_initialised==False:
            self.T_vc_in_head = self.get_virtual_chassis(self.dh, self.current_joint_angles, False)
            self.T_vc_in_head[:3,3] = [0, 0 ,0]
            self.jam_initialised=True
            self.get_logger().info("State Estimation Initialised")
            T_frames_in_head = self.compute_fk(self.dh, self.current_joint_angles)
            self.body_frame_in_world = np.linalg.inv(self.T_vc_in_head) @ T_frames_in_head[self.module_mapping[self.jam_type]] 
        else:
            T_frames_in_head = self.compute_fk(self.dh, self.current_joint_angles)
            self.body_frame_in_world = self.T_frames_in_world[self.module_mapping[self.jam_type]]

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
    
    def get_virtual_chassis(self, dh, joint_angles, initialised=False, T_vc_before=None):
        T_list = self.compute_fk(dh, joint_angles)
        coms, overall_com = self.compute_com(T_list, self.l) 

        # Data Matrix
        P = np.array(coms) - overall_com
        U, S, VT = np.linalg.svd(P)
        V = VT.T

        # Virtual Chassis with respect to the initial body frame
        T_vc = np.eye(4)
        T_vc[:3, :3] = V
        T_vc[:3, 3] = overall_com

        # Enforce no sign flips
        v1 = T_vc[:3, 0]
        v2 = T_vc[:3, 1]
        if initialised==False:
            self.v1_before = copy.copy(v1)
            self.v2_before = copy.copy(v2)
        else:
            self.v1_before = T_vc_before[:3, 0]
            self.v2_before = T_vc_before[:3, 1]

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

        return T_vc
    
    def convert_frames(self, T_list, T_target=None, frame_number=-1):
        T_list_new = []

        if frame_number != -1:
            T_target = T_list[frame_number]
            T_target_inv = np.linalg.inv(T_target)

            for i in range(len(T_list)):
                T_list_new.append(T_target_inv @ T_list[i])
        else: 
            T_target_inv = np.linalg.inv(T_target)

            for i in range(len(T_list)):
                T_list_new.append(T_target_inv @ T_list[i])
        

        return T_list_new
    

def main():
    rclpy.init()
    state_estimation_node = StateEstimator()

    try:
        rclpy.spin(state_estimation_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_estimation_node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
