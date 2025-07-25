from typing import Dict
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

JOINT_TOLERANCE = 0.15  # tolerance for joint angles
MIN_JOINT_ANGLE = -np.pi / 2.0 + JOINT_TOLERANCE
MAX_JOINT_ANGLE = np.pi / 2.0 - JOINT_TOLERANCE
dt = 0.025

def ik_two_modules(r, p, y):
    theta1, theta2 = 0,0
    if p != np.pi/2 and p != -np.pi/2:
        theta1 = -p
        theta2 = y
    else:
        theta1 = -p
        if theta1 == np.pi/2:
            theta2 = 90 - r
        else:
            theta2 = -(90 - r)
        
    return np.array([theta1, theta2])

def ik_three_modules(r, p, y):
    theta1 = np.arctan2(np.tan(p), np.sin(r))
    theta3 = np.arctan2(np.cos(r)*np.sin(p)*np.sin(y) - np.sin(r)*np.cos(y), np.cos(r)*np.sin(p)*np.cos(y) + np.sin(r)*np.cos(y))
    theta2 = np.arctan2((np.cos(r)*np.sin(p)*np.sin(y) - np.sin(r)*np.cos(y))/np.sin(theta3), -np.cos(r)*np.cos(p))

    return np.array([theta1, theta2, theta3])

def head_look_ik(self, t: float, current_angles: np.ndarray, robot, params: Dict = None) -> np.ndarray:
    """Controls the end-effector (snake head) in cartesian space using inverse jacobian approach.

    Args:
        t: current robot time
        robot: robot model object to get the jacobian
        current angles: latest joint angles readings
        params: update (if any) in parameters
    """

    # params_path1 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')
    # params_path2 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'launch_params.yaml')
        
    # with open(params_path2, "r") as file:
    #     snake_type = yaml.safe_load(file).get("snake_type")    

    # with open(params_path1, "r") as file:
    #     data = yaml.safe_load(file)
    
    # ik_params = data.get("command_manager").get("ros__parameters").get(snake_type).get("gait_params").get("head_look_ik")

    # n_headlook_modules = ik_params.get("n_modules")
    # self.current_gait = "head_look_ik"

    # Update the current parameters if params is not empty
    params = {} if params is None else params
    params = self.update_params(self.default_gait_params.get(self.current_gait, {}), params)

    head_acc_x, head_acc_y = params["head_acc_x"], params["head_acc_y"]

    n_headlook_modules=2

    x = params["head_x"]
    y = params["head_y"]
    z = params["head_z"]
    roll = params["head_roll"]
    pitch = params["head_pitch"]
    yaw = params["head_yaw"]

    if n_headlook_modules==2:
        yaw_sign, pitch_sign = -1,-1
        swap_yp = False
        if abs(head_acc_x)>abs(head_acc_y):
            if head_acc_x>0:
                yaw_sign*=-1
                pitch_sign*=-1
        else:
            # swap yaw and pitch in joystick
            swap_yp = True
            if head_acc_y>0:
                pitch_sign*=-1
            else:
                yaw_sign*=-1
        
        yaw_dt = params["yaw"]*dt*yaw_sign
        pitch_dt = params["pitch"]*dt*pitch_sign

        current_yaw = current_angles[1] 
        current_pitch = -current_angles[0]
        if swap_yp:
            target_yaw = current_yaw+pitch_dt
            target_pitch = current_pitch+yaw_dt
        else:
            target_yaw = current_yaw+yaw_dt
            target_pitch = current_pitch+pitch_dt

        theta1, theta2 = ik_two_modules(r=90, p=target_pitch, y=target_yaw)
        theta1, theta2 = np.clip([theta1, theta2], MIN_JOINT_ANGLE, MAX_JOINT_ANGLE)
        target_angles = np.concatenate(([theta1, theta2], current_angles[2:]))

        return target_angles
    elif n_headlook_modules==3:
        yaw_sign, pitch_sign, roll_sign = -1, -1, -1
        swap_yp = False
        if abs(head_acc_x)>abs(head_acc_y):
            if head_acc_x>0:
                yaw_sign*=-1
                pitch_sign*=-1
        else:
            # swap yaw and pitch in joystick
            swap_yp = True
            if head_acc_y>0:
                pitch_sign*=-1
            else:
                yaw_sign*=-1

        yaw_dt = params["yaw"]*dt*yaw_sign
        pitch_dt = params["pitch"]*dt*pitch_sign
        roll_dt = params["roll"]*dt*pitch_sign

        if swap_yp:
            target_roll = roll + roll_dt
            target_pitch = pitch + yaw_dt
            target_yaw = yaw + pitch_dt
        else:
            target_roll = roll + roll_dt
            target_pitch = pitch + pitch_dt
            target_yaw = yaw + yaw_dt

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        q1, q2, q3, q4 = quaternion_from_euler(target_roll, target_pitch, target_yaw)

        pose.orientation.x = q1
        pose.orientation.y = q2
        pose.orientation.z = q3
        pose.orientation.w = q4

        # Need to change this, can't keep creating a new node.
        node = Node("head_state_node")
        head_state_pub = node.create_publisher(Pose, 'snake/head_state', 100)
        head_state_pub.publish(pose)
        node.destroy_node()

        theta1, theta2, theta3 = ik_three_modules(target_roll, target_pitch, target_yaw)
        theta1, theta2, theta3 = np.clip([theta1, theta2, theta3], MIN_JOINT_ANGLE, MAX_JOINT_ANGLE)
        target_angles = np.concatenate(([theta1, theta2, theta3], current_angles[3:]))

        return target_angles


##################
# Old code
# from typing import Dict

# import numpy as np
# def head_look_ik(self, t: float, current_angles: np.ndarray, robot, params: Dict = None) -> np.ndarray:
#     """Controls the end-effector (snake head) in cartesian space using inverse jacobian approach.

#     Args:
#         t: current robot time
#         robot: robot model object to get the jacobian
#         current angles: latest joint angles readings
#         params: update (if any) in parameters
#     """

#     n_headlook_modules = 6  # number of modules in the headlook group.

#     self.current_gait = "head_look"
#     # Update the current parameters if params is not empty
#     params = {} if params is None else params

#     params = self.update_params(self.default_gait_params.get(self.current_gait, {}), params)

#     yaw, y, z = params["x_state"], params["y_state"], params["z_state"]
#     roll, pitch, x = params["roll"], params["pitch"], params["yaw"]

#     # delta = np.array([x, y, z, roll, pitch, yaw]) * 0.1
#     delta = np.array([roll, pitch, yaw, x, y, z]) * np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
#     # delta = np.array([x, y, z, roll, pitch, yaw]) * params['delta_scale']
#     head_look_group_joint_angles = robot.update_rect(delta, clip="norm")[:n_headlook_modules]
#     target_angles = np.concatenate((head_look_group_joint_angles, current_angles[n_headlook_modules:]))

#     return target_angles
