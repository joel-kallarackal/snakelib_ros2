import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import pybullet as p
import pybullet_data
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.clock import Clock
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter

class PyBulletRosWrapper(Node):
    def __init__(self, **kwargs):
        super().__init__('pybullet_interface', **kwargs)

        # parameters should be declared while launching this node
        # Or
        # Declare them below
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('snake_type', "REU"),
        #         ('loop_rate', 100.0),
        #         ('urdf_spawn_pose', [0.0, 0.0, 0.0]),
        #         ('terrain_path', ""),
        #         ('robot_description', ""),
        #         ('tracking_cam', False),
        #         ('cam_angle', [2.0, 0.0, -45.0]),
        #         ('max_torque', 7),
        #         ('joint_control_mode', 0),
        #         ('save_movie', 0),
        #     ]
        # )
        self.declare_parameter('snake_type', Parameter.Type.STRING)
        self.declare_parameter('loop_rate', Parameter.Type.DOUBLE)
        self.declare_parameter('urdf_spawn_pose',Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('terrain_path', Parameter.Type.STRING)
        self.declare_parameter('robot_description', Parameter.Type.STRING)
        self.declare_parameter('tracking_cam', Parameter.Type.BOOL)
        self.declare_parameter('cam_angle', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_torque', Parameter.Type.DOUBLE)
        self.declare_parameter('joint_control_mode', Parameter.Type.INTEGER)
        self.declare_parameter('save_movie', Parameter.Type.INTEGER)


        # Load snake type and set urdf path
        self._snake_type = self.get_parameter('snake_type').value
        self.set_urdf_path()

        # Load parameters
        self.loop_rate = self.get_parameter('loop_rate').value
        self.urdf_spawn_pose = self.get_parameter('urdf_spawn_pose').value # x,y,z
        self.terrain_path = self.get_parameter('terrain_path').value
        if self.terrain_path=="":
            self.terrain_path=None
        

        self.tracking_cam = self.get_parameter('tracking_cam').value
        self.cam_angle = self.get_parameter('cam_angle').value # dist, yaw, pitch

        self.max_torque = self.get_parameter('max_torque').value # Nm
        self.joint_control_mode = self.get_parameter('joint_control_mode').value # 0: position, 1: velocity, 3: torque control
        self.joint_control_mode_mapping = {
            0: p.POSITION_CONTROL,
            1: p.VELOCITY_CONTROL,
            2: p.TORQUE_CONTROL,
        }
        self.save_movie = self.get_parameter('save_movie').value # whether to save a movie

        self.create_service(Empty, 'reset_simulation', self.handle_reset_simulation)
        self.create_service(Empty, 'pause_physics', self.handle_pause_physics)
        self.create_service(Empty, 'unpause_physics', self.handle_unpause_physics)

        # start pybullet
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setTimeStep(1.0 / self.loop_rate)

        self.robot_id = 0
        self.num_dofs = 0
        self.pause_simulation = False

        # these need to be defined everytime we reset the sim
        p.setGravity(0, 0, -9.81)
        self.load_terrain()
        self.load_urdf()

        # placeholder for storing joint command data
        self.command_positions = [0.0] * self.num_dofs
        self.command_velocities = [0.0] * self.num_dofs

        # initialize effort as max torque for position, velocity control:
        # setting it to zero causes the robot to move when not giving any commands
        if self.joint_control_mode != 2:
            self.command_efforts = [self.max_torque] * self.num_dofs
        else:
            self.command_efforts = [0.0] * self.num_dofs

        # TODO: parse max torques from URDF

        # start joint command subscriber
        self.joint_command_topic = "/snake/joint_commands"
        self.joint_command_sub = self.create_subscription(
            JointState,
            self.joint_command_topic,
            self.joint_command_callback,
            10)
        self.joint_command_sub  # prevent unused variable warning
        self.warn = False

        # initialize publishers
        self.joint_state_pub = self.create_publisher(JointState, '/snake/joint_states', 100)

        # TODO: implement a way to simulate IMU readings OR publish orientations directly
        # imu_pub = rospy.Publisher('/snake/joint_states', , queue_size=100)
        self.joint_state = JointState()

    def start(self):
        self.joint_state.name = list(self.revolute_joint_idx.values())

        rate = self.create_rate(self.loop_rate)

        if self.save_movie:
            self.start_video_log()

        while rclpy.ok():
            self.send_joint_commands()
            
            time_now = self.get_clock().now().to_msg()
            self.joint_state.header.stamp = time_now
            (
                self.joint_state.position,
                self.joint_state.velocity,
                self.joint_state.effort,
            ) = self.get_feedback()
            
            self.joint_state_pub.publish(self.joint_state)
            
            # TODO: IMU reading publisher
            if self.tracking_cam:
                self.update_cam_view(self.cam_angle)
            
            if not self.pause_simulation:
                p.stepSimulation()
            
            rclpy.spin_once(self)
            rate.sleep()

        # Save movie if necessary
        if self.save_movie:
            self.stop_video_log()

    def enable_joint_torque_sensor(self):
        for i in self.joint_idx:
            p.enableJointForceTorqueSensor(self.robot_id, i, True)

    def get_feedback(self):
        # Get joint state (position, velocity and output torques).

        position = np.array([0.0] * self.num_dofs)
        velocity = np.array([0.0] * self.num_dofs)
        effort = np.array([0.0] * self.num_dofs)

        for idx, motor_no in enumerate(list(self.joint_idx)):
            joint_p, joint_v, _, output_torque = p.getJointState(self.robot_id, motor_no)
            position[idx] = joint_p
            velocity[idx] = joint_v
            effort[idx] = output_torque

        return (
            position.tolist(),
            velocity.tolist(),
            effort.tolist(),
        )

    def send_joint_commands(self):
        p.setJointMotorControlArray(
            self.robot_id,
            self.joint_idx,
            self.joint_control_mode_mapping[self.joint_control_mode],
            targetPositions=self.command_positions,
            targetVelocities=self.command_velocities,
            forces=self.command_efforts,
        )

    def joint_command_callback(self, msg):

        # This method parses the joint commands based on the control mode specified by the ROS param joint_control_mode.
        # Refer pybullet's setJointMotorControlArray method definition for more information about this implementation.

        """There is currently a discrepency between URDF joint names and the module names.
        This causes PyBullet to crash when we provide the module names. We do not want to
        require the URDF to match the module names, because the modules often get replaced
        and this would make this cumbersome. Instead, we should write a node that converts
        module names into URDF names and provides this to PyBullet. For now, we remove the
        names to make integration possible until we can implement this node.
        """
        msg.name = []

        if self.joint_control_mode == 0:
            # POSITION_CONTROL

            # make a dictionary of joint_name: command and send it to sort_joint_commands() for sorting
            self.command_positions = self.sort_joint_commands(msg.name, msg.position)

            # send sorted velocity commands if received, else send defaults.
            self.command_velocities = (
                self.sort_joint_commands(msg.name, msg.velocity) if len(msg.velocity) == self.num_dofs else [0.0] * self.num_dofs
            )

            # We may fill the joint state with NaNs for the HEBI actuators, but PyBullet cannot take NaNs
            self.command_velocities = np.nan_to_num(self.command_velocities)

            # send sorted effort commands (max torques) if received, else send defaults.
            # In position contorl mode, forces are the maximum motor force used to reach target position
            self.command_efforts = (
                self.sort_joint_commands(msg.name, msg.effort) if len(msg.effort) == self.num_dofs else [self.max_torque] * self.num_dofs
            )

        elif self.joint_control_mode == 1:
            # VELOCITY_CONTROL

            # pybullet implementation of velocity control does not use position commands, so we set them to defaults.
            self.command_positions = [0.0] * self.num_dofs

            # send sorted velocity commands
            self.command_velocities = self.sort_joint_commands(msg.name, msg.velocity)

            # send sorted effort commands (max torques) if received, else send defaults.
            self.command_efforts = (
                self.sort_joint_commands(msg.name, msg.effort) if len(msg.effort) == self.num_dofs else [self.max_torque] * self.num_dofs
            )

        elif self.joint_control_mode == 2:
            # TORQUE_CONTROL

            # pybullet implementation of torque control does not use position and velocity commands,
            # so we set them to defaults.
            self.command_positions = [0.0] * self.num_dofs
            self.command_velocities = [0.0] * self.num_dofs

            # send sorted effort commands.
            self.command_efforts = self.sort_joint_commands(msg.name, msg.effort)

    def sort_joint_commands(self, name, command):

        # If joint names list is not specified/incomplete, warn user and return unsorted commands.
        if len(name) < self.num_dofs:
            if not self.warn:
                self.get_logger().warn(
                    f"Message from topic {self.joint_command_topic} didn't contain a joint name list. Using unsorted commands.",
                )
                self.warn = True
            return command
        else:
            joint_commands_with_names = dict(zip(name, command))
            commands = []
            for joint_idx in sorted(self.revolute_joint_idx):
                commands.append(joint_commands_with_names.get(self.revolute_joint_idx.get(joint_idx)))
        return commands

    def load_urdf(self):
        print("##############################################")
        print(self.urdf_path)
        self.robot_id = p.loadURDF(
            self.urdf_path,
            self.urdf_spawn_pose,
            useFixedBase=0,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

        (
            self.revolute_joint_idx,
            self.prismatic_joint_idx,
            self.fixed_joint_idx,
            self.link_name_idx,
        ) = self.get_properties()
        self.num_dofs = len(self.revolute_joint_idx)
        self.joint_idx = list(self.revolute_joint_idx.keys())

    def set_urdf_path(self):
        """
        Sets the path to the URDF file based on snake type (ROS2 version)
        """
        base_path = get_package_share_directory("snakelib_description")

        if self._snake_type == "REU":
            self.urdf_path = base_path + "/REU_snake/REU_instance.urdf"
        elif self._snake_type == "SEA":
            self.urdf_path = base_path + "/SEA_snake/SEA_instance.urdf"
        elif self._snake_type == "RSNAKE":
            self.urdf_path = base_path + "/RSNAKE_snake/RSNAKE_instance.urdf"
        else:
            self.urdf_path = None
    
    def get_properties(self):
        # method to parse robot model and construct 4 dictionaries:
        # revolute_joint_idx: map revolute joint indices to joint names (in URDF)
        # fixed_joint_idx: map fixed joint indices to joint names (in URDF)
        # prismatic_joint_idx: map prismatic joint indices to joint names (in URDF)
        # link_name_idx: map link names (in URDF) to joint indices
        revolute_joint_idx = {}  # (joint index: joint name)
        fixed_joint_idx = {}  # (joint index: joint name)
        prismatic_joint_idx = {}  # (joint index: joint name)
        link_name_idx = {}  # (link name: joint index)
        for idx in range(0, p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, idx)
            link_name_idx[info[12].decode("utf-8")] = idx
            if info[2] == p.JOINT_REVOLUTE:
                revolute_joint_idx[idx] = info[1].decode("utf-8")
            elif info[2] == p.JOINT_FIXED:
                fixed_joint_idx[idx] = info[1].decode("utf-8")
            elif info[2] == p.JOINT_PRISMATIC:
                prismatic_joint_idx[idx] = info[1].decode("utf-8")
            else:  # Continuous joints are not supported by getJointInfo, so otherwise assume its a revolute joint
                revolute_joint_idx[idx] = info[1].decode("utf-8")
        return revolute_joint_idx, prismatic_joint_idx, fixed_joint_idx, link_name_idx
    
    def handle_reset_simulation(self, req):
        self.get_logger().info("reseting simulation now")
        self.pause_simulation = True
        p.resetSimulation()
        # load URDF model again, set gravity and floor
        p.setGravity(0, 0, -9.81)
        self.load_terrain()
        self.robot_id = self.load_urdf()
        self.pause_simulation = False
        return []

    def handle_pause_physics(self, req):
        self.get_logger().info("pausing simulation")
        self.pause_simulation = True
        return []

    def handle_unpause_physics(self, req):
        self.get_logger().info("unpausing simulation")
        self.pause_simulation = False
        return []
    
    def load_terrain(self):
        p.loadURDF("plane.urdf")
        try:
            if self.terrain_path is not None:
                p.loadURDF(self.terrain_path, useFixedBase=1)
        except p.error as e:
            print(f'Failed to load terrain URDF. Does the file exist? PyBullet error message: "{e}"')

    """
    Starts the logging to record a video of the bullet simulation
    """

    def start_video_log(self, filename="/data/ros/pb_video.mp4"):
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, filename)
        return 0

    """
    Stops the logging of the bullet simulation video
    """

    def stop_video_log(self, filename="/data/ros/pb_video.mp4"):
        p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4, filename)
        return 0

    def update_cam_view(self, dist=4.5, yaw=45, pitch=-45):

        # Get the position of the center module
        pos, *_ = p.getLinkState(self.robot_id, self.revolute_joint_idx[int(self.num_dofs / 2) - 1])

        _pos = list(pos)
        _pos[2] = 0
        pos = tuple(_pos)

        p.resetDebugVisualizerCamera(dist, yaw, pitch, pos)
        return 0


def main():
    rclpy.init()
    pybullet_node = PyBulletRosWrapper()

    try:
        pybullet_node.start()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        pybullet_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

