#! /usr/bin/python
#
# Kinematics class which subscribes to the /joint_states topic, providing utility
# methods for it on top of those provided by KDLKinematics.
#
# Copyright (c) 2012, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Kelsey Hawkins

import numpy as np
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import rclpy
from rclpy.node import Node



def create_joint_kin(base_link, end_link, urdf_filename=None, timeout=1.0, wait=False, description_param="robot_description"):
    if urdf_filename is None:
        robot = URDF.from_parameter_server(key=description_param)

    if not wait:
        return JointKinematics(robot, base_link, end_link, timeout=timeout)
    else:
        return JointKinematicsWait(robot, base_link, end_link)


class JointKinematicsBase(KDLKinematics):
    ##
    # Perform forward kinematics on the current joint angles.
    # @param q List of joint angles for the full kinematic chain.
    #          If None, the current joint angles are used.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation or None if the joint angles are not filled.
    def forward(self, q=None, end_link=None, base_link=None):
        if q is None:
            q = self.get_joint_angles()
            if q is None:
                return None
        return super(JointKinematicsBase, self).forward(q, end_link, base_link)

    ##
    # Returns the Jacobian matrix at the end_link from the current joint angles.
    # @param q List of joint angles. If None, the current joint angles are used.
    # @return 6xN np.mat Jacobian or None if the joint angles are not filled.
    def jacobian(self, q=None):
        if q is None:
            q = self.get_joint_angles()
            if q is None:
                return None
        return super(JointKinematicsBase, self).jacobian(q)

    ##
    # Returns the joint space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return NxN np.mat Inertia matrix or None if the joint angles are not filled.
    def inertia(self, q=None):
        if q is None:
            q = self.get_joint_angles()
            if q is None:
                return None
        return super(JointKinematicsBase, self).inertia(q)

    ##
    # Returns the cartesian space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6x6 np.mat Cartesian inertia matrix or None if the joint angles are not filled.
    def cart_inertia(self, q=None):
        if q is None:
            q = self.get_joint_angles()
            if q is None:
                return None
        return super(JointKinematicsBase, self).cart_inertia(q)

    ##
    # Returns joint angles for continuous joints to a range [0, 2*PI)
    # @param q List of joint angles.
    # @return np.array of wrapped joint angles.
    def wrap_angles(self, q):
        contins = self.joint_types == "continuous"
        return np.where(contins, np.mod(q, 2 * np.pi), q)

    ##
    # Returns the current effective end effector force.
    def end_effector_force(self):
        J = self.jacobian()
        tau = self.get_joint_efforts()
        if J is None or tau is None:
            return None
        f, _, _, _ = np.linalg.lstsq(J.T, tau)
        return f


##
# Kinematics class which subscribes to the /joint_states topic, recording the current
# joint states for the kinematic chain designated.
class JointKinematics(JointKinematicsBase, Node):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain.
    # @param end_link Name of the end link of the kinematic chain.
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    # @param timeout Time in seconds to wait for the /joint_states topic.
    def __init__(self, urdf, base_link, end_link, kdl_tree=None, timeout=1.0):
        Node.__init__(self, 'joint_kinematics')
        super(JointKinematics, self).__init__(urdf, base_link, end_link, kdl_tree)
        self._joint_angles = None
        self._joint_velocities = None
        self._joint_efforts = None
        self._joint_state_inds = None

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        if timeout > 0:
            self.wait_for_joint_angles(timeout)

    ##
    # Joint angles listener callback
    def _joint_state_cb(self, msg):
        if self._joint_state_inds is None:
            joint_names_list = self.get_joint_names()
            self._joint_state_inds = [msg.name.index(joint_name) for joint_name in joint_names_list]
        self._joint_angles = [msg.position[i] for i in self._joint_state_inds]
        self._joint_velocities = [msg.velocity[i] for i in self._joint_state_inds]
        self._joint_efforts = [msg.effort[i] for i in self._joint_state_inds]

    ##
    # Wait until we have found the current joint angles.
    # @param timeout Time at which we break if we haven't recieved the angles.
    def wait_for_joint_angles(self, timeout=1.0):
        if timeout <= 0:
            return self._joint_angles is not None
        start_time = self.get_clock().now().nanoseconds / 1e9
        rate = self.create_rate(100)

        while rclpy.ok() and ((self.get_clock().now().nanoseconds / 1e9) - start_time) < timeout:
            if self._joint_efforts is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)

        if rclpy.ok():
            self.get_logger().warn("[joint_state_kdl_kin] Cannot read joint angles, timing out.")
        return False

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        if self._joint_angles is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint angles haven't been filled yet.")
            return None
        if wrapped:
            return super(JointKinematics, self).wrap_angles(self._joint_angles)
        else:
            return np.array(self._joint_angles).copy()

    ##
    # Returns the current joint velocities
    def get_joint_velocities(self):
        if self._joint_velocities is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint velocities haven't been filled yet.")
            return None
        return np.array(self._joint_velocities).copy()

    ##
    # Returns the current joint efforts
    def get_joint_efforts(self):
        if self._joint_efforts is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint efforts haven't been filled yet.")
            return None
        return np.array(self._joint_efforts).copy()


# Doesn't subscribe to the /joint_states topic until necessary
# (delay of ~0.1s)
class JointKinematicsWait(JointKinematicsBase):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain.
    # @param end_link Name of the end link of the kinematic chain.
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    # @param timeout Time in seconds to wait for the /joint_states topic.
    def __init__(self, urdf, base_link, end_link, kdl_tree=None):
        super(JointKinematicsWait, self).__init__(urdf, base_link, end_link, kdl_tree)

    def get_joint_state(self, timeout=1.0):
        try:
            js = self.wait_for_joint_message(self, "/joint_states", JointState, timeout)
        except TimeoutError as e:
            self.get_logger().warn("get_joint_state timed out after %1.1f s" % timeout)
            return None, None
        joint_names_list = self.get_joint_names()
        joint_state_inds = [js.name.index(joint_name) for joint_name in joint_names_list]
        return js, joint_state_inds

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False, timeout=1.0):
        js, js_inds = self.get_joint_state()
        if js is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint states haven't been filled yet.")
            return None
        if len(js.position) < np.max(js_inds):
            self.get_logger().warn("[joint_state_kdl_kin] Joint positions not fully filled.")
            return None
        q = np.array(js.position)[js_inds]
        if wrapped:
            return super(JointKinematicsWait, self).wrap_angles(q)
        else:
            return q

    ##
    # Returns the current joint velocities
    def get_joint_velocities(self, timeout=1.0, wrapped=False):
        js, js_inds = self.get_joint_state()
        if js is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint states haven't been filled yet.")
            return None
        if len(js.velocity) < np.max(js_inds):
            self.get_logger().warn("[joint_state_kdl_kin] Joint velocity not fully filled.")
            return None
        qd = np.array(js.velocity)[js_inds]
        if wrapped:
            return super(JointKinematicsWait, self).wrap_angles(qd)
        else:
            return qd

    ##
    # Returns the current joint efforts
    def get_joint_efforts(self, timeout=1.0, wrapped=False):
        js, js_inds = self.get_joint_state()
        if js is None:
            self.get_logger().warn("[joint_state_kdl_kin] Joint states haven't been filled yet.")
            return None
        if len(js.effort) < np.max(js_inds):
            self.get_logger().warn("[joint_state_kdl_kin] Joint efforts not fully filled.")
            return None
        i = np.array(js.effort)[js_inds]
        if wrapped:
            return super(JointKinematicsWait, self).wrap_angles(i)
        else:
            return i
        
    def wait_for_joint_message(node, topic, msg_type, timeout=1.0):
        future_msg = rclpy.task.Future()

        def callback(msg):
            if not future_msg.done():
                future_msg.set_result(msg)

        sub = node.create_subscription(msg_type, topic, callback, 10)

        start_time = node.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if future_msg.done():
                return future_msg.result()
            if (node.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                break

        raise TimeoutError(f"Timeout while waiting for message on topic '{topic}'")



def main():
    rclpy.init()
    import sys
    def usage():
        print("Tests for kdl_parser:\n")
        print("kdl_parser <urdf file>")
        print("\tLoad the URDF from file.")
        print("kdl_parser")
        print("\tLoad the URDF from the parameter server.")
        sys.exit(1)

    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()
    if len(sys.argv) == 1:
        robot = URDF.from_parameter_server()

    if True:
        import random

        base_link = robot.get_root()
        end_link = list(robot.link_map.keys())[random.randint(0, len(robot.link_map) - 1)]
        print("Root link: %s; Random end link: %s" % (base_link, end_link))
        js_kin = JointKinematics(robot, base_link, end_link)
        print("Joint angles:", js_kin.get_joint_angles())
        print("Joint angles (wrapped):", js_kin.get_joint_angles(True))
        print("Joint velocities:", js_kin.get_joint_velocities())
        print("Joint efforts:", js_kin.get_joint_efforts())
        print("Jacobian:", js_kin.jacobian())
        kdl_pose = js_kin.forward()
        print("FK:", kdl_pose)
        print("End effector force:", js_kin.end_effector_force())

        if True:
            import tf
            from hrl_geom.pose_converter import PoseConv
            import time

            tf_list = tf.TransformListener()
            time.sleep(1.0)
            t = tf_list.getLatestCommonTime(base_link, end_link)
            tf_pose = PoseConv.to_homo_mat(tf_list.lookupTransform(base_link, end_link, t))
            print("Error with TF:", np.linalg.norm(kdl_pose - tf_pose))


if __name__ == "__main__":
    main()
