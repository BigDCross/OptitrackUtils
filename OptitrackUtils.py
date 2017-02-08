import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from RigidBody import *

class OptitrackUtils:
    def __init__(self, rigid_body_names):
        self.subscribers = []
        self.rigid_body_names = rigid_body_names
        self.vrpndatatopic = 'vrpn_client_node'

        self.rigid_bodies = {}
        for name in self.rigid_body_names:
            rb = RigidBody(name)
            self.rigid_bodies[name] = rb

        self.setup_subscribers()

    def pose_callback(self, pose, name):
        self.rigid_bodies[name].input_pose_msg(pose)

    def setup_subscribers(self):
        for idx, r in enumerate(self.rigid_body_names):
            rospy.loginfo("subscribing to topic: /" + self.vrpndatatopic + "/" + r + "/pose")
            sub = rospy.Subscriber(self.vrpndatatopic + "/" + r + "/pose", PoseStamped, self.pose_callback, (r))
            self.subscribers.append(sub)

    def get_rigid_body(self, name):
        return self.rigid_bodies[name]

"""
# Necessary?
    def get_rigid_body_pose(self, name):
        return self.rigid_bodies[name].get_pose_msg()

    def get_rigid_body_position(self, name):
        return self.rigid_bodies[name].get_position()

    def get_rigid_body_orientation(self, name):
        return self.rigid_bodies[name].get_orientation()

    def get_rigid_body_linear_velocity(self, name):
        return self.rigid_bodies[name].get_linear_velocity()
"""
