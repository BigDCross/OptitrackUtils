import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from math import fabs
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
from collections import deque

# Internally uses tf notation for vectors and quaternions, that is, they are
# array like. Vectors are [x,y,z] and quaternions are [x,y,z,w]
# Input is in rosmsg

# TODO: Linear velocity not correct.
# Linear velocity is much more correct when vrpn_client_ros uses server time
# rather than client time
# Linear velocity can be made much smoother with a windowed running average as
# well

def arrVector3(x, y, z):
    return [x, y, z]

def arrQuaternion(x, y, z, w):
    return [x, y, z, w]

class RigidBody:
    def __init__(self, name):
        self.last_pose_msg = PoseStamped()
        self.pose_msg = PoseStamped()
        self.position = arrVector3(0, 0, 0)
        self.orientation = arrQuaternion(0, 0, 0, 1)

        # TODO: Turn into full twist message
        self.position_diff_queue = deque(maxlen=5)
        self.linear_velocity = arrVector3(0, 0, 0)
        self.angular_velocity = arrVector3(0, 0, 0)

        self.last_timestamp = rospy.Time.now()

    def input_pose_msg(self, pose):
        self.pose_msg = pose

        self.position = arrVector3(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z)

        self.orientation = arrQuaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)

        dt = (self.pose_msg.header.stamp - self.last_timestamp).to_sec()
        pos_diff = arrVector3(
                self.pose_msg.pose.position.x - self.last_pose_msg.pose.position.x,
                self.pose_msg.pose.position.y - self.last_pose_msg.pose.position.y,
                self.pose_msg.pose.position.z - self.last_pose_msg.pose.position.z)

        #print pos_diff
        #print self.linear_velocity
        #print dt
        #print [i / dt for i in self.linear_velocity]

        self.position_diff_queue.append((pos_diff, dt))

        self.linear_velocity = arrVector3(0, 0, 0)
        for item in self.position_diff_queue:
            self.linear_velocity[0] += item[0][0] / item[1]
            self.linear_velocity[1] += item[0][1] / item[1]
            self.linear_velocity[2] += item[0][2] / item[1]

        self.linear_velocity[0] /= len(self.position_diff_queue)
        self.linear_velocity[1] /= len(self.position_diff_queue)
        self.linear_velocity[2] /= len(self.position_diff_queue)

        self.last_timestamp = self.pose_msg.header.stamp
        self.last_pose_msg = self.pose_msg

    def get_pose_msg(self):
        return self.pose_msg

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation

    def get_linear_velocity(self):
        return self.linear_velocity

    def get_position_msg(self):
        return Point(*self.position)

    def get_orientation_msg(self):
        return Quaternion(*self.orientation)

    def get_linear_velocity_msg(self):
        return Vector3(*self.linear_velocity)

    def within_box(self, box_center, box_dims):
        x_width = box_center[0] + box_dims[0]
        y_width = box_center[1] + box_dims[1]
        z_width = box_center[2] + box_dims[2]

        if fabs(self.position.x - box_center[0]) > x_width:
            return False
        if fabs(self.position.y - box_center[1]) > y_width:
            return False
        if fabs(self.position.z - box_center[2]) > z_width:
            return False

        return True

    def get_position_error(self, desired_pos):
        return [self.position.x - desired_pos[0],
                self.position.y - desired_pos[1],
                self.position.z - desired_pos[2]]

    def get_orientation_error(self, desired_orientation):
        # error * current_orientation = desired_orientation
        # error = desired_orientation / current_orientation
        # error = desired_orientation * inverse(current_orientation)

        # tf quaternions are represented as a vector of [x, y, z, w]
        inverted = arrQuaternion(
                -self.orientation[0],
                -self.orientation[1],
                -self.orientation[2],
                self.orientation[3])

        return quaternion_multiply(desired_orientation, inverted)

