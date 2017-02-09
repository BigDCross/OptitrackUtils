import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from optitrack_utils import *

import tf
import math

def main():
    rospy.init_node('OptitrackUtils')

    opti_utils = OptitrackUtils(["ArDroneA"])
    ArDroneA_rb = opti_utils.get_rigid_body("ArDroneA")

    vel_pub = rospy.Publisher("ArDroneA/linear_velocity", Vector3)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #print "turtleA: ",
        #print opti_utils.get_rigid_body_linear_velocity("turtleA")
        """
        print "ardroneA.position(): ",
        print ArDroneA_rb.get_position()
        print "ardroneA.within_box([0, 0, 0], [1, 1, 1]): ",
        print ArDroneA_rb.within_box([0, 0, 0], [1, 1, 1])
        print "ArDroneA_rb.get_position_error([0, 0, 0]): ",
        print ArDroneA_rb.get_position_error([2, 2, 0])
        orientation = ArDroneA_rb.get_orientation()
        print "ArDroneA_rb.get_orientation in euler: ",
        print [math.degrees(i) for i in tf.transformations.euler_from_quaternion(orientation)]
        print "ArDroneA_rb.get_orientation_error([0, 0, 0, 1]): ",
        print ArDroneA_rb.get_orientation_error([0, 0, 0, 1])
        orient_error = ArDroneA_rb.get_orientation_error([0, 0, 0, 1])
        orient_error_euler = tf.transformations.euler_from_quaternion(orient_error)
        print "orient_error_euler: ",
        print [math.degrees(i) for i in orient_error_euler]
        """

        print "ardroneA.position(): "
        print ArDroneA_rb.get_position_msg()
        orientation = ArDroneA_rb.get_orientation()
        print "ArDroneA_rb.get_orientation in euler: "
        print [math.degrees(i) for i in tf.transformations.euler_from_quaternion(orientation)]
        print "ArDroneA_rb.get_linear_velocity: "
        print ArDroneA_rb.get_linear_velocity_msg()
        print
        print

        vel_pub.publish(ArDroneA_rb.get_linear_velocity_msg())

        rate.sleep()

main()
