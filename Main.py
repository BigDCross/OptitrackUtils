import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from OptitrackUtils import *

import tf
import math

def main():
    rospy.init_node('OptitrackUtils')

    opti_utils = OptitrackUtils(["ArDroneA"])
    ardrone_rb = opti_utils.get_rigid_body("ArDroneA")

    vel_pub = rospy.Publisher("ArDroneA/linear_velocity", Vector3)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #print "turtleA: ",
        #print opti_utils.get_rigid_body_linear_velocity("turtleA")
        """
        print "ardroneA.position(): ",
        print ardrone_rb.get_position()
        print "ardroneA.within_box([0, 0, 0], [1, 1, 1]): ",
        print ardrone_rb.within_box([0, 0, 0], [1, 1, 1])
        print "ardrone_rb.get_position_error([0, 0, 0]): ",
        print ardrone_rb.get_position_error([2, 2, 0])
        orientation = ardrone_rb.get_orientation()
        print "ardrone_rb.get_orientation in euler: ",
        print [math.degrees(i) for i in tf.transformations.euler_from_quaternion(orientation)]
        print "ardrone_rb.get_orientation_error([0, 0, 0, 1]): ",
        print ardrone_rb.get_orientation_error([0, 0, 0, 1])
        orient_error = ardrone_rb.get_orientation_error([0, 0, 0, 1])
        orient_error_euler = tf.transformations.euler_from_quaternion(orient_error)
        print "orient_error_euler: ",
        print [math.degrees(i) for i in orient_error_euler]
        """

        print "ardroneA.position(): "
        print ardrone_rb.get_position_msg()
        orientation = ardrone_rb.get_orientation()
        print "ardrone_rb.get_orientation in euler: "
        print [math.degrees(i) for i in tf.transformations.euler_from_quaternion(orientation)]
        print "ardrone_rb.get_linear_velocity: "
        print ardrone_rb.get_linear_velocity_msg()
        print
        print

        vel_pub.publish(ardrone_rb.get_linear_velocity_msg())

        rate.sleep()

main()
