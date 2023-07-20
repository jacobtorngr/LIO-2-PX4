#! /usr/bin/env python3

"""
Node for relaying LIO pose from LIO-SAM to PX4
"""

import rospy
import copy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

current_state = State()

def state_cb(msg: State):
    global current_state
    current_state = msg

class OdometryTransformation:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("odometry/imu", Odometry, callback = self.odometry_cb)
        self.odom_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)

    def read_and_pub_lio_odom(self, msg: Odometry):
            px4_compliant_msg = copy.deepcopy(msg)

            # transform from FLU (ROS convention) to FRD (PX4 convention) 
            # according to https://docs.px4.io/main/en/ros/external_position_estimation.html
            px4_compliant_msg.pose.pose.position.x = msg.pose.pose.position.y
            px4_compliant_msg.pose.pose.position.y = msg.pose.pose.position.x
            px4_compliant_msg.pose.pose.position.z = -msg.pose.pose.position.z

            px4_compliant_msg.pose.pose.orientation.x = msg.pose.pose.orientation.y
            px4_compliant_msg.pose.pose.orientation.y = msg.pose.pose.orientation.x
            px4_compliant_msg.pose.pose.orientation.z = -msg.pose.pose.orientation.z

            px4_compliant_msg.twist.twist.linear.x = msg.twist.twist.linear.x
            px4_compliant_msg.twist.twist.linear.y = -msg.twist.twist.linear.y
            px4_compliant_msg.twist.twist.linear.z = -msg.twist.twist.linear.z

            px4_compliant_msg.twist.twist.angular.x = msg.twist.twist.angular.x
            px4_compliant_msg.twist.twist.angular.y = -msg.twist.twist.angular.y
            px4_compliant_msg.twist.twist.angular.z = -msg.twist.twist.angular.z
            
            px4_compliant_msg.child_frame_id = "base_link"

    # odometry_cb: Read odometry
    def odometry_cb(self, msg: Odometry):
         self.read_and_pub_lio_odom(msg)
         
    


    
        


if __name__ == "__main__":
    rospy.init_node("liosam_2_px4_node")

    # Wait for Flight Controller connection
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    rospy.loginfo("Flight controller connected")
    state_sub.unregister()


    odt = OdometryTransformation()
    rospy.Timer(rospy.Duration(.02), odt.odometry_cb())

    rospy.spin()

