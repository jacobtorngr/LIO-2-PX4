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

# odometry_cb: Read LIO
def odometry_cb(msg: Odometry):
     global lio_msg
     lio_msg = msg

    

class OdometryRelay:
    def __init__(self):
        self.odom_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=5)

    def read_and_pub_lio(self, event=None):
            global lio_msg
            px4_compliant_msg = copy.deepcopy(lio_msg)

            # transform from FLU (ROS convention) to FRD (PX4 convention) 
            # according to https://docs.px4.io/main/en/ros/external_position_estimation.html
            px4_compliant_msg.pose.pose.position.x = lio_msg.pose.pose.position.y
            px4_compliant_msg.pose.pose.position.y = lio_msg.pose.pose.position.x
            px4_compliant_msg.pose.pose.position.z = -lio_msg.pose.pose.position.z

            px4_compliant_msg.pose.pose.orientation.x = lio_msg.pose.pose.orientation.y
            px4_compliant_msg.pose.pose.orientation.y = lio_msg.pose.pose.orientation.x
            px4_compliant_msg.pose.pose.orientation.z = -lio_msg.pose.pose.orientation.z

            px4_compliant_msg.twist.twist.linear.x = lio_msg.twist.twist.linear.x
            px4_compliant_msg.twist.twist.linear.y = -lio_msg.twist.twist.linear.y
            px4_compliant_msg.twist.twist.linear.z = -lio_msg.twist.twist.linear.z

            px4_compliant_msg.twist.twist.angular.x = lio_msg.twist.twist.angular.x
            px4_compliant_msg.twist.twist.angular.y = -lio_msg.twist.twist.angular.y
            px4_compliant_msg.twist.twist.angular.z = -lio_msg.twist.twist.angular.z
            
            px4_compliant_msg.child_frame_id = "base_link"

            self.odom_pub.publish(px4_compliant_msg)
        


if __name__ == "__main__":
    rospy.init_node("liosam_2_px4_node")

    # Wait for Flight Controller connection
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    rospy.loginfo("Flight controller connected")
    
    state_sub.unregister()

    odom_sub = rospy.Subscriber("odometry/imu", Odometry, callback = odometry_cb)

    while "lio_msg" not in globals():
         rate.sleep()
    rospy.loginfo("Receiving LIO data")
    
    odr = OdometryRelay()
    rospy.Timer(rospy.Duration(.02), odr.read_and_pub_lio)

    rospy.spin()

