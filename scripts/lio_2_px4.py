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

            # The actual child frame of "odometry/imu" is called "odom_imu"
            # which does not appear in the tf_tree, nor follows ROS convention
            # (https://www.ros.org/reps/rep-0105.html). After inspection in rviz,
            # it appears that the frame "odom_imu" is the same as "base_link".
            # "odom_imu" also shares parent frame with "base_link", that is, "odom".
            # Therefore, assume they are identical and rename the child frame from
            # "odom_imu" to "base_link" and let MAVROS Odometry plugin handle all
            # necessary transforms from ROS to PX4 coordinate systems.
            # MAVROS Odometry plugin:
            # https://github.com/mavlink/mavros/blob/master/mavros_extras/src/plugins/odom.cpp
            px4_compliant_msg.child_frame_id = "base_link_px4"

            self.odom_pub.publish(px4_compliant_msg)
        


if __name__ == "__main__":
    rospy.init_node("lio_2_px4_node")

    # Wait for Flight Controller connection
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    rospy.loginfo("Flight controller connected")
    
    state_sub.unregister()

    odom_sub = rospy.Subscriber("odometry/imu", Odometry, callback = odometry_cb)

    while (not rospy.is_shutdown() and "lio_msg" not in globals()):
         rate.sleep()
    rospy.loginfo("Receiving LIO data")
    
    odr = OdometryRelay()
    rospy.Timer(rospy.Duration(.02), odr.read_and_pub_lio)

    rospy.spin()

