#! /usr/bin/env python3

"""
Node for relaying LIO pose from LIO-SAM to PX4
"""

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

current_state = State()

def state_cb(msg: State):
    global current_state
    current_state = msg

# odometry_cb: Transform and publish LIO pose to PX4
def odometry_cb(msg: Odometry):
    px4_compliant_msg = msg

    # transform from FLU (ROS convention) to FRD (PX4 convention) 
    # according to https://docs.px4.io/main/en/ros/external_position_estimation.html
    px4_compliant_msg.pose.pose.position.x = msg.pose.pose.position.y
    px4_compliant_msg.pose.pose.position.y = msg.pose.pose.position.x
    px4_compliant_msg.pose.pose.position.z = -msg.pose.pose.position.z

    px4_compliant_msg.pose.pose.orientation.x = msg.pose.pose.orientation.y
    px4_compliant_msg.pose.pose.orientation.y = msg.pose.pose.orientation.x
    px4_compliant_msg.pose.pose.orientation.z = -msg.pose.pose.orientation.z

    px4_compliant_msg.child_frame_id

    odom_pub.publish(px4_compliant_msg)
    odom_pub_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lio_2_px4")
    global time_last_odom
    time_last_odom = rospy.Time.now()

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    odom_sub = rospy.Subscriber("odometry/imu", Odometry, callback = odometry_cb)
    odom_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)

    # Wait for Flight Controller connection
    rate = rospy.Rate(20)
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    global odom_pub_rate
    odom_pub_rate = rospy.Rate(50)

    rospy.loginfo("Flight controller connected")
    rospy.spin()

