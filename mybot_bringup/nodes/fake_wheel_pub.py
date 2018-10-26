#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_wheel_pub")
pub = rospy.Publisher('joint_states', JointState, queue_size=5)

rate = 20
r = rospy.Rate(rate)

msg = JointState()
msg.name = ["left_wheel_joint", "right_wheel_joint"]
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    r.sleep()
