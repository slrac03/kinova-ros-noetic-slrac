#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def joint_state_publisher():
    rospy.init_node('joint_state_publisher_sim')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(60)

    joint_names = [
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6"
    ]

    angle = 0.0

    while not rospy.is_shutdown():
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = [math.sin(angle + i) * 0.5 for i in range(len(joint_names))]

        pub.publish(js)
        angle += 0.05
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
