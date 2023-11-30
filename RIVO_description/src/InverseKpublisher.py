#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
from InverseKinematics import inverse_kinematics

def publish_to_topic():
    rospy.init_node('robot_control', anonymous=True)
    pub1 = rospy.Publisher('dof_1_control/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('dof_2_control/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('dof_3_control/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('dof_4_control/command', Float64, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        x = float(input("Enter the x value: "))
        y = float(input("Enter the y value: "))
        z = float(input("Enter the z value "))
        theta=float(input("Enter the angle of the gripper wrt ground "))
        theta=math.radians(theta)
        z = z+145.6*math.sin(theta)
        a1_value = 217
        a2_value = 185
        theta0, theta1, theta2, theta3 = inverse_kinematics(
    x, z, y, a1_value, a2_value, theta
)
        pub1.publish(theta0)
        pub2.publish(theta1)
        pub3.publish(theta2)
        pub4.publish(theta3)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_to_topic()
    except rospy.ROSInterruptException:
        pass
