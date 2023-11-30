#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from InverseKinematics import inverse_kinematics
from ForwardKinematics import forward_kinematics

def publish_to_topic():
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        theta = math.radians(float(input("Enter the angle of end effecter: ")))
        theta0 = math.radians(float(input("Enter the theta0: ")))
        theta1 = math.radians(float(input("Enter the theta1: ")))
        theta2 = math.radians(float(input("Enter the theta2: ")))
        theta3 = math.radians(float(input("Enter the theta3: ")))
        a1 =217
        a2= 185
        pub1.publish(theta0)
        pub2.publish(theta1)
        pub3.publish(theta2)
        pub4.publish(theta3)
        x,y,z=forward_kinematics(theta,theta0,theta1, theta2, theta3, a1, a2)
        print(f"x is {x} y is {y} z is {z}")
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_control', anonymous=True)
        pub1 = rospy.Publisher('dof_1_control/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('dof_2_control/command', Float64, queue_size=10)
        pub3 = rospy.Publisher('dof_3_control/command', Float64, queue_size=10)
        pub4 = rospy.Publisher('dof_4_control/command', Float64, queue_size=10)
        publish_to_topic()
    except rospy.ROSInterruptException:
        pass
