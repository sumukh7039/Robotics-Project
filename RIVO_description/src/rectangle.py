#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import numpy as np
import matplotlib.pyplot as plt
from ForwardKinematics import forward_kinematics
from InverseKinematics import inverse_kinematics

def repetitiveFunc(theta0, theta1, theta2, theta3):
    rate = rospy.Rate(5)
    pub1.publish(theta0)
    pub2.publish(theta1)
    pub3.publish(theta2)
    pub4.publish(theta3)
    rate.sleep()

def move_and_generate_coordinates(x, y, z, s1, s2, a1_value, a2_value, theta, coordinatesX, coordinatesY):
    def calculate_coordinates(x1, y1):
        theta0, theta1, theta2, theta3 = inverse_kinematics(
            x1, z, y1, a1_value, a2_value, theta
        )
        co_x, co_y, co_z = forward_kinematics(theta, theta0, theta1, theta2, theta3, a1_value, a2_value)
        coordinatesX.append(co_x)
        coordinatesY.append(co_y)
        print(co_x, co_y)
        repetitiveFunc(theta0, theta1, theta2, theta3)

    for x1 in np.arange(x - s2/2, x + s2/2 + 1, 5):
        calculate_coordinates(x1, y - s1/2)
    
    for y1 in np.arange(y - s1/2, y + s1/2 + 1, 5):
        calculate_coordinates(x + s2/2, y1)
    
    for x1 in np.arange(x + s2/2, x - s2/2 - 1, -5):
        calculate_coordinates(x1, y + s1/2)

    for y1 in np.arange(y + s1/2, y - s1/2 - 1, -5):
        calculate_coordinates(x - s2/2, y1)

def publish_to_topic():

    while not rospy.is_shutdown():
        x = float(input("Enter the center x value: "))
        y = float(input("Enter the center y value: "))
        s1 = float(input("Enter the side 1 value: "))
        s2 = float(input("Enter the side 2 value: "))
        z = 75.6 + 70
        a1_value = 217
        a2_value = 185
        coordinatesX=[]
        coordinatesY=[]
        y1=y-s1/2 
        theta=3.142/2
        move_and_generate_coordinates(x, y, z, s1, s2, a1_value, a2_value, theta, coordinatesX, coordinatesY)
        plt.figure()
        plt.plot(coordinatesX, coordinatesY)
        plt.xlabel('x of the rectangle')
        plt.ylabel('y of the rectangle')
        plt.title('Rectangular trajectory')
        plt.xlim(0, 400)
        plt.ylim(0, 400)
        plt.show()

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
