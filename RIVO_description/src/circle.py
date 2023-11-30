#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import numpy as np
import matplotlib.pyplot as plt
from ForwardKinematics import forward_kinematics
from InverseKinematics import inverse_kinematics

def publish_to_topic():
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        x = float(input("Eter the center x value: "))
        y = float(input("Enter the center y value: "))
        r = float(input("Enter the radius: "))
        z = 75.6 + 70
        a1_value = 217
        a2_value = 185
        angle=0
        coordinatesX=[]
        coordinatesY=[]
        for angles in np.arange(0,361,1):
            theta=math.radians(angles)
            x1=x+r*math.cos(theta)
            y1=y+r*math.sin(theta)
            print(f"x y z a1 a2 {x1} {y1} {z} {a1_value} {a2_value}")
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value, math.radians(90)
            )
            print(f"theta values are {theta0} {theta1} {theta2}")
            co_x,co_y,co_z=forward_kinematics(math.radians(90),theta0,theta1, theta2, theta3, a1_value, a2_value)
            print(f"inverse {co_x}, {co_y}")
            coordinatesX.append(co_x)
            coordinatesY.append(co_y)
            pub1.publish(theta0)
            pub2.publish(theta1)
            pub3.publish(theta2)
            pub4.publish(theta3)
            rate.sleep()
        plt.figure()
        plt.plot(coordinatesX, coordinatesY)
        plt.xlabel('x of the circle')
        plt.ylabel('y of the circle')
        plt.title('Circular trajectory')
        plt.xlim(0, 400)
        plt.ylim(0,400)
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
