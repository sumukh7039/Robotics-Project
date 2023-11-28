#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import numpy as np

def publish_to_topic():
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        x = float(input("Eter the center x value: "))
        y = float(input("Enter the center y value: "))
        r = float(input("Enter the radius: "))
        z = 75.6 + 70
        a1_value = 217
        a2_value = 185
        angle=0
        for angles in np.arange(0,360,10):
            theta=math.radians(angles)
            x1=x+r*math.cos(theta)
            y1=y+r*math.sin(theta)
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value
            )
            print(theta0)
            print(theta1)
            print(theta2)
            print(theta3)
            pub1.publish(theta0)
            pub2.publish(theta1)
            pub3.publish(theta2)
            pub4.publish(theta3)
            rate.sleep()

        
        

def inverse_kinematics(x, z, y, a1, a2):
    # Calculate theta2 using inverse trigonometric function
    arg_acos = (x**2 + y**2 + (z - 131.9) ** 2 - a1**2 - a2**2) / (2 * a1 * a2)
    arg_acos = max(min(arg_acos, 1), -1)  # Ensure arg_acos is within [-1, 1]
    theta2 = -math.acos(arg_acos)
    theta0 = math.atan2(y, x)

    # Calculate theta1 using inverse trigonometric functions
    theta1_temp = math.atan2(z - 131.9, math.sqrt(x**2 + y**2)) - math.atan2(
        a2 * math.sin(theta2), a1 + a2 * math.cos(theta2)
    )
    theta1 = math.degrees(theta1_temp)

    # Adjust theta1 to [0, 180] degrees
    if theta1_temp > math.pi:
        theta1 = 180 - theta1

    theta3 = -3.142 / 2 - math.radians(theta1) - theta2

    return -theta0, math.radians(theta1), theta2, theta3


def forward_kinematics(theta0, theta1, theta2, a1, a2):
    r = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2)
    x = r * math.cos(theta0)
    y = r * math.sin(theta0)
    z = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2)
    return x, y, z + 131.9

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
