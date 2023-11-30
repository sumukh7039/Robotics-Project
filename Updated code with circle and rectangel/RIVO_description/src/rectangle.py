#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import numpy as np
import matplotlib.pyplot as plt

def repetitiveFunc(theta0, theta1, theta2, theta3):
    rate = rospy.Rate(5)
    print(theta0)
    print(theta1)
    print(theta2)
    print(theta3)
    pub1.publish(theta0)
    pub2.publish(theta1)
    pub3.publish(theta2)
    pub4.publish(theta3)
    rate.sleep()

def publish_to_topic():

    while not rospy.is_shutdown():
        x = float(input("Eter the center x value: "))
        y = float(input("Enter the center y value: "))
        s1 = float(input("Enter the sise 1 value: "))
        s2 = float(input("Enter the side 2 value: "))
        z = 75.6 + 70
        a1_value = 217
        a2_value = 185
        coordinatesX=[]
        coordinatesY=[]
        y1=y-s1/2 
        theta=3.142/2
        for x1 in np.arange(x-s2/2,x+s2/2+1,5):
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value,theta
            )
            co_x,co_y,co_z=forward_kinematics(theta,theta0,theta1, theta2, theta3, a1_value, a2_value)
            coordinatesX.append(co_x)
            coordinatesY.append(co_y)
            repetitiveFunc(theta0, theta1, theta2, theta3)
        
        x1=x+s2/2
        for y1 in np.arange(y-s1/2,y+s1/2+1,5):
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value,theta
            )
            co_x,co_y,co_z=forward_kinematics(theta,theta0,theta1, theta2, theta3, a1_value, a2_value)
            coordinatesX.append(co_x)
            coordinatesY.append(co_y)
            repetitiveFunc(theta0, theta1, theta2, theta3)
        
        y1=y+s1/2
        for x1 in np.arange(x+s2/2,x-s2/2+1-1,-5):
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value,theta
            )
            co_x,co_y,co_z=forward_kinematics(theta,theta0,theta1, theta2, theta3, a1_value, a2_value)
            coordinatesX.append(co_x)
            coordinatesY.append(co_y)
            repetitiveFunc(theta0, theta1, theta2, theta3)

        x1=x-s2/2
        for y1 in np.arange(y+s1/2,y-s1/2-1,-5):
            theta0, theta1, theta2, theta3 = inverse_kinematics(
                x1, z, y1, a1_value, a2_value,theta
            )
            co_x,co_y,co_z=forward_kinematics(theta,theta0,theta1, theta2, theta3, a1_value, a2_value)
            coordinatesX.append(co_x)
            coordinatesY.append(co_y)
            repetitiveFunc(theta0, theta1, theta2, theta3)

        plt.figure()
        plt.plot(coordinatesX, coordinatesY)
        plt.xlabel('x of the circle')
        plt.ylabel('y of the circle')
        plt.title('Circular trajectory')
        plt.xlim(-400, 400)
        plt.ylim(-400, 400)
        plt.show()

def inverse_kinematics(x, z, y, a1, a2, theta):
    # Calculate theta2 using inverse trigonometric function
    theta0 = math.atan2(y, x)
    y=y-145.6*math.cos(theta)*math.sin(theta0)
    x=x-145.6*math.cos(theta)*math.cos(theta0)
    arg_acos = (x**2 + y**2 + (z - 131.9) ** 2 - a1**2 - a2**2) / (2 * a1 * a2)
    arg_acos = max(min(arg_acos, 1), -1)  # Ensure arg_acos is within [-1, 1]
    theta2 = -math.acos(arg_acos)

    # Calculate theta1 using inverse trigonometric functions
    theta1_temp = math.atan2(z - 131.9, math.sqrt(x**2 + y**2)) - math.atan2(
        a2 * math.sin(theta2), a1 + a2 * math.cos(theta2)
    )
    theta1 = math.degrees(theta1_temp)

    # Adjust theta1 to [0, 180] degrees
    if theta1_temp > math.pi:
        theta1 = 180 - theta1
    theta1=math.radians(theta1)
    theta3 = -theta - theta1 - theta2

    print(f"forward kinamatics endPoint is {forward_kinematics(theta,theta0,theta1, theta2, theta3, a1, a2)}")
    print(f"{math.degrees(theta0)} {math.degrees(theta1)} {math.degrees(theta2)} {math.degrees(theta3)}")
    return theta0, theta1, theta2, theta3


def forward_kinematics(theta, theta0, theta1, theta2, theta3, a1, a2):
    r = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2)+145.6*math.cos(theta1+theta2+theta3)
    print(f"r is {r}")
    x = r * math.cos(-theta0)
    y = r * math.sin(-theta0)
    z = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2)-145.6*math.sin(theta)
    return x, y, z + 131.656

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
