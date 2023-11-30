#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math

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
        publish_to_topic()
    except rospy.ROSInterruptException:
        pass
