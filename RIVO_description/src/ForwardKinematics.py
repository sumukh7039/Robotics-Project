import math

def forward_kinematics(theta, theta0, theta1, theta2, theta3, a1, a2):
    r = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2)+145.6*math.cos(theta1+theta2+theta3)
    x = r * math.cos(theta0)
    y = r * math.sin(theta0)
    z = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2)-145.6*math.sin(theta)
    return x, y, z + 131.656