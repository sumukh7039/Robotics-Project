import math
from ForwardKinematics import forward_kinematics

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
    return theta0, theta1, theta2, theta3