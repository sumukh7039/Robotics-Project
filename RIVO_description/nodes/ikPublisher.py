# inverse of two degrees of freedom for RR manipulator
import math


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


# Example usage:
x_target = 300  # Replace with your desired x-coordinate
z_target = 75.6 + 70  # Replace with your desired y-coordinate
y_target = 150

a1_value = 217
a2_value = 185

theta0_solution, theta1_solution, theta2_solution, theta3_solution = inverse_kinematics(
    x_target, z_target, y_target, a1_value, a2_value
)

print("Joint angles (degrees):")
print("Theta0:", (theta0_solution))
print("Theta1:", (theta1_solution))
print("Theta2:", (theta2_solution))
print("Theta3:", (theta3_solution))


# Use the obtained joint angles for forward kinematics
x_forward, y_forward, z_forward = forward_kinematics(
    theta0_solution, theta1_solution, theta2_solution, a1_value, a2_value
)

print("Forward kinematics result:")
print("X:", x_forward)
print("Y:", y_forward)
print("Z:", z_forward)


# # Example usage:
# gamma_target = math.radians(90)  # Replace with your desired gamma (constraint)

# # Replace with your desired link lengths
# a1_value = 217
# a2_value = 185
# a3_value = 75.6

# theta1_solution, theta2_solution, theta3_solution = inverse_kinematics(x_target, y_target, gamma_target, a1_value, a2_value, a3_value)

# print("Inverse Kinematics:")
# print("Joint angles (degrees):")
# print("Theta1:", math.degrees(theta1_solution))
# print("Theta2:", math.degrees(theta2_solution))
# print("Theta3:", math.degrees(theta3_solution))


# def forward_kinematics(theta1, theta2, theta3, a1, a2, a3):
#     x = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2) + a3 * math.cos(theta1 + theta2 + theta3)
#     y = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2) + a3 * math.sin(theta1 + theta2 + theta3)
#     return x, y

# x_forward, y_forward = forward_kinematics(theta1_solution, theta2_solution, theta3_solution, a1_value, a2_value, a3_value)

# print("\nForward Kinematics:")
# print("End-effector coordinates:")
# print("X:", x_forward)
# print("Y:", y_forward)
