#!usr/bin/env python3
from sympy import symbols, cos, sin, Matrix, pi
import numpy as np

def getTransformationMatrix(theta, d, a, alpha):
    # Define transformation matrices
    R_z_theta = Matrix([[cos(theta), -sin(theta), 0, 0],
                 [sin(theta), cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    T_z_d = Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, d],
                 [0, 0, 0, 1]])

    T_x_a = Matrix([[1, 0, 0, a],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    R_x_alpha = Matrix([[1, 0, 0, 0],
                 [0, cos(alpha), -sin(alpha), 0],
                 [0, sin(alpha), cos(alpha), 0],
                 [0, 0, 0, 1]])

    # Calculate the homogeneous transformation matrix
    T = R_z_theta * T_z_d * T_x_a * R_x_alpha
    
    return T

a1 = 0
a2 = 2.17
a3 = 1.85
a4 = 1.456

d1 = 1.319
d2 = 0
d3 = 0
d4 = 0

alpha1 = -pi/2
alpha2 = 0
alpha3 = 0
alpha4 = 0

# all joint angles zero
theta1 = theta2 = theta3 = theta4 = 0
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
T = A1 * A2 * A3 * A4
print("\nTransformation Matrix 1")
for i in range(4):
    for j in range(4):
        print(f'{T[i, j]:.4f}', end='\t')
    print()

# 2nd validation
theta1_deg = 0
theta2_deg = -45 
theta3_deg = -45  
theta4_deg = -45 

# Convert angles to radians
theta1 = np.radians(theta1_deg)
theta2 = np.radians(theta2_deg)
theta3 = np.radians(theta3_deg)
theta4 = np.radians(theta4_deg)
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
T = A1 * A2 * A3 * A4
print("\nTransformation Matrix 2")
for i in range(4):
    for j in range(4):
        print(f'{T[i, j]:.4f}', end='\t')
    print()

# 3rd validation
theta1_deg = 90 
theta2_deg = -45  
theta3_deg = +45  
theta4_deg = +45  

# Convert angles to radians
theta1 = np.radians(theta1_deg)
theta2 = np.radians(theta2_deg)
theta3 = np.radians(theta3_deg)
theta4 = np.radians(theta4_deg)

A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
T = A1 * A2 * A3 * A4
print("\nTransformation Matrix 3")
for i in range(4):
    for j in range(4):
        print(f'{T[i, j]:.4f}', end='\t')
    print()

"""
robot = 
 
%+---+-----------+-----------+-----------+-----------+
%| j |     theta |         d |         a |     alpha | 
%+---+-----------+-----------+-----------+-----------+
%|  1|         q1|     +1.319|          0|    -1.5708|
%|  2|         q2|          0|       2.17|          0|
%|  3|         q3|          0|       1.85|          0|
%|  4|         q4|          0|      1.456|          0|
%+---+-----------+-----------+-----------+-----------+
 """