from sympy import symbols, Matrix, cos, sin, pi

def getTransformationMatrix(theta, d, a, alpha):
    # Define transformation matrices
    R_z_theta = Matrix([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta), cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T_z_d = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

    T_x_a = Matrix([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    R_x_alpha = Matrix([
        [1, 0, 0, 0],
        [0, cos(alpha), -sin(alpha), 0],
        [0, sin(alpha), cos(alpha), 0],
        [0, 0, 0, 1]
    ])

    # Calculate the homogeneous transformation matrix
    T = R_z_theta * T_z_d * T_x_a * R_x_alpha
    
    return T

theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')

a1 = 0
a2 = 217
a3 = 185
a4 = 145.6

d1 = 1319
d2 = 0
d3 = 0
d4 = 0

alpha1 = -pi/2
alpha2 = 0
alpha3 = 0
alpha4 = 0

# all joint angles zero
# theta1 = theta2 = theta3 = theta4 = 0
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
T = A1 * A2 * A3 * A4

# Define symbols for matrix entries
a11, a12, a13, a14 = symbols('a11 a12 a13 a14')
a21, a22, a23, a24 = symbols('a21 a22 a23 a24')
a31, a32, a33, a34 = symbols('a31 a32 a33 a34')
a41, a42, a43, a44 = symbols('a41 a42 a43 a44')

# Create a 4x4 matrix
matrix = Matrix([
    [a11, a12, a13, a14],
    [a21, a22, a23, a24],
    [a31, a32, a33, a34],
    [a41, a42, a43, a44]
])

# Display the matrix in a beautified manner
pprint(matrix)

# Split the matrix into individual coefficients
a11, a12, a13, a14 = T[0, 0], T[0, 1], T[0, 2], T[0, 3]
a21, a22, a23, a24 = T[1, 0], T[1, 1], T[1, 2], T[1, 3]
a31, a32, a33, a34 = T[2, 0], T[2, 1], T[2, 2], T[2, 3]
a41, a42, a43, a44 = T[3, 0], T[3, 1], T[3, 2], T[3, 3]

# Write the coefficients to a file
with open('coefficients.txt', 'w') as file:
    file.write(f"a11: {a11}\n")
    file.write(f"a12: {a12}\n")
    file.write(f"a13: {a13}\n")
    file.write(f"a14: {a14}\n")
    file.write(f"a21: {a21}\n")
    file.write(f"a22: {a22}\n")
    file.write(f"a23: {a23}\n")
    file.write(f"a24: {a24}\n")
    file.write(f"a31: {a31}\n")
    file.write(f"a32: {a32}\n")
    file.write(f"a33: {a33}\n")
    file.write(f"a34: {a34}\n")
    file.write(f"a41: {a41}\n")
    file.write(f"a42: {a42}\n")
    file.write(f"a43: {a43}\n")
    file.write(f"a44: {a44}\n")

print("Coefficients have been written to 'coefficients.txt'.")
