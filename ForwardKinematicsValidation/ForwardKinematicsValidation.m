% Link lengths (a) [m]
a1 = 0.5;  a2 = 0.5; 

% D-H Table  
% Format: Link([theta,   d,    a,    alpha,  R/P ])
% R - Revolute (0); P - Prismatic (1)

%+---+-----------+-----------+-----------+-----------+-----------+
%| j |     theta |         d |         a |     alpha |    offset |
%+---+-----------+-----------+-----------+-----------+-----------+
%|  1|         q1|      1.319|          0|     1.5708|          0|
%|  2|         q2|          0|       2.17|          0|          0|
%|  3|         q3|          0|       1.85|          0|          0|
%|  4|         q4|          0|      1.456|          0|          0|
%+---+-----------+-----------+-----------+-----------+-----------+

% The lengths have been scaled down by a factor of 100 for representation purposes

L(1) = Link([0, 1.319, 0, pi/2, 0]); % Joint 1: Revolute
L(2) = Link([0, 0, 2.17, 0, 0]); % Joint 2: Prismatic
L(3) = Link([0, 0, 1.85, 0, 0]); % Joint 3: Prismatic
L(4) = Link([0, 0, 1.456, 0, 0]); % Joint 4: Prismatic

robot = SerialLink(L);
robot.name = 'RRRR-Example';

robot.plot([pi, pi/2, 0, 0]);
% Visualize the robot and interactively control its joints
% robot.teach;
robot