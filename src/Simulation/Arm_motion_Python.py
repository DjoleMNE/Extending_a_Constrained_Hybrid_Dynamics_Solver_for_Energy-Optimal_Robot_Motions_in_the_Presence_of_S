# Code based on MRC assignment in SS2017, MAS-HBRS. Authors: Santosh Thoduka and Aleksandar Mitrevski
# Modified and applied by Djordje Vukcevic
import numpy as np
import matplotlib.pyplot as plt
import numpy.linalg as linalg
import sympy as sp

from IPython.display import display
from IPython.core.pylabtools import figsize, getfigs
import IPython

L1 = 0.4
L2 = 0.4

theta1, theta2 = sp.symbols('theta1, theta2')

# Equations for end-effector position 
eq1 = L1*sp.cos(theta1) + L2*sp.cos(theta1 + theta2)
eq2 = L1*sp.sin(theta1) + L2*sp.sin(theta1 + theta2)

# Equations for link 1 tip position
eq3 = L1*sp.cos(theta1)
eq4 = L1*sp.sin(theta1)

# initial joint angles
theta1_initial = 0
theta2_initial = np.pi/6.0

cur_t1 = theta1_initial
cur_t2 = theta2_initial

input_matrix = np.loadtxt("joint_poses.txt", dtype='float')
# print(len(input_matrix))

# initial position of link 2
cur_l2_x = eq1.evalf(subs={theta1:cur_t1, theta2:cur_t2})
cur_l2_y = eq2.evalf(subs={theta1:cur_t1, theta2:cur_t2})

# initial position of link 1
cur_l1_x = eq3.evalf(subs={theta1:cur_t1})
cur_l1_y = eq4.evalf(subs={theta1:cur_t1})

# store positions of link 1 and link 2
positions_l2 = []
positions_l1 = []
positions_l2.append([cur_l2_x, cur_l2_y])
positions_l1.append([cur_l1_x, cur_l1_y])
plt.cla()

# plot the links
plt.plot([0, cur_l1_x], [0, cur_l1_y])
plt.plot([cur_l1_x, cur_l2_x], [cur_l1_y, cur_l2_y])

#Define edges of axis
plt.axis([-2, 2, -1, 1])
# plt.axis('square')

for i in range(len(input_matrix)):

    # update current joint angles
    cur_t1 = input_matrix[i][0]+3*np.pi/2
    cur_t2 = input_matrix[i][1]

    # find updated Cartesian positions of both links
    cur_l2_x = eq1.evalf(subs={theta1:cur_t1, theta2:cur_t2})
    cur_l2_y = eq2.evalf(subs={theta1:cur_t1, theta2:cur_t2})

    cur_l1_x = eq3.evalf(subs={theta1:cur_t1})
    cur_l1_y = eq4.evalf(subs={theta1:cur_t1})

    # plot links
    positions_l2.append([cur_l2_x, cur_l2_y])
    positions_l1.append([cur_l1_x, cur_l1_y])

    plt.plot([0, cur_l1_x], [0, cur_l1_y])
    plt.plot([cur_l1_x], [cur_l1_y], 'bo')

    plt.plot([cur_l1_x, cur_l2_x], [cur_l1_y, cur_l2_y])
    plt.plot([cur_l1_x, cur_l2_x], [cur_l1_y, cur_l2_y], 'go')

    #pause the simulation step
    plt.pause(0.1)

plt.show()
