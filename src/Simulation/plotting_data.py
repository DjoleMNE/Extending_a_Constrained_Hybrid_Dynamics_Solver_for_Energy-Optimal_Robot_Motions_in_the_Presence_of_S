import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import matplotlib.path as mpath

input_data = np.loadtxt("plot_data.txt", dtype='double', delimiter=' ')
# print(input_data.shape[0])
rows = input_data.shape[0]

fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.
X = input_data[:-1,0]
xlen = len(X)
Y = input_data[:-1,1]
ylen = len(Y)
X, Y = np.meshgrid(X, Y)
Z = input_data[:-1,2]


# Create an empty array of strings with the same shape as the meshgrid, and
# populate it with two colors in a checkerboard pattern.
# colortuple = ('r', 'b')
# colors = np.empty(X.shape, dtype=str)
#
# for y in range(ylen):
#     for x in range(xlen):
#         colors[x, y] = colortuple[(x + y) % len(colortuple)]

# Plot the surface with face colors taken from the array we made.
surf = ax.plot_surface(X, Y, Z, linewidth=0)
# surf = ax.plot_surface(X, Y, Z, facecolors=colors, linewidth=0)

# Customize axes.
ax.set_zlim(0, input_data[-1,2]+100)
ax.set_xlim(-45, 45)
ax.set_ylim(-25, 25)
ax.set_xlabel('Tau_0')
ax.set_ylabel('Tau_1')
ax.set_zlabel('Acceleration energy')

ax.w_zaxis.set_major_locator(LinearLocator(6))

# Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

# circle1 = plt.Circle((input_data[-1,0], input_data[-1,1]), 2, color='r')

# ax.add_artist(circle1)
plt.show()
