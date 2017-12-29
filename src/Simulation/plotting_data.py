# Author(s): Djordje Vukcevic

import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from matplotlib import cm
import pandas as pd
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

input_data = np.loadtxt("plot_data.txt", dtype='double', delimiter=' ')
rows = input_data.shape[0]

# Make data.
X = input_data[:-1,0]
xlen = len(X)
Y = input_data[:-1,1]
ylen = len(Y)
Z = input_data[:-1,2]

df = pd.DataFrame({'x': X, 'y': Y, 'z': Z})

fig = plt.figure()
ax = Axes3D(fig)

surf = ax.plot_trisurf(df.x, df.y, df.z, cmap = cm.jet, linewidth = 0.1)
fig.colorbar(surf, shrink=0.5, aspect=5)

ax.set_xlabel(r'Friction torque 1 - [$Nm$]', fontsize=12)
ax.set_ylabel(r'Friction torque 2 - [$Nm$]', fontsize=12)
ax.set_zlabel(r'Acceleration energy - [$\frac{Nm}{s^2}$]', fontsize=12)

class Arrow3D(FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

a = Arrow3D([input_data[-1,0], input_data[-1,0]], [input_data[-1,1], input_data[-1,1]], [input_data[-1,2], input_data[-1,2]+input_data[-1,2]/10], mutation_scale=20,  lw=1, arrowstyle="-|>", color="k")
ax.add_artist(a)

ax.text(input_data[-1,0],input_data[-1,1], (input_data[-1,2] +input_data[-1,2]/10),  'Max: %d [%d, %d]' % (input_data[-1,2], input_data[-1,0], input_data[-1,1]), size=15, zorder=1, color='blue')

plt.savefig('acc_energy_surface.pdf')
plt.show()
