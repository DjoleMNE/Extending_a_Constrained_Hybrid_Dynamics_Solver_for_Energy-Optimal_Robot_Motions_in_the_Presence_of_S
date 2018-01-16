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

input_data1 = np.loadtxt("plot_data.txt", dtype='double', delimiter=' ')
input_data2 = np.loadtxt("plot_acc.txt", dtype='double', delimiter=' ')

# Make data.
X = input_data1[:-1,0]
Z = input_data1[:-1,1]
Y = input_data2[:,1]

df = pd.DataFrame({'x': X, 'y': Y, 'z': Z})

fig = plt.figure()
ax = Axes3D(fig)

surf = ax.plot_trisurf(df.x, df.y, df.z, cmap = cm.jet, linewidth = 0.1)
fig.colorbar(surf, shrink=0.5, aspect=5)

ax.set_xlabel(r'Friction torque $[Nm]$', fontsize=18)
ax.set_ylabel(r'Joint Acceleration $[\frac{rad}{s^2}]$', fontsize=18)
ax.set_zlabel(r'Acceleration energy $[\frac{Nm}{s^2}]$', fontsize=18)

class Arrow3D(FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

plt.show()
