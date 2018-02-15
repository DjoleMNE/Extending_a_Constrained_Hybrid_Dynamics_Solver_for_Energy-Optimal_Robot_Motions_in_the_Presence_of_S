# Author(s): Djordje Vukcevic

import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from matplotlib import cm
import pandas as pd
from scipy import stats
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

input_data1 = np.loadtxt("plot_data.txt", dtype='double', delimiter=' ')
input_data2 = np.loadtxt("plot_acc.txt", dtype='double', delimiter=' ')
input_data3 = np.loadtxt("plot_acc_true.txt", dtype='double', delimiter=' ')
input_data4 = np.loadtxt("plot_data_true.txt", dtype='double', delimiter=' ')

# Make data.
X = input_data1[:,0]
Z = input_data1[:,1]
Y = input_data2[:,1]

df = pd.DataFrame({'x': X, 'y': Y, 'z': Z})

fig = plt.figure()
ax = Axes3D(fig)

surf = ax.plot_trisurf(df.x, df.y, df.z, cmap = cm.jet, linewidth = 0.1)
fig.colorbar(surf, shrink=0.5, aspect=5)

# Make data.
tau_1 = input_data3[:,0]

acc_1 = input_data3[:,1]
tau_line = input_data4[:-1,0]
z_line = input_data4[:-1,1]
z_constant = np.full(input_data3.shape[0], -50)

df1 = pd.DataFrame({'x': tau_1, 'y': acc_1})
slope1, intercept1, r_value1, p_value1, std_err1 = stats.linregress(df1.x,df1.y)
# ax.plot(df1.x, intercept1 + slope1*df1.x, z_constant, 'r', linewidth=4.0, label='_nolegend_',c='orange')
# ax.plot(df1.x, intercept1 + slope1 * df1.x, z_constant, 'r', linewidth=4.0, label=r'Line: $\ddot{q}$ ={0:.1f}$\tau$ + {1:.1f}'.format(slope1,intercept1))

# calculate polynomial for minimums
min_energy = np.polyfit(tau_line, z_line, 2)
gauss = np.poly1d(min_energy)

# calculate new x's and y's
x_new = np.linspace(tau_line[0], tau_line[-1], 41)
y_new = gauss(x_new)


# ax.plot(tau_line,z_line,'o', x_new, y_new)
ax.plot(df1.x, df1.y, z_line,".", c='red', markersize=5, label='_nolegend_')
# ax.plot(df1.x, df1.y, z_line,".", label=r'Polynomial: $Z$={0:.1f}$\tau^2$ + {1:.1f}$\tau$ + {2:.1f}'.format(min_energy[0],min_energy[1],min_energy[2]))

ax.set_xlabel(r'Friction torque $[Nm]$', labelpad = 20, fontsize = 18)
ax.set_ylabel(r'Joint Acceleration $[\frac{rad}{s^2}]$', labelpad = 20, fontsize=18)
ax.set_zlabel(r'Acceleration energy $[\frac{Nm}{s^2}]$', labelpad = 20, fontsize=18)
# ax.legend(prop={'size': 18})

plt.show()
