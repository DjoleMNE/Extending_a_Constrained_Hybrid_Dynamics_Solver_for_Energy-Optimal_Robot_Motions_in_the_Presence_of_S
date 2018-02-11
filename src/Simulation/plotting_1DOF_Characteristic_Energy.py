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
z_line = np.full((input_data3.shape[0]),-50)

df1 = pd.DataFrame({'x': tau_1, 'y': acc_1})
slope1, intercept1, r_value1, p_value1, std_err1 = stats.linregress(df1.x,df1.y)
ax.plot(df1.x, df1.y, z_line,"o",label='_nolegend_')
ax.plot(df1.x, intercept1 + slope1*df1.x, z_line, 'r', label='Line: y={0:.1f}x + {1:.1f}'.format(slope1,intercept1))

ax.set_xlabel(r'Friction torque $[Nm]$', labelpad = 20, fontsize=18)
ax.set_ylabel(r'Joint Acceleration $[\frac{rad}{s^2}]$', labelpad = 20, fontsize=18)
ax.set_zlabel(r'Acceleration energy $[\frac{Nm}{s^2}]$', labelpad = 20, fontsize=18)
ax.legend(prop={'size': 18})

plt.show()
