# Author(s): Djordje Vukcevic

import numpy as np
import sympy as sp
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from matplotlib import cm
from scipy import stats
import pandas as pd
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import seaborn as sns

input_data = np.loadtxt("plot_data.txt", dtype='double', delimiter=' ')
rows = input_data.shape[0]

# Make data.
tau_1 = input_data[:-1,0]
acc_1 = input_data[:-1,1]

# plot 1
# ====================================

# plt.plot([1,2,3])
# plt.subplot(121)
df1 = pd.DataFrame({'x': tau_1, 'y': acc_1})
slope1, intercept1, r_value1, p_value1, std_err1 = stats.linregress(df1.x,df1.y)
plt.plot(df1.x, df1.y, 'o',label='_nolegend_')
y_constant = np.full(input_data.shape[0]-1, -10)
x_constant = np.full(input_data.shape[0]-1, 78)
new_x = tau_1[::-1]
plt.plot(y_constant, acc_1,label='Optimum value', linewidth=2.0, c='green')
# plt.plot(new_x[:x_constant.shape[0]], x_constant, linewidth=2.0, label='_nolegend_', c='green')
# sns.set_style('ticks')
#
# ax1 = sns.regplot(df1.x, df1.y, line_kws={'label':"y={0:.1f}x+{1:.1f}".format(slope1,intercept1)})
plt.title(r'Linear cost function', fontsize=25)
plt.xlabel(r'Friction torque $[Nm]$', fontsize=25)
plt.ylabel(r'Accelerations energy $[\frac{Nm}{s^2}]$', fontsize=25)
plt.xlim(xmin = -10.3, xmax = 10.5)
plt.ylim(ymin = -25)

x_ax = np.arange(-10,11,1)
plt.xticks(x_ax)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
# plt.legend(bbox_to_anchor = (1,0.5), loc="center right", fontsize=12,
           # bbox_transform = plt.gcf().transFigure)
plt.legend(prop={'size': 17}, loc=8)
# plt.tight_layout()
# plt.axis('equal')
plt.grid()

plt.show()
