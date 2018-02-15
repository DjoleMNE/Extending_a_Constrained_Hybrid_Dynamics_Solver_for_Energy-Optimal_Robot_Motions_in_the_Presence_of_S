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

input_data = np.loadtxt("plot_control.txt", dtype='double', delimiter=' ')
rows = input_data.shape[0]

# Make data.
tau_1 = input_data[:,0]
acc_1 = input_data[:,1]

# plot 1
# ====================================

# plt.plot([1,2,3])
# plt.subplot(121)
df1 = pd.DataFrame({'x': tau_1, 'y': acc_1})
slope1, intercept1, r_value1, p_value1, std_err1 = stats.linregress(df1.x,df1.y)
plt.plot(df1.x, df1.y,"o",label='_nolegend_')
# plt.plot(df1.x, intercept1 + slope1*df1.x, 'r', label='Line: y={0:.1f}x + {1:.1f}'.format(slope1,intercept1))
plt.plot(df1.x, intercept1 + slope1*df1.x, 'r', label='_nolegend_')
y_constant = np.full(input_data.shape[0], 10)
x_constant = np.full(input_data.shape[0], -1)
new_tau = tau_1[::-1]
new_acc = acc_1[::-1]
plt.plot(y_constant, new_acc[:y_constant.shape[0]],label='Optimum value', c='green')
plt.plot(new_tau[:x_constant.shape[0]], x_constant,label='_nolegend_', c='green')
# sns.set_style('ticks')
#
# ax1 = sns.regplot(df1.x, df1.y, line_kws={'label':"y={0:.1f}x+{1:.1f}".format(slope1,intercept1)})
plt.title('Resulting joint torque computed by the solver \n' + r'External force = -11 $N$', fontsize=25)
plt.xlabel(r'Friction torque $[Nm]$', fontsize=25)
plt.ylabel(r'Resulting joint torque $[{Nm}]$', fontsize=25)
plt.xlim(xmin = -10.3, xmax = 10.5)
plt.xticks(fontsize=18)
y_lim = 0
y_ax = np.arange(-21,y_lim,1)
x_ax = np.arange(-10,11,1)
plt.yticks(y_ax)
plt.xticks(x_ax)
plt.yticks(fontsize=18)
plt.legend(prop={'size': 17})
# plt.axis('equal')
plt.grid()

plt.show()
