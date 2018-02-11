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

input_data = np.loadtxt("plot_acc.txt", dtype='double', delimiter=' ')
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
plt.plot(df1.x, intercept1 + slope1*df1.x, 'r', label='Line: y={0:.1f}x + {1:.1f}'.format(slope1,intercept1))

# sns.set_style('ticks')
#
# ax1 = sns.regplot(df1.x, df1.y, line_kws={'label':"y={0:.1f}x+{1:.1f}".format(slope1,intercept1)})
plt.title('1 DOF Robot', fontsize=25)
plt.xlabel(r'Friction torque $[Nm]$', fontsize=25)
plt.ylabel(r'Joint accelerations $[\frac{Nm}{s^2}]$', fontsize=25)
plt.xlim(xmin=-20, xmax = 20)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(prop={'size': 18})
# plt.axis('equal')
plt.grid()

plt.show()
