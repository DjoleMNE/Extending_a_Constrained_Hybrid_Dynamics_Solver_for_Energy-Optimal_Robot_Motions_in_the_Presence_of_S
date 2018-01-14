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
tau_2 = input_data[:-1,2]
acc_2 = input_data[:-1,3]

# plot 2
# ====================================
df2 = pd.DataFrame({'x': tau_2, 'y': acc_2})
slope2, intercept2, r_value2, p_value2, std_err2 = stats.linregress(df2.x,df2.y)
plt.title('Joint 2', fontsize = 25)
plt.plot(df2.x, df2.y, 'o', label='Raw data', c='g')
plt.plot(df2.x, intercept2 + slope2*df2.x, 'r', label='Line: y={0:.1f}x + {1:.1f}'.format(slope2,intercept2))

plt.xlabel(r'Friction torque $[Nm]$', fontsize=25)
plt.ylabel(r'Joint accelerations $[\frac{Nm}{s^2}]$', fontsize=25)
plt.xlim(xmin=-50, xmax = 50)
plt.legend( prop={'size': 18})
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.grid()
plt.show()
