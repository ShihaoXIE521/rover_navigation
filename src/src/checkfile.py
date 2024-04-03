from mpc_controller_1_1 import *
from cost_cache import *
from klc_controller import *
from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt


targetx1_1 = 0.3*np.load('./waypoints2/wx_1_1.npy')
targety1_1 = 0.3*np.load('./waypoints2/wy_1_1.npy')

targetx1_2 = 0.3*np.load('./waypoints2/wx_1_2.npy')
targety1_2 = 0.3*np.load('./waypoints2/wy_1_2.npy')
targetx1_2 = np.append(targetx1_2,[targetx1_2[-1],targetx1_2[-1],targetx1_2[-1],targetx1_2[-1],targetx1_2[-1]])
targety1_2 = np.append(targety1_2,[targety1_2[-1],targety1_2[-1],targety1_2[-1],targety1_2[-1],targety1_2[-1]])

targetx1_3 = 0.3*np.load('./waypoints2/wx_1_3.npy')
targety1_3 = 0.3*np.load('./waypoints2/wy_1_3.npy')

targetx2_1 = 0.3*np.load('./waypoints2/wx_2_1.npy')
targety2_1 = 0.3*np.load('./waypoints2/wy_2_1.npy')

targetx2_2 = 0.3*np.load('./waypoints2/wx_2_2.npy')
targety2_2 = 0.3*np.load('./waypoints2/wy_2_2.npy')

targetx2_3 = 0.3*np.load('./waypoints2/wx_2_3.npy')
targety2_3 = 0.3*np.load('./waypoints2/wy_2_3.npy')

targetx3_1 = 0.3*np.load('./waypoints2/wx_3_1.npy')
targety3_1 = 0.3*np.load('./waypoints2/wy_3_1.npy')

targetx3_2 = 0.3*np.load('./waypoints2/wx_3_2.npy')
targety3_2 = 0.3*np.load('./waypoints2/wy_3_2.npy')

targetx3_3 = 0.3*np.load('./waypoints2/wx_3_3.npy')
targety3_3 = 0.3*np.load('./waypoints2/wy_3_3.npy')


#length = targetx.shape[0]
#cost = np.load('./cost.npy')

#plt.plot(targetx,targety)
#plt.show()

print('target1_1 is ', targetx1_1, targety1_1[-1])
print('target1_2 is ', targetx1_2, targety1_2)
print('target1_3 is ', targetx1_3[-1], targety1_3[-1])
print('target2_1 is ', targetx2_1[-1], targety2_1[-1])
print('target2_2 is ', targetx2_2[-1], targety2_2[-1])
print('target2_3 is ', targetx2_3[-1], targety2_3[-1])
print('target3_1 is ', targetx3_1[-1], targety3_1[-1])
print('target3_2 is ', targetx3_2[-1], targety3_2[-1])
print('target3_3 is ', targetx3_3[-1], targety3_3[-1])



