import rospy
from mpc_controller_1_3 import *
from cost_cache import *
from klc_controller import *

cache = CostCache()

target = [2.13,6.74] #8,8
mode = 0
klc = ControllerKLC(target, mode)

cache.set_next_target(np.load('./waypoints2/wx_1_3.npy'),np.load('./waypoints2/wy_1_3.npy')) #Waypoint folder in ./catkin_ws/src/husky_mpc_datadriven_src/
#Change line above to set waypoints

mpc = ControllerMPC([0,0,0])

mpc_x_history = np.array([])
mpc_y_history = np.array([])
mpc_t_history = np.array([])
mpc_t = 0

while not rospy.is_shutdown():   
    mpc_x, mpc_y, stopping_cond = mpc.update(target)
    mpc_x_history = np.append(mpc_x_history, mpc_x)
    mpc_y_history = np.append(mpc_y_history, mpc_y)
    mpc_t += 0.1
    mpc_t_history = np.append(mpc_t_history, mpc_t)
    if stopping_cond == 1:
        break
        
u1, u2 = mpc.get_inputs()
np.save('u1.npy',u1)
np.save('u2.npy',u2)
