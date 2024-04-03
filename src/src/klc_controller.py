import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from math import log
from obstacle import *
import rospy
from Plants.uniform_plant import *
#from Plants.linearized_plant import *
#from Plants.trajectory_based_plant import *
import time
from pylab import rcParams
from utility import *
from cost_cache import *

from nav_msgs.msg import Odometry, OccupancyGrid

"""
ControllerKLC: A class implementing a Kinematic Linearization Controller (KLC) for robot motion planning.

This class defines methods to initialize the controller and update its behavior based on robot states.
"""
class ControllerKLC:

    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def __init__(self, goal, mode):

        self.cache = CostCache()
        self.mode = mode
        

        rospy.init_node('husky', anonymous=True)

        # Get the trasformation between odom and world
        self.init_position = get_position()
        self.cache.set_T(self.init_position)

        self.obstacles = Obstacle()
        
        self.poses = []

        #Target definition
        self.goal = goal
        self.xd = goal[0]
        self.yd = goal[1]
        
        #Dimensions of the variables
        self.zdim = 2

        #Minimum values
        self.zmin = [0, 0] 

        #Discretization steps
        self.zstep = [0.3, 0.3]

        #Amount of discrete bins
        self.zdiscr = [30, 30]

        #Number of iterations for the simulations
        #self.zsim = 15
        #Duration of the simulation
        self.duration = 40
        self.cbase = np.zeros((self.zdiscr[0],self.zdiscr[1]))
        #self.cbase[16,16]=-100
        #for i in range(self.zdiscr[0]):
        #    for j in range(self.zdiscr[1]):
        #        self.cbase[i,j] = 0.175*((i-16)**2 + (j-16)**2)
        
        self.cost = self.cbase

        # Creazione del vettore 4D inizializzato con zeri

        self.passive_dynamics = np.zeros((self.zdiscr[0], self.zdiscr[0], self.zdiscr[0], self.zdiscr[0]))

        #if mode == 0:
            #self.passive_dynamics = uniform_plant().get_plant(self.zdiscr[0])
            #self.passive_dynamics = np.load('./dynamics/both_dyna.npy')#np.load('./dynamics/dyna_3_1.npy')    


        self.stateVect = np.zeros((self.zdiscr[0]**2, 2))
        self.discStates = np.zeros((self.zdiscr[0]**2, 2))

        for i in range(self.zdiscr[0]):
            #Enumerate the states from 1 to 36^2. Here we explicitly build the enumeration to later build the Q matrix and for convenience
            for j in range(self.zdiscr[0]):
                # Compute the angle and speed values for the current state
                x = (i)*self.zstep[0]
                y = (j)*self.zstep[0]
                # Calculate the index of the current state in the state vector
                ind = i*self.zdiscr[0] + j
                # Assign the angle and speed values to the state vector
                self.stateVect[ind] = [x, y] 
                self.discStates[ind]=[i,j]

        self.Prob = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2))

        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                pf = self.passive_dynamics[i,j]
                ind1 = i*self.zdiscr[0] + j
                self.Prob[ind1] = self.unravelPF(pf)

        self.z = np.array((np.shape(self.Prob))[0])
        
    
    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def update(self):
        current_position = get_actual_position(self.init_position)
        state = [3,0]#self.discretize(current_position, self.zdim, self.zmin, self.zstep) #INDEX
        nSteps = self.duration
        self.poses.append(state)
        np.save('poses.npy',self.poses)

        diagMinusQ = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2)) # q
        hist = [[0,0]]*nSteps
        
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[1]):
                k = i*self.zdiscr[0]+j
                diagMinusQ[k,k] = np.exp(-self.cost[i,j])

        self.z = self.power_method(diagMinusQ@self.Prob, self.zdiscr[0]**2)
                
        for t in range(nSteps):
            print(t)
            hist[t]=self.stateVect[int(state[0])*self.zdiscr[0]+int(state[1])] #Log the state
            state = self.loop(state) #Sample the new state
        waypoints_x = [pt[0] for pt in hist]
        waypoints_y = [pt[1] for pt in hist]
        print(waypoints_x,waypoints_y)
        self.cache.set_next_target(waypoints_x,waypoints_y)

    
    # Utility methods for init and update methods


    """
    Discretize the continuous state variables.
    
    :param Z: The continuous state variables.
    :param Zdim: The dimensionality of the variables.
    :param Zmin: The minimum values for each dimension.
    :param Zstep: The discretization steps for each dimension.
    :return: The discretized indices.
    """
    def discretize(self, Z, Zdim, Zmin, Zstep):
        res = [0]*Zdim #n-dimensional index
        for i in range(Zdim): #For each dimension
            elt = Z[i] #Extract the i-th element
            ind = int((elt - Zmin[i])//Zstep[i]) #Discretize
            res[i] = ind
        return(tuple(res)) #Return as tuple for array indexing
    
    
    """
    Calculate the cost of a given state.
    
    :param state: The state to calculate the cost for.
    :return: The calculated cost.
    """
    #def cost(self, state):
    #    xind = self.discretize(state, self.zdim, self.zmin, self.zstep)
    #    return(self.cost[xind])    
    
    """
    Unravel a 2D passive dynamics array into a 1D array.
    
    :param pf: The 2D passive dynamics array.
    :return: The unraveled 1D array.
    """
    def unravelPF(self, pf):
    
        res = np.zeros(self.zdiscr[0]**2)
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                res[i*self.zdiscr[0]+j] = pf[i][j]
        return(res)
    
    
    """
    Perform the power method for eigenvalue estimation.
    
    :param mat: The matrix for eigenvalue estimation.
    :param dim: The dimensionality of the matrix.
    :return: The estimated eigenvector.
    """
    def power_method(self, mat, dim, epsilon=1e-6):
        vect = np.ones(dim)
        nrm = np.linalg.norm(vect)
        
        for i in range(50):
            #prev_vect = vect.copy() 
            vect = mat.dot(vect)
            nrm = np.linalg.norm(vect)
            vect = vect / nrm
            
            """# Calcola la differenza tra l'autovettore attuale e quello precedente
            diff = np.linalg.norm(vect - prev_vect)
            
            # Verifica la condizione di arresto
            if diff < epsilon:
                break"""
        return vect

    
    """
    Perform the power method for eigenvalue estimation.
    
    :param mat: The matrix for eigenvalue estimation.
    :param dim: The dimensionality of the matrix.
    :return: The estimated eigenvector.
    """
    def loop(self, x):
        ind = (int(x[0]),int(x[1]))
        #ind = self.discretize(x,  self.zdim, self.zmin, self.zstep) #Discretize the state
        pf = self.passive_dynamics[ind[0],ind[1]] #Get the pf corresponding to the passive dynamics
        pf_1D = self.unravelPF(pf) #Unravel it
        pf_weighted = pf_1D*self.z #Calculate the actual transition pf using z and the passive dynamics
        S = np.sum(pf_weighted) #Normalize
        pf_weighted = pf_weighted/S #probabilities contain NaN ERRORE SPESSO USCITO FUORI forse perché non si riesce a minimizzare la funzione di costo a causa di qualche limite raggiunto
        ind = np.random.choice(range(self.zdiscr[0]**2), p=pf_weighted) #Get the new (enumerated) state index using the calculated dynamics
        newState = self.discStates[ind]#self.stateVect[ind] #Get the new state from the state vector
        return(newState)
    

	

    def export_metrics(self, x, y, time):
        np.save("klc_online_real_planning_" + str(self.mode), np.array([x, y, time]))

