import rospy
from geometry_msgs.msg import Twist
import sys, select, os
  
import time

from utility import *
import numpy as np

import pygame


### Pygame stuff
pygame.init()

SCREEN_WIDTH = 60
SCREEN_HEIGHT = 40

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

### ROS stuff
init_pos = get_position()
log = [get_actual_position(init_pos)[0:2]]


LIN_VEL_LIMIT = 2.0 # m/s
ANG_VEL_LIMIT = 1.2 # rads
LIN_VEL_STEP  = 0.2
ANG_VEL_STEP  = 0.1

lin_vel = 0.0
ang_vel = 0.0

rospy.init_node('husky_teleop_keyboard')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

###ROS functions
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def bound_lin_vel(lin_vel_cmd):
    lin_vel_cmd = constrain(lin_vel_cmd, -LIN_VEL_LIMIT, LIN_VEL_LIMIT)
    return lin_vel_cmd

def bound_ang_vel(ang_vel_cmd):
    ang_vel_cmd = constrain(ang_vel_cmd, -ANG_VEL_LIMIT, ANG_VEL_LIMIT)
    return ang_vel_cmd

def generate_cmd_vel_msg(lin_vel_cmd, ang_vel_cmd):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x  = lin_vel_cmd
    cmd_vel_msg.linear.y  = 0.0
    cmd_vel_msg.linear.z  = 0.0
    cmd_vel_msg.angular.x = 0.0
    cmd_vel_msg.angular.y = 0.0
    cmd_vel_msg.angular.z = ang_vel_cmd
    return cmd_vel_msg


vel_log = []
run = True
while run:
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        lin_vel = bound_lin_vel(lin_vel + LIN_VEL_STEP)
    elif keys[pygame.K_s]:# keyboard.is_pressed('s'):
        lin_vel = bound_lin_vel(lin_vel - LIN_VEL_STEP)
    else:
        lin_vel = 0.25*lin_vel
    if keys[pygame.K_a]: #keyboard.is_pressed('a'):
        ang_vel = bound_ang_vel(ang_vel + ANG_VEL_STEP)
    elif keys[pygame.K_d]: #keyboard.is_pressed('d'):
        ang_vel = bound_ang_vel(ang_vel - ANG_VEL_STEP)
    else:
        ang_vel = 0.25*ang_vel
        
    cmd_vel_msg = generate_cmd_vel_msg(lin_vel, ang_vel)
    vel_log.append(lin_vel)
    np.save('VL.npy',vel_log)
    cmd_vel_pub.publish(cmd_vel_msg)
    pos = get_actual_position(init_pos)
    log.append(pos[0:2])
    np.save('log.npy',np.array(log))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False


pygame.quit()
