#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

import numpy as np
from math import pi as PI

import utilities.coverage_giorgio as cov
from utilities.utility_functions import unit_vec

import pickle

"""CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
filename = "./lmks_test.txt"
file_object = open(filename,'w+')

q_0 = [-3.,-1.,0.]
q_1 = [-2.,-1.,0.]
q_2 = [-1.,-1.,0.]
psi_0 = PI/2.
theta_0 = PI/6.
psi_1 = PI/3.
theta_1 = -PI/4.
psi_2 = PI/3.
theta_2 = 0.
u_0 = unit_vec(psi_0, theta_0)
u_1 = unit_vec(psi_1, theta_1)
u_2 = unit_vec(psi_2, theta_2)
lm_0 = cov.Landmark(q_0,u_0)
lm_1 = cov.Landmark(q_1,u_1)
lm_2 = cov.Landmark(q_2,u_2)
lmk_list = cov.Landmark_list([lm_0,lm_1,lm_2])

pickle.dump(lmk_list,file_object)

# Test
#file_object.close()
#file_object = open(filename)
#print pickle.load(file_object)