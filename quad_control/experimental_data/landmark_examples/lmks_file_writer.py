#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

import numpy as np
from math import pi

from utilities.coverage_giorgio import Landmark, Landmark_list
from utilities.utility_functions import unit_vec, unit_vec_from_lat_long

import pickle

"""CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
filename_0 = "./cube_3quads_list_0.txt"
file_object_0 = open(filename_0,'w+')
filename_1 = "./cube_3quads_list_1.txt"
file_object_1 = open(filename_1,'w+')
filename_2 = "./cube_3quads_list_2.txt"
file_object_2 = open(filename_2,'w+')


l = 0.5
u_l = unit_vec_from_lat_long(-pi/2,0)
u_r = unit_vec_from_lat_long(pi/2,0)
u_u = unit_vec_from_lat_long(0,pi/2)
u_b = unit_vec_from_lat_long(pi,0)
u_f = unit_vec_from_lat_long(0,0)
lmk_l_0 = Landmark([0., -l/2, l/2],u_l)
lmk_l_1 = Landmark([-l/3,-l/2, 3./4.*l],u_l)
lmk_l_2 = Landmark([-l/4, -l/2,  l/3],u_l)
lmk_r_0 = Landmark([0., l/2, l/2],u_r)
lmk_r_1 = Landmark([-l/4, l/2, 3/4*l],u_r)
lmk_r_2 = Landmark([-l/3, l/2, l/4 ],u_r)
lmk_u_0 = Landmark([0., 0., l],u_u)
lmk_u_1 = Landmark([-l/3, -l/3, l],u_u)
lmk_u_2 = Landmark([-l/4, l/4, l],u_u)
lmk_b_0 = Landmark([-l/2, 0., l/2],u_b)
lmk_b_1 = Landmark([-l/2, l/3, l/4],u_b)
lmk_b_2 = Landmark([-l/2, l/4, 2./3.*l],u_b)
lmk_f_0 = Landmark([l/2, 0., l/2],u_f)
lmk_f_1 = Landmark([l/2, l/3, 3./4.*l],u_f)
lmk_f_2 = Landmark([l/2, l/4, 2./3.*l],u_f)
Q_0 = Landmark_list([lmk_l_0,lmk_r_0,lmk_u_0,lmk_b_0,lmk_f_0])
Q_1 = Landmark_list([lmk_l_1,lmk_r_1,lmk_u_1,lmk_b_1,lmk_f_1])
Q_2 = Landmark_list([lmk_l_2,lmk_r_2,lmk_u_2,lmk_b_2,lmk_f_2])

pickle.dump(Q_0,file_object_0)
pickle.dump(Q_0,file_object_1)
pickle.dump(Q_0,file_object_2)

# """CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
# filename = "./lmks_test.txt"
# file_object = open(filename,'w+')

# q_0 = [-3.,-1.,0.]
# q_1 = [-2.,-1.,0.]
# q_2 = [-1.,-1.,0.]
# psi_0 = PI/2.
# theta_0 = PI/6.
# psi_1 = PI/3.
# theta_1 = -PI/4.
# psi_2 = PI/3.
# theta_2 = 0.
# u_0 = unit_vec(psi_0, theta_0)
# u_1 = unit_vec(psi_1, theta_1)
# u_2 = unit_vec(psi_2, theta_2)
# lm_0 = cov.Landmark(q_0,u_0)
# lm_1 = cov.Landmark(q_1,u_1)
# lm_2 = cov.Landmark(q_2,u_2)
# lmk_list = cov.Landmark_list([lm_0,lm_1,lm_2])

# pickle.dump(lmk_list,file_object)

# Test
#file_object.close()
#file_object = open(filename)
#print pickle.load(file_object)