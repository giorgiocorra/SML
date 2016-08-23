#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

import numpy as np
from math import pi

from utilities.coverage_giorgio import Landmark, Landmark_list
from utilities.utility_functions import unit_vec, unit_vec_from_lat_long, GetRotFromEulerAngles

import pickle

# """CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
# package_data_path = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/'
# filename = package_data_path + "./sim_1quad_real.txt"
# file_object = open(filename,'w+')

# x = 1.6
# y = -2.0
# z = 0.7
# roll = -1. *pi/180.0
# pitch = 7. *pi/180.0
# yaw = 0 *pi/180.0
# R = GetRotFromEulerAngles([roll,pitch,yaw])
# q_0 = [x,y, z+0.15]
# q_1 = [x-0.05, y, z+0.1]
# q_2 = [x-0.05, y, z-0.1]
# q_3 = [x, y-0.05, z]
# q_4 = [x+0.05, y, z+0.05]
# u_0 = R.dot([0,0,1])
# u_1 = R.dot([0,1,0])
# u_2 = R.dot([0,1,0])
# u_3 = R.dot([-1,0,0])
# u_4 = R.dot([0,-1,0])
# lm_0 = Landmark(q_0,u_0)
# lm_1 = Landmark(q_1,u_1)
# lm_2 = Landmark(q_2,u_2)
# lm_3 = Landmark(q_3,u_3)
# lm_4 = Landmark(q_4,u_4)
# lmk_list = Landmark_list([lm_0,lm_1,lm_2,lm_3,lm_4])

# pickle.dump(lmk_list,file_object)


"""CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
filename_0 = "./cube_3quads_real_list_0.txt"
file_object_0 = open(filename_0,'w+')
filename_1 = "./cube_3quads_real_list_1.txt"
file_object_1 = open(filename_1,'w+')
filename_2 = "./cube_3quads_real_list_2.txt"
file_object_2 = open(filename_2,'w+')


u_l = unit_vec_from_lat_long(-pi/2,0)
u_r = unit_vec_from_lat_long(pi/2,0)
u_u = unit_vec_from_lat_long(0,pi/2)
u_b = unit_vec_from_lat_long(pi,0)
u_f = unit_vec_from_lat_long(0,0)
l_x = 0.5
l_y = 0.5
l_z = 0.25
z = 1.
lmk_l_0 = Landmark([0.,		-l_y/2, z],u_l)
lmk_l_1 = Landmark([-l_x/2,	-l_y/2, z],u_l)
lmk_l_2 = Landmark([l_x/2, 	-l_y/2, z],u_l)
lmk_r_0 = Landmark([0., 	l_y/2, 	z],u_r)
lmk_r_1 = Landmark([l_x/2, 	l_y/2, 	z],u_r)
lmk_r_2 = Landmark([-l_x/2,	l_y/2, 	z],u_r)
lmk_u_0 = Landmark([0., 	0., 	z],u_u)
lmk_u_1 = Landmark([0., -l_y/4,	z],u_u)
lmk_u_2 = Landmark([0., 	l_y/4, 	z],u_u)
lmk_b_0 = Landmark([-l_x/2, 0., 	z],u_b)
lmk_b_1 = Landmark([-l_x/2, -l_y/2, z],u_b)
lmk_b_2 = Landmark([-l_x/2, l_y/2, 	z],u_b)
lmk_f_0 = Landmark([l_x/2, 	0., 	z],u_f)
lmk_f_1 = Landmark([l_x/2, 	l_y/2, 	z],u_f)
lmk_f_2 = Landmark([l_x/2, 	-l_y/2, z],u_f)
Q_0 = Landmark_list([lmk_l_0,lmk_r_0,lmk_u_0,lmk_b_0,lmk_f_0])
Q_1 = Landmark_list([lmk_l_1,lmk_r_1,lmk_u_1,lmk_b_1,lmk_f_1])
Q_2 = Landmark_list([lmk_l_2,lmk_r_2,lmk_u_2,lmk_b_2,lmk_f_2])

pickle.dump(Q_0,file_object_0)
pickle.dump(Q_1,file_object_1)
pickle.dump(Q_2,file_object_2)

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
# file_object.close()
# file_object = open(filename)
# print pickle.load(file_object)

# """CHANGE FILE NAME IF YOU DON'T WANT TO OVERWRITE"""
# filename = "./collav_test.txt"
# file_object = open(filename,'w+')

# q_0 = [2.,0.,1.5]
# q_1 = [2.,0.3,1.5]
# q_2 = [2.,-0.3,1.5]
# psi_0 = pi
# theta_0 = 0.
# psi_1 = pi
# theta_1 = 0.
# psi_2 = pi
# theta_2 = 0.
# u_0 = unit_vec(psi_0, theta_0)
# u_1 = unit_vec(psi_1, theta_1)
# u_2 = unit_vec(psi_2, theta_2)
# lm_0 = Landmark(q_0,u_0)
# lm_1 = Landmark(q_1,u_1)
# lm_2 = Landmark(q_2,u_2)
# lmk_list = Landmark_list([lm_0,lm_1,lm_2])

# pickle.dump(lmk_list,file_object)

# # Test
# file_object.close()
# file_object = open(filename)
# print pickle.load(file_object)