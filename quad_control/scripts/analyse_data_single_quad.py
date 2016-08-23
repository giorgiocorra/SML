#!/usr/bin/env python

import numpy as np
norm = np.linalg.norm
import math
import rospy
from rospkg import RosPack
import utilities.coverage_giorgio as cov
import utilities.utility_functions as utf
import glob
import os
import matplotlib.pyplot as plt


class Sample():

	def __init__(self):
		self.time = 0.
		self.p = np.zeros(3)
		self.v = np.zeros(3)
		self.p_cmd = np.zeros(3)
		self.p_dot_cmd = np.zeros(3)
		self.ee = np.zeros(3)
		self.force_3d = np.zeros(3)
		self.lat = 0.
		self.long = 0.
		self.p_dot_max = np.zeros(3)
		self.omega_max = np.zeros(3)
		self.p_dot_avoid = np.zeros(3)
		self.p_dot_star = np.zeros(3)
		self.omega_star = np.zeros(3)
		self.p_dot = np.zeros(3)
		self.lat_rate = 0.
		self.long_rate = 0.
		self.n_lmks = 0
		self.lmks = cov.Landmark_list([])
		self.a = 0
		self.state = ''
		self.vision = 0.

	def read_from_line(self,line):
		data = line.rsplit(' ')
		default_data = data[0:19]
		complementary_data = data[19:]

		self.time = float(default_data[0])
		self.p = [float(default_data[k]) for k in range(1,4)]
		self.p_dot = [float(default_data[k]) for k in range(4,7)]
		self.p_cmd = [float(default_data[k]) for k in range(7,10)]
		self.p_dot_cmd = [float(default_data[k]) for k in range(10,13)]

		self.ee = [float(default_data[k]) for k in range(13,16)]
		self.force_3d = [float(default_data[k]) for k in range(16,19)]
	
		self.a = int(float(complementary_data[0]))

		if self.a == 1:
			self.state = 'stop'
		elif self.a==2:
			self.state = 'position'
		elif self.a==3:
			self.state = 'speed'
		elif self.a==4:
			self.state = 'test_speed_control'

		self.long = float(complementary_data[1])
		self.lat = float(complementary_data[2])
		self.p_dot_max = [float(complementary_data[k]) for k in range(3,6)]
		self.omega_max = [float(complementary_data[k]) for k in range(6,9)]
		self.p_dot_avoid = [float(complementary_data[k]) for k in range(9,12)]
		self.p_dot_star = [float(complementary_data[k]) for k in range(12,15)]
		self.omega_star = [float(complementary_data[k]) for k in range(15,18)]
		
		self.long_rate = self.omega_star[2]
		self.lat_rate = math.sin(self.long)*self.omega_star[0] - math.cos(self.long)*self.omega_star[1]
		self.v = utf.unit_vec_from_lat_long(self.long, self.lat)
		
		self.n_lmks = int(float(complementary_data[18]))
		self.vision = float(complementary_data[19])

		if not(self.n_lmks == 0):
			if not(len(complementary_data) == 20 + 5*self.n_lmks):
				print('Error! Sample at time: ' + str(self.time))
				print len(complementary_data)
				print(20+5*self.n_lmks)
			q = [float(complementary_data[k]) for k in range(20,20 + 3*self.n_lmks)]
			u = [float(complementary_data[k]) for k in range(20+ 3*self.n_lmks,20 + 5*self.n_lmks)]
			self.lmks.from_lists(q,u)

class SampleList():

	def __init__(self, lst = None):
		self.time = []
		self.p = []
		self.v = []
		self.p_cmd = []
		self.p_dot_cmd = []
		self.ee = []
		self.force_3d = []
		self.lat = []
		self.long = []
		self.p_dot_max = []
		self.omega_max = []
		self.p_dot_avoid = []
		self.p_dot_star = []
		self.omega_star = []
		self.p_dot = []
		self.lat_rate = []
		self.long_rate = []
		self.n_lmks = []
		self.lmks = []
		self.state = []
		self.vision = []

		self.time_velocity_control = []
		self.p_dot_velocity_control = []
		self.p_dot_cmd_velocity_control = []

		for time in sorted(lst):
			sample = lst[time]
			self.time.append(sample.time)
			self.p.append(sample.p)
			self.v.append(sample.v)
			self.p_cmd.append(sample.p_cmd)
			self.p_dot_cmd.append(sample.p_dot_cmd)
			self.ee.append(sample.ee)
			self.force_3d.append(sample.force_3d)
			self.lat.append(sample.lat)
			self.long.append(sample.long)
			self.p_dot_max.append(sample.p_dot_max)
			self.omega_max.append(sample.omega_max)
			self.p_dot_avoid.append(sample.p_dot_avoid)
			self.p_dot_star.append(sample.p_dot_star)
			self.omega_star.append(sample.omega_star)
			self.p_dot.append(sample.p_dot)
			self.lat_rate.append(sample.lat_rate)
			self.long_rate.append(sample.long_rate)
			self.n_lmks.append(sample.n_lmks)
			self.lmks.append(sample.lmks)
			self.state.append(sample.state)
			self.vision.append(sample.vision)
			if sample.state == 'stop':
				self.time_velocity_control.append(sample.time)
				self.p_dot_velocity_control.append(sample.p_dot)
				self.p_dot_cmd_velocity_control.append(sample.p_dot_cmd)



rp = RosPack()
package_path = rp.get_path('quad_control')
package_save_path = package_path+'/experimental_data/data/'

newest_file = max(glob.iglob(package_save_path+'*.txt'), key = os.path.getmtime)

my_file  = open(newest_file, 'r')

print('Reading from file: ' + newest_file)

samples = {}
description = ''

first_line = my_file.readline()

if (first_line[0] == '#'):
	start = False
	for line in my_file:
		if not(start):
			if (line[0] == '#'):
				start = True
				continue
			else:	
				description += line + '\n' 
		else:
			s = Sample()
			s.read_from_line(line)
			samples[s.time] = s
else:
	print('File should start with ##########')

my_file.close()

sample_list = SampleList(samples)

########################## PLOTS ##################################

plt.figure()
plt.plot(sample_list.time, sample_list.vision) 
plt.title('Vision')
plt.show()

plt.figure()
x,y,z  = plt.plot(sample_list.time, sample_list.p[:]) 
x_d,y_d,z_d = plt.plot(sample_list.time, sample_list.p_cmd[:],'--')
plt.title('Position')
plt.legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0)
plt.show()

plt.figure()
lat  = plt.plot(sample_list.time, sample_list.lat) 
longit = plt.plot(sample_list.time, sample_list.long)
plt.title('Latitude longitude')
plt.legend([lat,longit], ['Latitude','Longitude'], loc = 0)
plt.show()

# Velocity
plt.figure()
x, y, z  = plt.plot(sample_list.time, sample_list.p_dot[:]) 
x_d, y_d, z_d = plt.plot(sample_list.time, sample_list.p_dot_cmd[:],'--')
plt.title('Velocity')
plt.legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0)
plt.show()

# Velocity max
plt.figure(3)
x, y, z = plt.plot(sample_list.time, sample_list.p_dot_max[:]) 
plt.title('Velocity max')
plt.legend([x,y,z], ['x','y','z'], loc = 0)
plt.show()

# Omega
plt.figure()
x, y, z  = plt.plot(sample_list.time, sample_list.omega_star[:]) 
x_d, y_d, z_d = plt.plot(sample_list.time, sample_list.omega_max[:],'--')
plt.title('Omega')
plt.legend([x,y,z,x_d,y_d,z_d], ['w_star_x','w_star_y','w_star_z','w_max_x','w_max_y','w_max_z'], loc = 0)
plt.show()

# Omega max
plt.figure()
x_d, y_d, z_d = plt.plot(sample_list.time, sample_list.omega_max[:],'--')
plt.title('Omega')
plt.legend([x_d,y_d,z_d], ['w_max_x','w_max_y','w_max_z'], loc = 0)
plt.show()

# # # Velocity speed control
# # plt.figure()
# # x, y, z  = plt.plot(sample_list.time_velocity_control, sample_list.p_dot_velocity_control[:]) 
# # x_d, y_d, z_d = plt.plot(sample_list.time_velocity_control, sample_list.p_dot_cmd_velocity_control[:],'--')
# # plt.title('Velocity only during velocity control')
# # plt.legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0)
# # plt.show()

# # Norm velocity
# norm_vel = np.zeros(len(sample_list.time))
# norm_omega = np.zeros(len(sample_list.time))
# for i in range(len(sample_list.time)):
# 	norm_vel[i] = norm(sample_list.p_dot_cmd[i])
# 	norm_omega[i] = norm(sample_list.omega_star[i])

# # plt.figure()
# # plt.plot(sample_list.time,norm_omega)
# # plt.show()

# l = 50
# v_min = 0.01
# omega_min = 0.1 

# vel_buffer = [v_min]*l
# omega_buffer = [omega_min]*l
# vel_threshold = [v_min] * l
# omega_threshold = [omega_min] * l


# change_state = np.zeros(len(sample_list.time))
# for i in range(len(sample_list.time)):
# 	vel_buffer.append(norm_vel[i])
# 	omega_buffer.append(norm_omega[i])
# 	del vel_buffer[0]
# 	del omega_buffer[0]
# 	if (sample_list.state[i]=='speed') and any(vel_buffer[k] < v_min for k in range(l)) and any(omega_buffer[k] < omega_min for k in range(l)):
# 		change_state[i] = 1

# plt.figure()
# plt.plot(sample_list.time,change_state)
# plt.show()