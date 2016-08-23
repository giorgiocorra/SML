#!/usr/bin/env python

import numpy as np
import math
import rospy
from rospkg import RosPack
import utilities.coverage_giorgio as cov
import utilities.utility_functions as utf
import glob
import os
import matplotlib.pyplot as plt
from operator import add


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
			self.state == 'stop'
		elif self.a==2:
			self.state == 'position'
		elif self.a==3:
			self.state == 'speed'
		elif self.a==4:
			self.state == 'test_speed_control'

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
		self.a = []

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
			self.a.append(sample.a)

	def adjust_length(self, l):
		while (len(self.time) > l):
			del self.time[-1]
			del self.p[-1]
			del self.v[-1]
			del self.p_cmd[-1]
			del self.p_dot_cmd[-1]
			del self.ee[-1]
			del self.force_3d[-1]
			del self.lat[-1]
			del self.long[-1]
			del self.p_dot_max[-1]
			del self.omega_max[-1]
			del self.p_dot_avoid[-1]
			del self.p_dot_star[-1]
			del self.omega_star[-1]
			del self.p_dot[-1]
			del self.lat_rate[-1]
			del self.long_rate[-1]
			del self.n_lmks[-1]
			del self.lmks[-1]
			del self.state[-1]
			del self.vision[-1]
			del self.a[-1]




rp = RosPack()
package_path = rp.get_path('quad_control')
package_save_path = package_path+'/experimental_data/data/'

files = filter(os.path.isfile, glob.glob(package_save_path + "*.txt"))
files.sort(key=lambda x: os.path.getmtime(x),reverse = True)
newest_files = files[0:3]

name_agents = {'Iris1', 'Iris2', 'Iris3'}
namefile = {}

l = len(package_save_path)
for name in newest_files:
	#print(name)
	if (name[l:l+5] == 'Iris1'):
		namefile['Iris1'] = name
		print('File Iris1: ' + name)
	elif (name[l:l+5] == 'Iris2'):
		namefile['Iris2'] = name
		print('File Iris2: ' + name)
	elif (name[l:l+5] == 'Iris3'):
		namefile['Iris3'] = name
		print('File Iris3: ' + name)
	else:
		print('Error: file ' + name[l:l+5] + ' not recognised')

sample_list = {}

for ns in name_agents:
	#print(ns)
	my_file  = open(namefile[ns], 'r')
	#print('Reading from file: ' + namefile[ns])
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
	sample_list[ns] = SampleList(samples)

l = min([len(sample_list[ns].time) for ns in name_agents])
coverage = [0]*l

for ns in name_agents:
	sample_list[ns].adjust_length(l)
	coverage = map(add,coverage,sample_list[ns].vision)

print len(coverage)

########################## PLOTS ##################################

# Vision
f, axarr = plt.subplots(4)
axarr[0].plot(sample_list['Iris1'].time, coverage)
axarr[0].plot(sample_list['Iris1'].time, sample_list['Iris1'].vision)
axarr[0].plot(sample_list['Iris2'].time, sample_list['Iris2'].vision)
axarr[0].plot(sample_list['Iris3'].time, sample_list['Iris3'].vision)
axarr[1].plot(sample_list['Iris1'].time, sample_list['Iris1'].vision)
axarr[1].plot(sample_list['Iris1'].time, sample_list['Iris1'].a)
axarr[2].plot(sample_list['Iris2'].time, sample_list['Iris2'].vision)
axarr[2].plot(sample_list['Iris2'].time, sample_list['Iris2'].a)
axarr[3].plot(sample_list['Iris3'].time, sample_list['Iris3'].vision)
axarr[3].plot(sample_list['Iris3'].time, sample_list['Iris3'].a)
plt.show()

# Position
f, axarr = plt.subplots(3)
x,y,z = axarr[0].plot(sample_list['Iris1'].time, sample_list['Iris1'].p)
x_d,y_d,z_d = axarr[0].plot(sample_list['Iris1'].time, sample_list['Iris1'].p_cmd,'--')
axarr[0].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 1, labelspacing = 0.)
x,y,z = axarr[1].plot(sample_list['Iris2'].time, sample_list['Iris2'].p)
x_d,y_d,z_d = axarr[1].plot(sample_list['Iris2'].time, sample_list['Iris2'].p_cmd,'--')
axarr[1].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 2, labelspacing = 0.)
x,y,z = axarr[2].plot(sample_list['Iris3'].time, sample_list['Iris3'].p)
x_d,y_d,z_d = axarr[2].plot(sample_list['Iris3'].time, sample_list['Iris3'].p_cmd,'--')
axarr[2].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 3, labelspacing = 0.)
plt.show()

# Velocity
f, axarr = plt.subplots(3)
x,y,z = axarr[0].plot(sample_list['Iris1'].time, sample_list['Iris1'].p_dot)
x_d,y_d,z_d = axarr[0].plot(sample_list['Iris1'].time, sample_list['Iris1'].p_dot_cmd,'--')
axarr[0].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 1, labelspacing = 0.)
x,y,z = axarr[1].plot(sample_list['Iris2'].time, sample_list['Iris2'].p_dot)
x_d,y_d,z_d = axarr[1].plot(sample_list['Iris2'].time, sample_list['Iris2'].p_dot_cmd,'--')
axarr[1].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 2, labelspacing = 0.)
x,y,z = axarr[2].plot(sample_list['Iris3'].time, sample_list['Iris3'].p_dot)
x_d,y_d,z_d = axarr[2].plot(sample_list['Iris3'].time, sample_list['Iris3'].p_dot_cmd,'--')
axarr[2].legend([x,y,z,x_d,y_d,z_d], ['x','y','z','x_d','y_d','z_d'], loc = 0, ncol = 3, labelspacing = 0.)
plt.show()











