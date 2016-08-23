#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""


import matplotlib.pyplot as plt
import matplotlib.animation as animation

from std_msgs.msg import Float64
import rospy

get = rospy.get_param

import numpy

#import utilities.coverage_utilities as cov

class CoveragePlotNode():

	def __init__(self):

		self.frequency = get("/planner_frequency",1e1)

		self.name_agents = get("/name_agents", '').rsplit(' ')

		rospy.init_node('plot_node', anonymous=True)

		time_length = 10.
		samples = int(time_length * self.frequency)

		self.t = numpy.arange(0.,time_length,1./self.frequency)

		self.vision = {}

		ax = plt.gca()
		plt.xlim(0.,time_length)
		plt.ylim(0.,15.)
		self.figure = plt.gcf()
		self.plot_list = {}

		for ns in self.name_agents:
			self.vision[ns] = numpy.zeros(samples) 
			plot, = ax.plot([], [], lw=2, label = ns)
			self.plot_list[ns] = plot
			rospy.Subscriber("/" + ns + "/vision", Float64, lambda data,ns = ns: self.vision_callback(data,ns))

		plot, = ax.plot([], [], lw=2, label = 'Coverage')
		self.plot_list['coverage'] = plot
		plt.legend(self.plot_list.values())
		self.ani = animation.FuncAnimation(self.figure, self.redraw, int(self.frequency), blit=False)

	def vision_callback(self, data, ns):
		self.vision[ns] = numpy.concatenate([self.vision[ns], [data.data]])
		self.vision[ns] = self.vision[ns][1:]

	def redraw(self,data=None):
		coverage = numpy.zeros(len(self.t))
		for ns in self.name_agents:
			self.plot_list[ns].set_data(self.t,self.vision[ns])
			coverage += self.vision[ns]
		self.plot_list['coverage'].set_data(self.t,coverage)
		return self.plot_list

	def run(self):
		plt.show()


class VisionPlotNode():

	def __init__(self):

		self.frequency = get("/planner_frequency",1e1)

		rospy.init_node('plot_node', anonymous=True)

		time_length = 10.
		samples = int(time_length * self.frequency)

		self.vision = numpy.zeros(samples)

		self.t = numpy.arange(0.,time_length,1./self.frequency)

		rospy.Subscriber("/vision", Float64, self.vision_callback)

		ax = plt.gca()
		plt.xlim(0.,time_length)
		plt.ylim(0.,3.5)
		self.figure = plt.gcf()
		self.plot, = ax.plot([], [], lw=2)
		self.ani = animation.FuncAnimation(self.figure, self.redraw,int(self.frequency), blit=False)

	def vision_callback(self, data):
		self.vision = numpy.concatenate([self.vision, [data.data]])
		self.vision = self.vision[1:]

	def redraw(self,data=None):
		X = numpy.random.rand(2, 3)

		self.plot.set_data(self.t,self.vision)
		return self.plot,

	def run(self):
		plt.show()

if get("/name_agents", '')=="":
	plot_node = VisionPlotNode()
else:
	plot_node = CoveragePlotNode()

try:
	plot_node.run()
except rospy.ROSInterruptException:
	pass


