#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from matplotlib.backends.backend_agg import FigureCanvasAgg

from matplotlib.patches import Rectangle

from quad_control.msg import camera_pose
from quad_control.srv import LandmarksTrade


import rospy

from utilities.coverage_giorgio import Camera, Landmark_list

get = rospy.get_param

import numpy

from math import pi,cos,sin

#import utilities.coverage_utilities as cov

class ProjectorPlotNode():

	def __init__(self):

		self.frequency = get("/planner_frequency",1e1)
		self.D_OPT = get("/D_OPT",2)
		self.name_agents = get("/name_agents", '').rsplit(' ')

		self.x_min = get("/x_min",-1.5)
		self.y_min = get("/y_min",-2.2)
		self.x_max = get("/x_max",2)
		self.y_max = get("/y_max",1.7)
		self.d_worry = get("/delta_worry",0.5)

		self.r = get("/radius_circle_proj",0.5)

		rospy.init_node('projector_node', anonymous=True)

		self.colors = ['b', 'r', 'g']

		self.cam = {}
		self.lmks = {}
		self.index = {}

		i =0
		for ns in self.name_agents:
			self.cam[ns] = Camera()
			self.lmks[ns] = Landmark_list([])
			self.index[ns] = i
			i += 1
			# plot, = ax.plot([], [], lw=2, label = "position " + ns, color = "k")
			# self.pos_plot_list[ns] = plot
			# plot, = ax.plot([], [], lw=2, label = "orientation " + ns , color = "k")
			# # arrow(0, 0, 0.5, 0.5, head_width=0.05, head_length=0.1, fc='k', ec='k')
			# # plot, = ax.arrow([], [], [], [],  )
			# self.v_plot_list[ns] = plot
			if ns == "":
				rospy.Subscriber("/camera_pose", camera_pose, lambda data,ns = ns: self.camera_pose_callback(data,ns))
			else:
				rospy.Subscriber("/" + ns + "/camera_pose", camera_pose, lambda data,ns = ns: self.camera_pose_callback(data,ns))

		rospy.Service("/load_lmks_projector", LandmarksTrade, self.load_lmks)	



		#plt.legend(self.plot_list.values())
		#self.ani = animation.FuncAnimation(self.figure, self.redraw, int(self.frequency), blit=False)

	def camera_pose_callback(self, data, ns):
		self.cam[ns].new_pose(data)

	def load_lmks(self,data):
		# rospy.logwarn(data.name_agent + str(id(self.lmks[data.name_agent])))
		self.lmks[data.name_agent] = Landmark_list([])
		self.lmks[data.name_agent].from_lists(q = data.q, u = data.u)
		# rospy.logwarn("Landmarks loaded projector")
		# rospy.logwarn(str(data.name_agent))
		# rospy.logwarn(data.name_agent + str(id(self.lmks[data.name_agent])))
		return {}

	# def redraw(self,data=None):
	# 	angle = numpy.linspace(-pi, pi, num=100)
	# 	for ns in self.name_agents:
	# 		p = self.cam[ns].position()
	# 		x = [p[0] + self.r * cos(a) for a in angle]
	# 		y = [p[1] + self.r * sin(a) for a in angle]
	# 		self.pos_plot_list[ns].set_data(x,y)
	# 		v = self.cam[ns].orientation_as_vector()
	# 		self.v_plot_list[ns].set_data([p[0],p[0]+v[0]],[p[1],p[1]+v[1]])
	# 	return self.pos_plot_list, self.v_plot_list

	# def run(self):
	# 	plt.show()
	def save_to_proj(self):
		ax = plt.gca()
		plt.axis('off')
		r = 4./3./1.04
		ly = 6.50
		lx = ly*r
		cy = 0.75
		cx = 0.6
		ax.set_xlim([cx-lx/2.,cx+lx/2.])
		ax.set_ylim([cy-ly/2.,cy+ly/2.])
		transform = Affine2D().rotate_deg(105)
		ax.set_transform(transform)
		plt.savefig("/home/giorgiocorra/videoproj/videoproj.png",bbox_inches='tight',dpi=120)


	def save_plot(self):
		pos_plot_list = {}
		v_plot_list = {}
		lmk_plot_list = {}

		plt.clf()
		plt.cla()

		# # Cage
		# plt.axhspan(ymin = self.y_min, ymax = self.y_max, xmin = self.x_min, xmax = self.x_max, lw = 4, fc = 'none')
		# plt.axhspan(ymin = self.y_min + self.d_worry, ymax = self.y_max - self.d_worry, xmin = self.x_min + self.d_worry, xmax = self.x_max - self.d_worry, lw = 4, ls = 'dashed', fc = 'none')
		
		# Quads and lmks
		ax = plt.gca()
		figure = plt.gcf()

		rect = Rectangle(xy=(self.x_min,self.y_min), width = self.x_max - self.x_min, height = self.y_max - self.y_min, fill = False, angle = 0.0, lw = 4)
		ax.add_patch(rect)
		rect = Rectangle(xy=(self.x_min + self.d_worry,self.y_min + self.d_worry), width = (self.x_max - self.d_worry) - (self.x_min + self.d_worry), height = (self.y_max - self.d_worry) - (self.y_min + self.d_worry), fill = False, angle = 0.0, lw = 4, ls = 'dashed')
		ax.add_patch(rect)
		angle = numpy.linspace(-pi, pi, num=100)
		for ns in self.name_agents:
			col = self.colors[self.index[ns]]
			#rospy.logwarn("Color: " + str(self.index[ns]))
			p = self.cam[ns].position()
			x = [p[0] + self.r * cos(a) for a in angle]
			y = [p[1] + self.r * sin(a) for a in angle]
			plot, = ax.plot([], [], lw=4, color = col)
			pos_plot_list[ns] = plot
			pos_plot_list[ns].set_data(x,y)
			v = self.cam[ns].orientation_as_vector()
			v_plot_list[ns] = ax.arrow(p[0], p[1], v[0], v[1], head_width=0.1, head_length=0.2, fc = col, ec= col, lw = 4)
			vis_set = self.lmks[ns].visible_set(self.cam[ns],self.D_OPT)
			not_vis_set = self.lmks[ns].not_visible_set(self.cam[ns],self.D_OPT)
			#rospy.logwarn(str(not_vis_set))
			lmk_plot_list[ns] = []
			for lmk in vis_set.lst:
				q = lmk.q
				u = lmk.u
				lmk_plot_list[ns].append(ax.arrow(q[0], q[1], 0.3*u[0], 0.3 * u[1], lw = 2,head_width=0.2, head_length=0.3, fc = col, ec= col, fill=True))
			for lmk in not_vis_set.lst:
				q = lmk.q
				u = lmk.u
				lmk_plot_list[ns].append(ax.arrow(q[0], q[1], 0.3*u[0], 0.3 * u[1], lw=2, head_width=0.2, head_length=0.3, fc = 'w', ec= col,  fill=True))


		self.save_to_proj()


if __name__ == '__main__':
	plot_node = ProjectorPlotNode()
	rate = rospy.Rate(4)
	while not rospy.is_shutdown():
		plot_node.save_plot()
		rate.sleep()
	# plot_node.run()


