#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy
get = rospy.get_param


import utilities.coverage_giorgio as cov
from utilities.utility_functions import lat_long_from_unit_vec
#import geometry_msgs.msg as gms
#import tf.transformations as tf


from quad_control.msg import quad_speed_cmd_3d, camera_pose
from std_msgs.msg import Float64

import numpy as np


class SaveDataNode():

	def __init__(self):

		rospy.init_node('trade_node', anonymous=True)

		self.name_agents = get("/name_agents", '').rsplit(' ')
		self.n_ag = len(self.name_agents)

		self.cams = {}
		self.p_dot_max = {}
		self.omega_max = {}
		self.p_dot_dir = {}
		self.p_dot_star = {}
		self.omega_star = {}
		self.vision = {}
		self.q = {}
		self.u = {}

		self.is_active = False

		self.index_agents = {}
		i = 0
		for ns in self.name_agents:
			self.cams[ns] = cov.Camera()
			self.p_dot_max[ns] = [0.,0.,0.]
			self.omega_max[ns] = [0.,0.,0.]
			self.p_dot_dir[ns] = [0.,0.,0.]
			self.p_dot_star[ns] = [0.,0.,0.]
			self.omega_star[ns] = [0.,0.,0.]
			self.vision[ns] = 0.
			self.q[ns] = []
			self.u[ns] = []
			self.index_agents[ns] = i
			i += 1
			rospy.Subscriber("/" + ns + "/camera_pose", camera_pose, lambda data,ns = ns: self.camera_pose_callback(data,ns))
			rospy.Subscriber("/" + ns + "/quad_max_speed_cmd", quad_speed_cmd_3d, lambda data,ns = ns: self.max_speed_callback(data,ns))
			rospy.Subscriber("/" + ns + "/quad_speed_cmd_avoid", quad_speed_cmd_3d, lambda data,ns = ns: self.avoid_speed_callback(data,ns))
			rospy.Subscriber("/" + ns + "/quad_speed_magnitude", quad_speed_cmd_3d, lambda data,ns = ns: self.magnitude_speed_callback(data,ns))
			rospy.Subscriber("/" + ns + "/vision", Float64, lambda data,ns = ns: self.vision_callback(data,ns))

	def camera_pose_callback(self, data, ns):
		self.cams[ns].new_pose(data)

	def max_speed_callback(self, data, ns):
		self.p_dot_max[ns] = [data.px, data.py, data.pz]
		self.omega_max[ns] = [data.wx, data.wy, data.wz]

	def avoid_speed_callback(self, data, ns):
		self.p_dot_dir[ns] = [data.px, data.py, data.pz]

	def magnitude_speed_callback(self, data, ns):
		self.p_dot_star[ns] = [data.px, data.py, data.pz]
		self.omega_star[ns] = [data.wx, data.wy, data.wz]

	def vision_callback(self,data,ns):
		self.vision[ns] = data.data

	def run(self):
		





















if (get("/save_data",True)):
	ASaveDataNode = SaveDataNode()
	try:
		ASaveDataNode.run()
	except rospy.ROSInterruptException:
		pass