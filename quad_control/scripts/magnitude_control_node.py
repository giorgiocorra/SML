#!/usr/bin/env python
"""This module implements a ROS node that guarantees the collision avoidance.
It receives p_dot_max and omega_max from the planner and publishes p_dot_star and omega_star."""

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

get = rospy.get_param

import numpy as np
norm = np.linalg.norm
import utilities.coverage_giorgio as cov

from quad_control.msg import quad_speed_cmd_3d, camera_pose
from quad_control.srv import Filename

class MagnitudeControlNode():
	def __init__(self,namespace=""):

		self.namespace = namespace

		D_OPT = get("D_OPT",0.3)
		v_lim = get("v_lim",0.3)
		alpha = get("alpha",1)
		beta = get("beta",0.5)
		tau = get("tau",0.9)
		v_min = get("v_min",1e-2)
		freq = get("planner_frequency",1e1)
		t_s = 1. / freq
		control_type = get("magnitude_control_type","None")

		rospy.init_node('magnitude', anonymous=True)

		self.p_dot_max = np.zeros(3)
		self.omega_max = np.zeros(3)
		self.p_dot_star = np.zeros(3)
		self.omega_star = np.zeros(3)
		self.cam = cov.Camera()
		self.Q = cov.Landmark_list()

		rospy.Service("/"+self.namespace+'load_lmks_magnitude_control', Filename, self.load_lmks)

		max_speed_topic = "/" + self.namespace + "quad_max_speed_cmd"
		magnitude_speed_topic = "/" + self.namespace + "quad_speed_magnitude"

		collision_av_active = rospy.get_param("collision_avoidance_active", True)

		if (collision_av_active):
			star_speed_topic = "/" + self.namespace + "quad_speed_cmd_avoid"
		else:
			star_speed_topic = "/" + self.namespace + "quad_max_speed_cmd"						# terrible way of solving the problem!
		
		rospy.Subscriber('camera_pose', camera_pose, self.camera_pose_callback)
		rospy.Subscriber(max_speed_topic, quad_speed_cmd_3d, self.max_speed_callback)
		rospy.Subscriber(star_speed_topic, quad_speed_cmd_3d, self.star_speed_callback)

		self.magnitude_speed_publisher = rospy.Publisher(magnitude_speed_topic, quad_speed_cmd_3d, queue_size=10)

		rate = rospy.Rate(freq)
		while not rospy.is_shutdown():
			if (control_type == "None"):
				p_dot = self.p_dot_star
			elif (control_type == "Projection"):
				if not (norm(self.p_dot_star)==0):
					self.p_dot_star = self.p_dot_star.dot(1/ norm(self.p_dot_star))
				magnitude = self.p_dot_star.dot(self.p_dot_max)
				p_dot = self.p_dot_star.dot(magnitude)
			elif (control_type == "BLS"):
				if not (norm(self.p_dot_star)==0):
					self.p_dot_star = self.p_dot_star.dot(1/ norm(self.p_dot_star))
				p_dot = cov.backtracking(self.cam, self.Q, self.p_dot_max, self.p_dot_star, D_OPT, v_lim, alpha, beta, tau, t_s, v_min = 0)
			msg = quad_speed_cmd_3d()
			msg.vx = p_dot[0]
			msg.vy = p_dot[1]
			msg.vz = p_dot[2]
			msg.wx = self.omega_star[0]
			msg.wy = self.omega_star[1]
			msg.wz = self.omega_star[2]
			self.magnitude_speed_publisher.publish(msg)
			rate.sleep()

	def max_speed_callback(self, data):
		self.p_dot_max = np.array([data.vx, data.vy, data.vz])
		self.omega_max = np.array([data.wx, data.wy, data.wz])		

	def star_speed_callback(self, data):
		self.p_dot_star = np.array([data.vx, data.vy, data.vz])
		self.omega_star = np.array([data.wx, data.wy, data.wz])

	def camera_pose_callback(self,data):
		self.cam.new_pose(data)

	def load_lmks(self,data):
		self.Q = cov.read_lmks_from_file(data.filename)
		rospy.logwarn("Landmarks loaded in magnitude control: ")
		rospy.logwarn(str(self.Q))
		return {}


"""PROBLEMA: serve subscribe anche alla posizione del quad (e degli altri), ma deve essere sincronizzata con la velocita"""

try:
	MagnitudeControlNode()
except rospy.ROSInterruptException:
	pass