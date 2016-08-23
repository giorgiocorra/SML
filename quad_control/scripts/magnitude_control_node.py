#!/usr/bin/env python
"""This module implements a ROS node that computes the magnitude of the velocity."""

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

get = rospy.get_param

import numpy as np
norm = np.linalg.norm
import utilities.coverage_giorgio as cov

import json

from quad_control.msg import quad_speed_cmd_3d, camera_pose
from quad_control.srv import LandmarksTrade

class MagnitudeControlNode():
	def __init__(self,namespace=""):

		self.namespace = rospy.get_namespace()[1:]

		D_OPT = get("/D_OPT",3.)
		v_lim = get("/v_lim",0.3)
		omega_lim = get("/omega_lim",0.79)
		alpha = get("/alpha",1)
		beta = get("/beta",0.5)
		tau = get("/tau",0.9)
		v_min = get("/v_min",1e-2)
		omega_min = get("/omega_min",0.02)
		freq = get("/planner_frequency",1e1)
		t_s = 1. / freq
		control_type = get("/magnitude_control_type","None")

		rospy.init_node('magnitude', anonymous=True)

		self.p_dot_max = np.zeros(3)
		self.omega_max = np.zeros(3)
		self.p_dot_star = np.zeros(3)
		self.omega_star = np.zeros(3)
		self.cam = cov.Camera()
		self.Q = cov.Landmark_list([])

		rospy.Service("/"+self.namespace+'load_lmks_magnitude_control', LandmarksTrade, self.load_lmks)

		max_speed_topic = "/" + self.namespace + "quad_max_speed_cmd"
		magnitude_speed_topic = "/" + self.namespace + "quad_speed_magnitude"

		collision_av_active = rospy.get_param("/collision_avoidance_active", True)

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
				omega = self.omega_max
			elif (control_type == "Projection"):
				if not (norm(self.p_dot_star)==0):
					self.p_dot_star = self.p_dot_star.dot(1/ norm(self.p_dot_star))
				magnitude = self.p_dot_star.dot(self.p_dot_max)
				p_dot = self.p_dot_star.dot(magnitude)
				omega = self.omega_max
			elif (control_type == "BLS"):
				if not (norm(self.p_dot_star)==0):
					self.p_dot_star = self.p_dot_star.dot(1/ norm(self.p_dot_star))
				p_dot = cov.bls_velocity(self.cam, self.Q, self.p_dot_max, self.p_dot_star, D_OPT, v_lim, alpha, beta, tau, t_s, v_min = 0.01)
				omega = cov.bls_omega(self.cam,self.Q, self.omega_max, D_OPT, omega_lim, alpha, beta, tau, t_s, omega_min = 0.02)
			msg = quad_speed_cmd_3d()
			msg.vx = p_dot[0]
			msg.vy = p_dot[1]
			msg.vz = p_dot[2]
			msg.wx = omega[0]
			msg.wy = omega[1]
			msg.wz = omega[2]
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
		self.Q = cov.Landmark_list([])
		self.Q.from_lists(q = data.q, u = data.u)
		#rospy.logwarn("Landmarks loaded in magnitude control: ")
		#rospy.logwarn(str(self.Q))
		return {}

try:
	MagnitudeControlNode()
except rospy.ROSInterruptException:
	pass