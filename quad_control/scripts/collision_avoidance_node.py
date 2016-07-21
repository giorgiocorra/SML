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
import utilities.collision_avoidance_utilities as coll_av

from quad_control.msg import quad_speed_cmd_3d, camera_pose

class CollisionAvoidanceNode():
	def __init__(self,namespace=""):

		self.namespace = namespace

		x_min = get("x_min",-3.)
		y_min = get("y_min",-3.)
		z_min = get("z_min",0.)
		x_max = get("x_max",3.)
		y_max = get("y_max",3.)
		z_max = get("z_max",4.)
		delta_walls_x = get("delta_walls_x", 0.5)
		delta_walls_y = get("delta_walls_y", 0.5)
		delta_walls_z = get("delta_walls_z", 0.3)
		DELTA_OTHERS = get("delta_others", 2.)
		
		P_MIN = np.array([x_min,y_min,z_min])
		P_MAX = np.array([x_max,y_max,z_max])
		DELTA_WALLS = np.array([delta_walls_x,delta_walls_y,delta_walls_z])

		rospy.init_node('collision_avoidance', anonymous=True)

		self.p_dot_max = np.zeros(3)
		self.omega_max = np.zeros(3)
		self.p_dot_star = np.zeros(3)
		self.omega_star = np.zeros(3)
		self.p = [0,0,2]				# so that it does not stop the quad movement
		self.p_others = []
		self.time = 0.
		self.p_dictionary = {}
		self.p_others_dictionary = {}

		max_speed_topic = "/" + self.namespace + "quad_max_speed_cmd"
		star_speed_topic = "/" + self.namespace + "quad_speed_cmd_avoid"

		rospy.Subscriber('camera_pose', camera_pose, self.camera_pose_callback)
		rospy.Subscriber(max_speed_topic, quad_speed_cmd_3d, self.max_speed_callback)

		self.star_speed_publisher = rospy.Publisher(star_speed_topic, quad_speed_cmd_3d, queue_size=10)

		rate = rospy.Rate(1e1)
		while not rospy.is_shutdown():
			W = coll_av.wall_directions(self.p, self.p_others, P_MIN, P_MAX, DELTA_WALLS, DELTA_OTHERS)
			self.p_dot_star = coll_av.collision_avoidance_vel(self.p_dot_max, W)
			self.omega_star = self.omega_max
			msg = quad_speed_cmd_3d()
			msg.vx = self.p_dot_star[0]
			msg.vy = self.p_dot_star[1]
			msg.vz = self.p_dot_star[2]
			msg.wx = self.omega_star[0]
			msg.wy = self.omega_star[1]
			msg.wz = self.omega_star[2]
			self.star_speed_publisher.publish(msg)
			rate.sleep()

	def max_speed_callback(self, data):
		self.p_dot_max = np.array([data.vx, data.vy, data.vz])
		self.omega_max = np.array([data.wx, data.wy, data.wz])
		self.time = data.time
		self.p = self.p_dictionary[self.time]
		self.p_others = self.p_others_dictionary[data.time]
		# Remove useless entries from dictionaries
		self.p_dictionary = {t:p for (t,p) in self.p_dictionary.items() if t < self.time}
		self.p_others_dictionary = {t:p for (t,p) in self.p_others_dictionary.items() if t < self.time}

	def camera_pose_callback(self,data):
		time = data.time
		pos = [data.px, data.py, data.pz]
		self.p_dictionary[time] = pos 
		# Fake update of position of others
		self.p_others_dictionary[time] = []



"""PROBLEMA: serve subscribe anche alla posizione del quad (e degli altri), ma deve essere sincronizzata con la velocita"""

try:
	CollisionAvoidanceNode()
except rospy.ROSInterruptException:
	pass