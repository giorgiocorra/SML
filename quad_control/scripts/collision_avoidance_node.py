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

		self.namespace = rospy.get_namespace()[1:]

		smooth = get("/" + self.namespace + "smooth",True)

		freq = get("/" + self.namespace + "planner_frequency",1e1)

		x_min = get("/" + self.namespace + "x_min",-3.)
		y_min = get("/" + self.namespace + "y_min",-3.)
		z_min = get("/" + self.namespace + "z_min",0.)
		x_max = get("/" + self.namespace + "x_max",3.)
		y_max = get("/" + self.namespace + "y_max",3.)
		z_max = get("/" + self.namespace + "z_max",4.)
		delta_walls_x = get("/" + self.namespace + "delta_walls_x", 0.5)
		delta_walls_y = get("/" + self.namespace + "delta_walls_y", 0.5)
		delta_walls_z = get("/" + self.namespace + "delta_walls_z", 0.3)
		DELTA_OTHERS = get("/" + self.namespace + "delta_others", 2.)
		DELTA_WORRY = get("/" + self.namespace + "delta_worry", 0.5)
		name_others = get("/" + self.namespace + "name_others", '')

		if (name_others == ''):
			self.p_others = []
		else:
			self.name_others = name_others.rsplit(' ')
			self.p_others = np.zeros([len(self.name_others),3])
			self.index_others = {}
			i = 0
			for ns in self.name_others:
				self.index_others[ns] = i
				i += 1
				rospy.Subscriber("/" + ns + "/camera_pose", camera_pose, lambda data,ns = ns: self.camera_pose_others_callback(data,ns) )
		
		P_MIN = np.array([x_min,y_min,z_min])
		P_MAX = np.array([x_max,y_max,z_max])
		DELTA_WALLS = np.array([delta_walls_x,delta_walls_y,delta_walls_z])

		rospy.init_node('collision_avoidance', anonymous=True)

		self.p_dot_max = np.zeros(3)
		self.omega_max = np.zeros(3)
		self.p_dot_star = np.zeros(3)
		self.omega_star = np.zeros(3)
		self.ready = False
		self.p = []	


		max_speed_topic = "/" + self.namespace + "quad_max_speed_cmd"
		star_speed_topic = "/" + self.namespace + "quad_speed_cmd_avoid"

		rospy.Subscriber('camera_pose', camera_pose, self.camera_pose_callback)
		rospy.Subscriber(max_speed_topic, quad_speed_cmd_3d, self.max_speed_callback)

		self.star_speed_publisher = rospy.Publisher(star_speed_topic, quad_speed_cmd_3d, queue_size=10)

		rate = rospy.Rate(freq)
		while not self.ready:			# wait for all the callbacks
			if all(any(self.p_others[i]) for i in range(len(self.p_others))) and any(self.p):
				self.ready = True
			else:
				rate.sleep()

		while not rospy.is_shutdown():
			if (smooth):
				W, Lambda = coll_av.wall_directions_smooth(self.p, self.p_others, P_MIN, P_MAX, DELTA_WALLS, DELTA_OTHERS, DELTA_WORRY)
				self.p_dot_star = coll_av.collision_avoidance_vel_smooth(self.p_dot_max, W, Lambda)
			else:
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

	def camera_pose_callback(self,data):
		self.p = np.array([data.px, data.py, data.pz])

	def camera_pose_others_callback(self,data,ns):
		self.p_others[self.index_others[ns]] = [data.px, data.py, data.pz] 



"""PROBLEMA: serve subscribe anche alla posizione del quad (e degli altri), ma deve essere sincronizzata con la velocita"""

try:
	CollisionAvoidanceNode()
except rospy.ROSInterruptException:
	pass