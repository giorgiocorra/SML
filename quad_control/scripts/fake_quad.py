#!/usr/bin/env python
"""This module implements a ROS node that acts as a fake 
quad: receives the velocity and updates the pose."""

import example_vision as ex
import rospy as rp
import numpy as np
norm = np.linalg.norm
import geometry_msgs.msg as gms

from quaternion_utilities import q_mult, qv_mult

__INITIAL_POSITION = ex.position()                       # non sono sicuro che serva
#__INITIAL_ORIENTATION = ex.orientation()              # non sono sicuro che serva, nel dubbio punta in basso
__p = __INITIAL_POSITION
__quat = [0., 0., 0., 1.]

def __vel_callback(twist):
	global __p
	global __quat
	global __t_last
	t_now = rp.get_time()
	t_s = __t_last - t_now
	t_s = 0.1
	__t_last = t_now
	vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
	omega = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
	__p += vel.dot(t_s)
	omega_q = np.concatenate((omega,[0.]),axis = 0)
	__quat += q_mult(omega_q,__quat).dot(0.5 * t_s)
	__quat /= norm(__quat)

def pose_from_point_quat(p,q):
	p_msg = gms.Point(p[0],p[1],p[2])
	q_msg = gms.Quaternion(q[0],q[1],q[2],q[3])
	return gms.Pose(p_msg,q_msg)

rp.init_node('fake_quad')
__t_last = rp.get_time()
__RATE = rp.Rate(1e1)
__pose_pub = rp.Publisher('pose', gms.Pose, queue_size=10)
rp.Subscriber('cmd_vel', gms.Twist, __vel_callback)

def __work():
	global __p
	global __quat
	pose = pose_from_point_quat(__p,__quat)
	__pose_pub.publish(pose)


while not rp.is_shutdown():
	__work()
	__RATE.sleep()
