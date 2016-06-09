#!/usr/bin/env python
"""This module implements a ROS node that
acts as a planner for a vision task for a single quad."""

import sys

sys.path.append('~/sml_ws/src/quad_control/src/utilities')

import coverage_giorgio as cov
import example_vision as ex
import geometry_msgs.msg as gms
import std_msgs.msg as sms
#import quad_control.msg as qms
#import quad_control.srv as qsv
#import threading as thd
import numpy as np
import rospy as rp
#import random as rdm


__VEL_MIN = 0.1
__INITIAL_LANDMARK_LIST = ex.landmarks()
__INITIAL_POSITION = ex.position()                       # non sono sicuro che serva
__INITIAL_ORIENTATION = ex.orientation()              # non sono sicuro che serva, nel dubbio punta in basso
__MOVING = True

__landmarks = __INITIAL_LANDMARK_LIST
__camera = cov.Camera(__INITIAL_POSITION, __INITIAL_ORIENTATION)  
	
	
def __pose_callback(pose):
	global __camera
	__camera.new_pose(pose)
	
	
rp.init_node('coverage_planner')
__initial_time = rp.get_time()
__RATE = rp.Rate(1e1)
rp.Subscriber('pose', gms.Pose, __pose_callback)
__cmd_vel_pub = rp.Publisher('cmd_vel', gms.Twist, queue_size=10)
__cov_pub = rp.Publisher('vision', sms.Float64, queue_size=10)

def twist_from_velocities(lin,ang):
	lin_msg = gms.Vector3(lin[0],lin[1],lin[2])
	ang_msg = gms.Vector3(ang[0],ang[1],ang[2])
	return gms.Twist(lin_msg,ang_msg)

def __move():
	global __camera, __landmarks
	global __cmd_vel_pub, __cov_pub
	global __MOVING
	dot_p = cov.linear_velocity(__camera, __landmarks)
	omega = cov.angular_velocity(__camera, __landmarks)
	vision = __landmarks.vision(__camera)
	if np.linalg.norm(dot_p) < __VEL_MIN and np.linalg.norm(omega) < __VEL_MIN:
		"""End of optimization"""		
		__MOVING = False
	#pose = __camera.pose_msg
	#if np.linalg.norm(dot_p) > 10.3:
	#	dot_p *= 10.3/np.linalg.norm(dot_p)
	#omega = np.clip(omega, -10.0, 10.0)
	#
	cmd_vel = twist_from_velocities(dot_p,omega)
	__cmd_vel_pub.publish(cmd_vel)
	__cov_pub.publish(vision)
	


while (__MOVING) and (not rp.is_shutdown()):
	__move()
	__RATE.sleep()

while (not __MOVING) and (not rp.is_shutdown()):
	cmd_vel = twist_from_velocities([0., 0., 0.],[0., 0., 0.])
	__cmd_vel_pub.publish(cmd_vel)
	__RATE.sleep() 
	
