#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

import utilities.coverage_giorgio as cov
#from utilities.utility_functions import lat_long_from_unit_vec
import geometry_msgs.msg as gms
import tf.transformations as tf

try:
	from quad_control.srv import Filename, LandmarksTrade
	from std_srvs.srv import Empty
	from quad_control.msg import quad_speed_cmd_3d, camera_pose
	from std_msgs.msg import Float64
except ImportError:
	pass
import json
import numpy as np

# from converter_between_standards.rotorS_converter import RotorSConverter
# # node will publish motor speeds
# from mav_msgs.msg import Actuators
# #node will subscribe to odometry measurements
# from nav_msgs.msg import Odometry


class MonotonePlannerNode():
	def __init__(self,namespace,firefly=False):

		rospy.init_node('planner', anonymous=True)

		self.namespace = rospy.get_namespace()[1:]

		# rospy.logerr("Namespace name: " + self.namespace)

		speed_command_topic = "/"+self.namespace+"quad_max_speed_cmd"
		vision_topic = "/"+self.namespace+"vision"

		self.speed_controller = rospy.Publisher(speed_command_topic, quad_speed_cmd_3d, queue_size=10)
		self.vision_publisher = rospy.Publisher(vision_topic, Float64, queue_size=10)

		self.D_OPT = rospy.get_param("/D_OPT")
		freq = rospy.get_param("/planner_frequency",1e1)
		self.__landmarks = cov.Landmark_list([])
		self.__camera = cov.Camera()
		self.__time = 0.

		rospy.Subscriber("/"+self.namespace+'camera_pose', camera_pose, self.update_pose)

		rospy.Service("/"+self.namespace+'load_lmks_planner', LandmarksTrade, self.load_lmks)
		rospy.Service("/"+self.namespace+'start_planner', Empty, self.start_planner_execution)
		rospy.Service("/"+self.namespace+'stop_planner', Empty, self.stop)

		self.state = "stop"
		self.pos = None

		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			if self.state == "start":
				break
			rate.sleep()

		# Start of the plan
		rate = rospy.Rate(freq)		

		while not rospy.is_shutdown():
			if self.state=="stop":
				control_v = np.array([0,0,0])
				control_omega = np.array([0,0,0])
			elif self.state=="start":
				control_v = cov.linear_velocity(self.__camera, self.__landmarks, self.D_OPT)
				control_omega = cov.angular_velocity(self.__camera, self.__landmarks, self.D_OPT)
			# 	rospy.logwarn("Planned linear velocity: " + str(control_v))
			# 	rospy.logwarn("Planned angular velocity: " + str(control_omega))
			# rospy.logerr("Position: " + str(self.__camera.position()))
			# rospy.logerr("Orientation vector: " + str(self.__camera.orientation_as_lat_long()))
			# #rospy.logerr("Landmarks: " + str(self.__landmarks))
			vision = self.__landmarks.vision(self.__camera,self.D_OPT)
			vel_command = quad_speed_cmd_3d(vx=control_v[0],vy=control_v[1],vz=control_v[2],wx=control_omega[0],wy=control_omega[1],wz=control_omega[2],time=self.__time)
			#rospy.logerr(command)
			self.vision_publisher.publish(vision)
			self.speed_controller.publish(vel_command)
			# go to sleep
			rate.sleep()

	# def get_state_from_rotorS_simulator(self,odometry_rotor_s):
	# 	# RotorS has attitude inner loop that needs to known attitude of quad
	# 	self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
	# 	# get state from rotorS simulator
	# 	state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)
	# 	# update __camera
	# 	pos = gms.Point(state_quad[0], state_quad[1], state_quad[2])
	# 	quat_np = tf.transformations.quaternion_from_euler(state_quad[6], state_quad[7], state_quad[8])   #returns a numpy array [x,y,z,w]
	# 	quat = gms.Quaternion(quat_np[0], quat_np[1], quat_np[2], quat_np[3])
	# 	pose = gms.Pose(pos,quat)
	# 	self.__camera.new_pose(pose)

	def update_pose(self,cam_pose):
		self.__camera.new_pose(cam_pose)
		self.__time = cam_pose.time


	# def get_measure(self,data):
	# 	pos = gms.Point(data.x, data.y, data.z)
	# 	self.pos = pos
	# 	quat_np = tf.quaternion_from_euler(data.roll, data.pitch, data.yaw)   #returns a numpy array [x,y,z,w]
	# 	quat = gms.Quaternion(quat_np[0], quat_np[1], quat_np[2], quat_np[3])
	# 	pose = gms.Pose(pos,quat)
	# 	self.__camera.new_pose(pose)

	def start_planner_execution(self,data=None):
		self.state = "start"
		rospy.logwarn("Planner started")
		return {}

	def stop(self,data=None):
		self.state = "stop"
		rospy.logwarn("Stop planner")
		return {}

	def load_lmks(self,data):
		self.__landmarks = cov.Landmark_list([])
		self.__landmarks.from_lists(q = data.q, u = data.u)
		# rospy.logwarn("Landmarks" + str(self.namespace) +" loaded planner: ")
		# rospy.logwarn(str(self.__landmarks))
		return {}

	def reset(self):
		self.state = "stop"

if __name__ == '__main__':
	try:
		id_arg = sys.argv.index("--namespace")
		namespace = sys.argv[id_arg+1]
	except ValueError:
		namespace = ""

	firefly = "firefly" in sys.argv

	try:
		MonotonePlannerNode(namespace,firefly=firefly)
	except rospy.ROSInterruptException:
		pass
