#!/usr/bin/env python

import sys

import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))
import rospy

try:
	from quad_control.srv import PlannerStart
	from std_srvs.srv import Empty
	from quad_control.msg import quad_speed_controller_cmd
	from quad_control.msg import quad_state
except ImportError:
	pass
import pickle
import numpy as np

sys.path.append('/home/paul/workspace/master thesis/pwa/core')
import convert_plans

from converter_between_standards.rotorS_converter import RotorSConverter
# node will publish motor speeds
from mav_msgs.msg import Actuators
#node will subscribe to odometry measurements
from nav_msgs.msg import Odometry


class MonotonePlannerNode():
	def __init__(self,namespace,firefly=False):
		self.namespace = namespace

		self.execute_plan = False
		self.planner = None
		self.namespace = namespace

		rospy.init_node('planner', anonymous=True)

		speed_command_topic = "/"+self.namespace+"quad_speed_controller_cmd"

		self.speed_controller = rospy.Publisher(speed_command_topic, quad_speed_controller_cmd, queue_size=10)

		if firefly==False:
			rospy.Subscriber("/"+self.namespace+"quad_state", quad_state,self.get_measure, queue_size=10)
		else:
			# subscriber to odometry from RotorS
			self.sub_odometry = rospy.Subscriber("/firefly/ground_truth/odometry",Odometry,self.get_state_from_rotorS_simulator,queue_size=10)
		# converting our controlller standard into rotors standard
		self.RotorSObject = RotorSConverter()

		rospy.Service("/"+self.namespace+'load_plan', PlannerStart, self.load_plan)
		rospy.Service("/"+self.namespace+'start_plan', Empty, self.start_plan_execution)
		rospy.Service("/"+self.namespace+'stop_plan', Empty, self.stop)


		self.planner = None
		self.state = "stop"
		self.position = None

		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			if self.state == "start":
				break
			rate.sleep()

		# Start of the plan
		rate = rospy.Rate(1.0/self.planner.period)
		

		control = np.array([0,0])
		while not rospy.is_shutdown():
			if self.state=="stop":
				control = np.array([0,0])
			elif self.state=="start":

				try:
					if self.planner.fts_state == None:
						control = self.planner.init_state(self.position)
					else:
						control = self.planner.di_sim(self.position)
				except Exception,e:
					rospy.logwarn(e)
					self.reset()

				if type(control)==str:
					rospy.logerr(control)
					self.reset()
			if type(control)==np.ndarray:
				self.speed_controller.publish(quad_speed_controller_cmd(control[0],control[1]))
				
			# go to sleep
			rate.sleep()

	def get_state_from_rotorS_simulator(self,odometry_rotor_s):
		self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
		state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)

		self.position = state_quad[0:2]


	def get_measure(self,data):
		self.position = np.array([data.x,data.y])

	def start_plan_execution(self,data):
		self.state = "start"
		rospy.logwarn("Start planner")
		return {}

	def stop(self,data):
		self.state = "stop"
		rospy.logwarn("Stop planner")
		return {}

	def load_plan(self,data):
		self.planner = convert_plans.load_from_ROS(data.plan)
		rospy.logwarn("File loaded")
		rospy.logwarn("Period:"+str(self.planner.period))
		return {}

	def reset(self):
		self.state = "stop"
		self.planner.fts_state = None
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