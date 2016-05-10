import sys
sys.path.append("/home/paul/workspace/master thesis/pwa/core")

import rospy

try:
	from quad_control.srv import PlannerStart
	from quad_control.msg import speed_controller_cmd
	from quad_control.msg import quad_state_and_cmd
except ImportError:
	pass
import pickle
import numpy as np

class MonotonePlanner():
	def __init__(self,monotone_system,fts,verbose=0):
		self.fts = fts
		self.monotone_system = monotone_system
		self.period = 0.1
		self.last_time = 0.0
		self.fts_state = None
		self.control = None
		self.speed_controller = None

		self.position = None

		self.execute_plan = False
		
		self.control = None

		self.verbose = verbose
		self.name = "planner"

	def init_state(self,position):
		cell = self.monotone_system.get_cell(position)
		for n in self.fts.graph['initial']:
			if cell==n[0]:
				self.fts_state = n
				break

		self.control = nx.get_node_attributes(self.fts,"apply_control")

		return self.control[self.fts_state]


	def get_next_state(self,position):
		cell = self.monotone_system.get_cell(position)
		for n in self.fts.successors(self.fts_state):
			if cell==n[0]:
				return n

	def transition_available(self,sim):
		return sim.time-self.last_time>=self.period

	def di_sim(self,position):
		prev_fts_state = self.fts_state
		self.fts_state = self.get_next_state()
		if self.fts_state:
			return self.control[self.fts_state]
		else:
			print "No fts state!!"
			print "------------------------"
			print "Time:",sim.time
			print "Current continuous state:",self.position
			print "Current observation:",self.monotone_system.get_cell(self.position)

			print "Previous state of the plan:",prev_fts_state
			print "Possible transitions:"
			cell = self.monotone_system.get_cell(self.position)
			for n in self.fts.successors(prev_fts_state):
				print n
			print "------------------------"
			return np.array([0,0])



class MonotonePlannerNode():
	def __init__(self):
		self.execute_plan = False
		self.planner = None

		rospy.init_node('planner', anonymous=True)

		self.speed_controller = rospy.Publisher(speed_controller_topic, speed_controller_cmd, queue_size=10)

		rospy.Suscriber(quad_state, quad_state_and_cmd,self.get_measure, queue_size=10)

		rospy.Service('StartPlanner', PlannerStart, self.start_plan_execution)


		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			if self.execute_plan:
				break
			rate.sleep()

		# Start of the plan
		rate = rospy.Rate(1.0/planner.period)

		while not rospy.is_shutdown():
			if self.execute_plan:
				if planner.fts_state == None:
					control = planner.init_state()
				else:
					control = planner.di_sim(planner.position)
				self.speed_controller.publish(speed_controller_cmd(control[0],control[1]))
			# go to sleep
			rate.sleep()

	def get_measure(self,data):
		if self.execute_plan:
			planner.position = data

	def start_plan_execution(self,data):
		self.planner = pickle.load(open(data.plan,"wb"))
		self.execute_plan = True

if __name__ == '__main__':
	try:
		MonotonePlannerNode()
	except rospy.ROSInterruptException:
		pass
