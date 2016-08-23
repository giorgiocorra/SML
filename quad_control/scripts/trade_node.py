#!/usr/bin/env python

import rospy
get = rospy.get_param


# import services defined in quad_control
# Four SERVICES ARE BEING USED: SaveData, ServiceTrajectoryDesired, Mocap_Id, StartSim
# SaveData is for saving data in txt file
# TrajDes is for selecting trajectory
# Mocap_Id for body detection from QUALISYS
# StartSim stop simulator
from quad_control.srv import LandmarksTrade
from std_srvs.srv import Empty
from quad_control.msg import camera_pose

# to work with directories relative to ROS packages
from rospkg import RosPack

import utilities.coverage_giorgio as cov

import numpy

# For reading landmarks
import json


class TradeNode():

	def __init__(self):

		self.frequency = 1e1

		rospy.init_node('trade_node', anonymous=True)

		self.D_OPT = get("/D_OPT",3.)
		self.name_agents = get("/name_agents", '').rsplit(' ')
		self.n_ag = len(self.name_agents)

		self.trade_matrix = numpy.zeros([self.n_ag, self.n_ag])

		self.cams = {}
		self.lmks = {}

		self.index_agents = {}
		i = 0
		for ns in self.name_agents:
			self.cams[ns] = cov.Camera()
			self.lmks[ns] = cov.Landmark_list([])
			self.index_agents[ns] = i
			i += 1
			rospy.Subscriber("/" + ns + "/camera_pose", camera_pose, lambda data,ns = ns: self.camera_pose_callback(data,ns))

		rospy.Service('trade_request', LandmarksTrade, self.trade_request_handle)

			


	def camera_pose_callback(self,data,ns):
		self.cams[ns].new_pose(data)

	def trade_request_handle(self,data):
		ns = data.name_agent
		self.lmks[ns] = cov.Landmark_list([])
		self.lmks[ns].from_lists(q = data.q, u = data.u)
		self.trade_matrix[self.index_agents[ns],self.index_agents[ns]] = 1
		return {}
        

	def run(self):
		rate = rospy.Rate(self.frequency)

		while not rospy.is_shutdown():
			for ns in self.name_agents:
				ag = self.index_agents[ns]
				if self.trade_matrix[ag,ag]:
					if all(self.trade_matrix[ag]):
						continue
					partner = ag
					while (self.trade_matrix[ag,partner]):
						partner += 1
						if partner >= self.n_ag:
							partner = 0
					ns_partner = 'to_change'
					for ns_i in self.name_agents:
						if self.index_agents[ns_i] == partner:
							ns_partner = ns_i
					if not(self.trade_matrix[partner,partner]):
						self.trade_matrix[ag,partner] = 1
					else:
						result,Q_ag,Q_partner = cov.trade_function(self.cams[ns],self.lmks[ns],self.cams[ns_partner],self.lmks[ns_partner],self.D_OPT)
						if result:
							rospy.logerr("Succesful trade between %s and %s" %(ns,ns_partner))
							self.trade_matrix[ag] = numpy.zeros(self.n_ag)
							self.trade_matrix[partner] = numpy.zeros(self.n_ag)
							q_ag, u_ag = Q_ag.to_lists()
							q_partner, u_partner = Q_partner.to_lists()
							service_name = "/"+ ns+'/change_landmarks'
							rospy.wait_for_service(service_name,2.0)
							change_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
							change_lmks_srv(q = q_ag, u = u_ag, name_agent = ns)
							service_name = "/"+ ns_partner + '/change_landmarks'
							rospy.wait_for_service(service_name,2.0)
							change_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
							change_lmks_srv(q = q_partner, u = u_partner, name_agent = ns_partner)
						else:
							self.trade_matrix[ag,partner] = 1
							self.trade_matrix[partner,ag] = 1

			if (self.trade_matrix.all().all()):
				rospy.logerr('Trade completed!')
			rate.sleep()



if (get("/trade_allowed",False)):
	ATradeNode = TradeNode()
	try:
		ATradeNode.run()
	except rospy.ROSInterruptException:
		pass