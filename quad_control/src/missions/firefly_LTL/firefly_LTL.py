#!/usr/bin/env python

# import Misson abstract class
from .. import mission

from converter_between_standards.rotorS_converter import RotorSConverter

# node will publish motor speeds
from mav_msgs.msg import Actuators

#node will subscribe to odometry measurements
from nav_msgs.msg import Odometry


# import yaw controllers dictionary
from yaw_rate_controllers import yaw_controllers_database


import math
import numpy as np

# for subscribing to topics, and publishing
import rospy

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state,quad_speed_controller_cmd

from controllers.fa_trajectory_tracking_controllers.simple_pid_speed_controller.simple_pid_controller import SimplePIDSpeedController

# import yaw controllers dictionary
from yaw_rate_controllers.neutral_yaw_controller.neutral_yaw_controller import NeutralYawController

from quad_control.srv import GotoPosition
from std_srvs.srv import Empty,SetBool


import threading
import time

from missions.ltl_mission.planner import MonotonePlanner



class LTLMission(mission.Mission):

    inner = {}


    @classmethod
    def description(cls):
        string = """
        LTL planner for Firefly
        """
        
        return string
    
    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        # initialize time instant, and  state
        mission.Mission.__init__(self)

        # converting our controlller standard into rotors standard
        self.RotorSObject = RotorSConverter()

        # subscriber to odometry from RotorS
        self.sub_odometry = rospy.Subscriber(
            "/firefly/ground_truth/odometry",
            Odometry,
            self.get_state_from_rotorS_simulator
            )

        # publisher: command firefly motor speeds 
        self.pub_motor_speeds = rospy.Publisher(
            '/firefly/command/motor_speed',
            Actuators,
            queue_size=10
            )      

        # dy default, desired trajectory is staying still in origin
        self.reference     = np.array([0]*9)

        # controllers selected by default
        self.ControllerObject = SimplePIDSpeedController()

        self.YawControllerObject = NeutralYawController()


        rospy.Subscriber("quad_speed_controller_cmd", quad_speed_controller_cmd, self.set_command)
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)
        
        rospy.Service('wait_at_position', GotoPosition, self.goto)
        rospy.Service('use_speed_controller', SetBool, self.use_speed_controller)

        self.command = np.zeros(3*5)
        self.reference = np.zeros(3*5)

        self.position_reference = None
        self.is_speed_controller = False


    def initialize_state(self):
        # state of quad: position, velocity and attitude
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = np.zeros(3+3+3)

        
    def __str__(self):
        return self.description()
        # Add the self variables
        
        
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()


    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = np.zeros(3)
    	return np.concatenate([euler_rad,euler_rad_dot])

    def get_reference(self,time_instant):
        self.reference = self.command
        if self.is_speed_controller:
            self.reference[0:3] = self.state_quad[0:3]

        return self.command


    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc_output):
		# publish message
		self.pub_motor_speeds.publish(self.RotorSObject.rotor_s_message(desired_3d_force_quad,yaw_rate))


    # callback when ROTORS simulator publishes states
    def get_state_from_rotorS_simulator(self,odometry_rotor_s):        
        # RotorS has attitude inner loop that needs to known attitude of quad
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        # get state from rotorS simulator
        self.state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)
    

    def goto(self,data):
        if not self.is_speed_controller:
            self.position_reference = np.array([data.x,data.y,data.z])
            self.command  = np.concatenate([self.position_reference,np.zeros(4*3)])
        return {}

    def set_command(self,data):
        if self.is_speed_controller:
            vel = np.array([data.vx,data.vy,0])
            self.command  = np.concatenate([self.state_quad[0:3],vel,np.zeros(3*3)]) 
            rospy.logwarn(self.command)

    def use_speed_controller(self,data):
        self.is_speed_controller=data.data
        return {"success":True,"message":""}
