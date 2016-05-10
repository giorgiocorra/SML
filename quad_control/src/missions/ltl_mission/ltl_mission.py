#!/usr/bin/env python

# import Misson abstract class
from .. import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_between_standards.iris_plus_converter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state,quad_speed_controller_cmd

from controllers.fa_trajectory_tracking_controllers.simple_pid_speed_controller.simple_pid_controller import SimplePIDSpeedController

# import yaw controllers dictionary
from yaw_rate_controllers.neutral_yaw_controller.neutral_yaw_controller import NeutralYawController


import math
import numpy

# for subscribing to topics, and publishing
import rospy


import threading
import time

class LTLMission(mission.Mission):

    inner = {}

    @classmethod
    def description(cls):
        return "Iris, simulated, to track a desired trajectory"

    
    def __init__(self):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)
        
        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator)

        rospy.Subscriber("quad_speed_controller_cmd", quad_speed_controller_cmd, self.set_command)
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)
        
        # controllers selected by default
        self.ControllerObject = SimplePIDSpeedController()

        self.YawControllerObject = NeutralYawController()

        self.command = numpy.zeros(3*5)
        self.reference = numpy.zeros(3*5)

    def initialize_state(self):
        # state of quad: position, velocity and attitude 
        # ROLL, PITCH, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3) 
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
         
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.sub_odometry.unregister()


    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])


    def get_reference(self,time_instant):
        self.reference = self.command
        return self.command

    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.command[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc):

        euler_rad     = self.state_quad[6:9]*math.pi/180 

        self.IrisPlusConverterObject.set_rotation_matrix(euler_rad)
        iris_plus_rc_input = self.IrisPlusConverterObject.input_conveter(desired_3d_force_quad,yaw_rate)

        # create a message of the type quad_cmd, that the simulator subscribes to 
        cmd  = quad_cmd()
        
        cmd.cmd_1 = iris_plus_rc_input[0]
        cmd.cmd_2 = iris_plus_rc_input[1]
        cmd.cmd_3 = iris_plus_rc_input[2]
        cmd.cmd_4 = iris_plus_rc_input[3]

        cmd.cmd_5 = 1500.0
        cmd.cmd_6 = 1500.0
        cmd.cmd_7 = 1500.0
        cmd.cmd_8 = 1500.0
        
        self.pub_cmd.publish(cmd)


    # callback when simulator publishes states
    def get_state_from_simulator(self,simulator_message):

        # position
        p = numpy.array([simulator_message.x,simulator_message.y,simulator_message.z])
        # velocity
        v = numpy.array([simulator_message.vx,simulator_message.vy,simulator_message.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([simulator_message.roll,simulator_message.pitch,simulator_message.yaw])
        # collect all components of state
        self.state_quad = numpy.concatenate([p,v,ee])  

    def set_command(self,data):
        vel = numpy.array([data.vx,data.vy,0])
        self.command  = numpy.concatenate([numpy.zeros(3),vel,numpy.zeros(3*3)]) 
        rospy.logwarn(self.command)

    def get_command(self):
        return self.command
