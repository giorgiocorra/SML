# import Misson abstract class
from .. import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_between_standards.iris_plus_converter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state,quad_speed_controller_cmd

from controllers.fa_trajectory_tracking_controllers.simple_pid_speed_controller.simple_pid_controller import SimplePIDSpeedController
from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database

# import yaw controllers dictionary
from yaw_rate_controllers.neutral_yaw_controller.neutral_yaw_controller import NeutralYawController

from quad_control.srv import GotoPosition
from std_srvs.srv import Empty,SetBool

import math
import numpy

from mavros_msgs.msg import OverrideRCIn

# for subscribing to topics, and publishing
import rospy

import threading
import time

from planner import MonotonePlanner

import utilities.mocap_source as mocap_source

from utilities.utility_functions import Velocity_Filter

class LTLMission(mission.Mission):

    inner = {}
    inner['controller'] = fa_trajectory_tracking_controllers_database.database

    @classmethod
    def description(cls):
        return "Speed tuning"

    
    def __init__(self,controller     = fa_trajectory_tracking_controllers_database.database["Default"]()):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any
        self.IrisPlusConverterObject = IrisPlusConverter()

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator)

        rospy.Subscriber("quad_speed_controller_cmd", quad_speed_controller_cmd, self.set_command)
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)

        # controllers selected by default
        self.ControllerObject = controller

        self.YawControllerObject = NeutralYawController()

        self.command = numpy.zeros(3*5)
        self.reference = numpy.zeros(3*5)

        self.position_reference = None

        mission.Mission.__init__(self)
        
        self.command = numpy.zeros(3*5)
        self.reference = numpy.zeros(3*5)

        self.command[2] = 1.0
        self.reference[2] = 1.0

        self.position_reference = None
        self.is_speed_controller = False

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


    def use_speed_controller(self,data):
        self.is_speed_controller=data.data
        return {"success":True,"message":""}

    def get_quad_ea_rad(self):
    	euler_rad     = self.state_quad[6:9]*math.pi/180
    	euler_rad_dot = numpy.zeros(3)
    	return numpy.concatenate([euler_rad,euler_rad_dot])


    def get_reference(self,time_instant):
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
        self.command[3:6]  = vel
        rospy.logwarn(self.command)

    def get_command(self):
        return self.command

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
