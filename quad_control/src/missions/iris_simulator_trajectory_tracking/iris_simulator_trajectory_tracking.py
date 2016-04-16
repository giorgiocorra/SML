#!/usr/bin/env python

# import Misson abstract class
from .. import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from ConverterBetweenStandards.IrisPlusConverter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state

# import list of available trajectories
from trajectories import trajectories_dictionary

# import controllers dictionary
from Yaw_Rate_Controller import yaw_controllers_dictionary
from yaw_controllers import yaw_controllers_dictionary


# import controllers dictionary
from controllers_hierarchical.fully_actuated_controllers import controllers_dictionary

import math
import numpy

# for subscribing to topics, and publishing
import rospy



class IrisSimulatorTrajectoryTracking(mission.Mission):

    inner = {}

    inner['controller']     = controllers_dictionary.controllers_dictionary
    inner['trajectory']     = trajectories_dictionary.trajectories_dictionary
    inner['yaw_controller'] = yaw_controllers_dictionary.yaw_controllers_dictionary    


    @classmethod
    def description(cls):
        return "Iris, simulated, to track a desired trajectory"

    
    def __init__(self,
            controller     = controllers_dictionary.controllers_dictionary['PIDSimpleBoundedIntegralController'](),
            trajectory     = trajectories_dictionary.trajectories_dictionary['StayAtRest'](),
            yaw_controller = yaw_controllers_dictionary.yaw_controllers_dictionary['TrackingYawController']()
            ):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)
        
        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator)

        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)
        
        # dy default, desired trajectory is staying still in origin
        self.TrajGenerator = trajectory
        self.reference     = self.TrajGenerator.output(self.time_instant_t0)

        # controllers selected by default
        self.ControllerObject = controller

        self.YawControllerObject = yaw_controller

        pass  


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
        self.reference = self.TrajGenerator.output(time_instant)
        return self.reference
        # return numpy.zeros(3*5)


    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.reference[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]


    def real_publish(self,desired_3d_force_quad,yaw_rate):

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

