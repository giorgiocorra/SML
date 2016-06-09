# import Misson abstract class
from .. import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_between_standards.iris_plus_converter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state,quad_speed_cmd_3d

from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database

# import yaw controllers dictionary
from yaw_rate_controllers.neutral_yaw_controller.neutral_yaw_controller import NeutralYawController
from yaw_rate_controllers.simple_tracking_yaw_controller.simple_tracking_yaw_controller import SimpleTrackingYawController
from yaw_rate_controllers import yaw_controllers_database

from quad_control.srv import GotoPosition, GotoPose
from std_srvs.srv import Empty,SetBool

from utilities.utility_functions import psi_theta_from_unit_vec
import math
import numpy

# for subscribing to topics, and publishing
import rospy

import threading
import time

#from planner import MonotonePlanner

class LTLMission(mission.Mission):

    inner = {}
    inner['controller'] = fa_trajectory_tracking_controllers_database.database
    inner['yaw_controller'] = yaw_controllers_database.database

    @classmethod
    def description(cls):
        return "LTL planner"

    
    def __init__(self,controller=fa_trajectory_tracking_controllers_database.database["Default"](),yaw_controller = yaw_controllers_database.database["Default"]()):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)

        # service for setting camera orientation

        self.place_service = rospy.Service('PlaceTheCamera', GotoPose, self.place_camera)

        rospy.Service('start_speed_control', Empty, self.start_speed_control)
        
        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator)

        rospy.Subscriber("quad_speed_cmd_3d", quad_speed_cmd_3d, self.set_command)
        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)
        
        # controllers selected by default
        self.ControllerObject = controller
        self.YawControllerObject = yaw_controller

        self.speed_planner_is_active = False

        # SimpleTrackingYawController(gain=5.0)

        self.command_filtered = numpy.zeros(3*5)        # the one that is sent to the quad
        self.reference = numpy.zeros(3*5)

        self.command  = self.command_filtered.copy()    # the one that is computed by the planner (received on "quad_speed_cmd")

    def initialize_state(self):
        # state of quad: position, velocity, attitude, angular velocity 
        # PITCH, ROLL, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3+3) 
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
         
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        #self.sub_odometry.unregister()
        self.place_service.shutdown()

    def shutdown(self):
        self.stop_service.shutdown()
        self.place_service.shutdown()

    def get_quad_ea_rad(self):
    	return numpy.array(self.state_quad[6:12])

    def get_reference(self,time_instant):
        p=0.05
        self.command_filtered = (1.0-p) * self.command_filtered + p * self.command
        # rospy.logerr("Command: ")
        # rospy.logwarn("Position: " + str(self.command_filtered[0:3]))
        # rospy.logwarn("Velocity: " + str(self.command_filtered[3:6]))
        # rospy.logwarn("Angles: " + str(self.command_filtered[9:12]))
        # rospy.logwarn("Omega: " + str(self.command_filtered[12:15]))
        return self.command_filtered

    def get_state(self):
        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.command_filtered[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]

    def get_desired_yaw_rad(self,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        return self.command[9:15]     # yaw angle and its derivative


    def real_publish(self,desired_3d_force_quad,yaw_rate,rc):

        euler_rad     = self.state_quad[6:9]

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

        # FAKE CAMERA PITCH
        #self.camera_pitch = 
        # position
        p = numpy.array([simulator_message.x,simulator_message.y,simulator_message.z])
        # velocity
        v = numpy.array([simulator_message.vx,simulator_message.vy,simulator_message.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([simulator_message.pitch,simulator_message.roll,simulator_message.yaw])
        # euler angles derivative (not returned in simulator messages)
        ww = numpy.zeros(3)
        # collect all components of state
        self.state_quad = numpy.concatenate([p,v,ee,ww])  

    def set_command(self,data):
        if self.speed_planner_is_active:
            pos = self.state_quad[0:3]
            vel = [data.vx, data.vy, data.vz]
            acc = numpy.zeros(3)
            angles = self.state_quad[6:9]
            omega = [data.w_pitch, data.w_roll, data.w_yaw]
            self.command = numpy.concatenate([pos, vel, acc, angles, omega])
        #rospy.logwarn("received message: " + str(self.command))

    def get_command(self):
        return self.command_filtered

    # Switch to the default controller and stop
    def stop_the_quad(self,data = None):

        self.speed_planner_is_active = False
        self.stop_planner()

        if self.ControllerObject.__class__ != self.inner['controller']["Default"]:
            rospy.logwarn("Switch to position controller")
            ControllerClass      = self.inner['controller']["Default"]
            self.ControllerObject = ControllerClass()
        
        if self.YawControllerObject.__class__ != self.inner['yaw_controller']["NeutralYawController"]:
            rospy.logwarn("Switch to neutral yaw controller")
            ControllerClass      = self.inner['yaw_controller']["NeutralYawController"]
            self.YawControllerObject = ControllerClass()

        self.command[0:3] = self.state_quad[0:3]
        self.command_filtered = self.command.copy()
        rospy.logwarn("Stop the quad at " + str(self.command[0:3]))
        return {}

    # callback for service for initial positioning of camera
    def place_camera(self,data = None):

        self.speed_planner_is_active = False
        self.stop_planner()

        if self.ControllerObject.__class__ != self.inner['controller']["Default"]:
            rospy.logwarn("Switch to position controller")
            ControllerClass      = self.inner['controller']["Default"]
            self.ControllerObject = ControllerClass()

        if self.YawControllerObject.__class__ != self.inner['yaw_controller']["Default"]:
            rospy.logwarn("Switch to yaw angle controller")
            ControllerClass      = self.inner['yaw_controller']["Default"]
            self.YawControllerObject = ControllerClass()
        
        self.command[0:3] = numpy.array([data.x,data.y,data.z])
        self.command[3:6] = numpy.array([0.,0.,0.])
        psi,theta = psi_theta_from_unit_vec([data.v0,data.v1,data.v2])
        self.command[9:12] = numpy.array([theta,0.,psi])
        self.command[12:15] = numpy.array([0.,0.,0.])
        self.command_filtered = self.command.copy()
        rospy.logwarn("Stop the quad at (p,v,a,ee,ww): " + str(self.command))
        return {}

    def stop_planner(self):
        service_name = "stop_planner"
        rospy.wait_for_service(service_name,2.0)
        stop_planner_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        stop_planner_srv()

    def start_speed_control(self,data=None):

        self.speed_planner_is_active = True
        self.start_planner()

        if self.ControllerObject.__class__ != self.inner['controller']["Default"]:
            rospy.logwarn("Switch to position controller")
            ControllerClass      = self.inner['controller']["Default"]
            self.ControllerObject = ControllerClass()

        if self.YawControllerObject.__class__ != self.inner['yaw_controller']["Default"]:
            rospy.logwarn("Switch to yaw angle controller")
            ControllerClass      = self.inner['yaw_controller']["Default"]
            self.YawControllerObject = ControllerClass()

        return{}

    def start_planner(self):
        service_name = "start_planner"
        rospy.wait_for_service(service_name,2.0)
        start_planner_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        start_planner_srv()
