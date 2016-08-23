# import Misson abstract class
from .. import mission

# import converter from 3d_force and yaw rate into iris rc standard 
from converter_between_standards.iris_plus_converter import IrisPlusConverter

# publish message quad_cmd that contains commands to simulator
from quad_control.msg import quad_cmd, quad_state,quad_speed_cmd_3d, camera_pose
from std_msgs.msg import Float64

from controllers.fa_trajectory_tracking_controllers import fa_trajectory_tracking_controllers_database

# import yaw controllers dictionary
from yaw_rate_controllers.neutral_yaw_controller.neutral_yaw_controller import NeutralYawController
from yaw_rate_controllers.simple_tracking_yaw_controller.simple_tracking_yaw_controller import SimpleTrackingYawController
from yaw_rate_controllers import yaw_controllers_database

from quad_control.srv import GotoPosition, GotoPose, LandmarksTrade, Filename, Time
from std_srvs.srv import Empty,SetBool

from utilities.utility_functions import skew, lat_long_from_unit_vec, unit_vec_from_lat_long
from utilities.coverage_giorgio import Camera, Landmark_list, read_lmks_from_file
import math
cos = math.cos
sin = math.sin
pi = math.pi

import numpy
norm = numpy.linalg.norm

# for subscribing to topics, and publishing
import rospy
get = rospy.get_param

import rospkg
import sys
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('quad_control'))

# for launching node of collision avoidance, only if necessary
import roslaunch

import threading
import time

import json

import utilities.mocap_source as mocap_source
from utilities.utility_functions import Velocity_Filter,Velocity_Mean_Filter

from mavros_msgs.msg import OverrideRCIn
#from planner import MonotonePlanner

class LTLMission(mission.Mission):

    inner = {}
    inner['controller'] = fa_trajectory_tracking_controllers_database.database
    inner['yaw_controller'] = yaw_controllers_database.database

    @classmethod
    def description(cls):
        D_OPT = get("/D_OPT",3.)
        coll_av = get("/collision_avoidance_active", True)
        mag_ctrl = get("/magnitude_control_type","None")
        n_ag = len(get("/name_agents", '').rsplit(' '))
        ag = rospy.get_namespace()[1:]
        return "Vision mission real, %d agents.\n Agent %s \n D_OPT = %f \n Collision avoidance = %s \n Magnitude control = %s \n" % (n_ag,ag, D_OPT, coll_av, mag_ctrl)

    
    def __init__(self,body_id=1,controller=fa_trajectory_tracking_controllers_database.database["Default"](),yaw_controller = yaw_controllers_database.database["Default"]()):
        # Copy the parameters into self variables
        # Subscribe to the necessary topics, if any

        mission.Mission.__init__(self)

        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # establish connection to qualisys
        self.Qs = mocap_source.Mocap(info=0)

        self.body_id = body_id

        # publisher: command firefly motor speeds    
        self.rc_override = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=100)

        # intiialization should be done in another way,
        # but median will take care of minimizing effects
        self.VelocityEstimator = Velocity_Mean_Filter(4,numpy.zeros(3),0.0)


        self.namespace = rospy.get_namespace()[1:]

        self.start_planner()

        self.quad_position_publisher = rospy.Publisher('quad_state', quad_state, queue_size=100)

        # service for setting camera orientation

        self.place_service = rospy.Service('PlaceTheCamera', GotoPose, self.place_camera)

        self.set_yaw_srv = rospy.Service('set_yaw_value', GotoPose, self.set_yaw_handle)

        self.take_off_service = rospy.Service('slow_take_off', Empty, self.take_off)

        self.load_lmks_srv = rospy.Service("/"+self.namespace+'load_lmks_mission', Filename, self.load_lmks_callback)

        self.start_speed_control_srv = rospy.Service('start_speed_control', Empty, self.start_speed_control)

        self.speed_cont = rospy.Service('use_speed_controller',SetBool, self.use_speed_controller)

        self.reset_t0_srv = rospy.Service('reset_t0', Time, self.reset_t0)

        # converting our controlller standard into iris+ standard
        self.IrisPlusConverterObject = IrisPlusConverter()

        # message published by quad_control that simulator will subscribe to 
        self.pub_cmd = rospy.Publisher('quad_cmd', quad_cmd, queue_size=10)

        # publish new camera position and orientation for the planner
        self.pub_cam_pose = rospy.Publisher('camera_pose', camera_pose, queue_size = 10)

        # controller needs to have access to STATE: comes from simulator
        self.SubToSim = rospy.Subscriber("quad_state", quad_state, self.get_state_from_simulator)

        # # subscribe to velocity message published by the planner or by collision avoidance node
        # collision_av_active = rospy.get_param("collision_avoidance_active", True)

        # if (collision_av_active):
        #     rospy.Subscriber("quad_speed_cmd_avoid", quad_speed_cmd_3d, self.set_command)
        # else:
        #     rospy.Subscriber("quad_max_speed_cmd", quad_speed_cmd_3d, self.set_command)

        rospy.Subscriber("quad_speed_magnitude",quad_speed_cmd_3d, self.set_command)
        rospy.Subscriber("quad_speed_3d_test",quad_speed_cmd_3d, self.set_command_speed_test)

        # Subscriptions just for saving data
        #rospy.Subscriber("camera_pose", camera_pose, self.camera_pose_callback)
        rospy.Subscriber("quad_max_speed_cmd", quad_speed_cmd_3d,  self.max_speed_callback)
        rospy.Subscriber("quad_speed_cmd_avoid", quad_speed_cmd_3d, self.avoid_speed_callback)
        #rospy.Subscriber("quad_speed_magnitude", quad_speed_cmd_3d, self.magnitude_speed_callback)
        rospy.Subscriber("vision", Float64, self.vision_callback)


        # Trading landmarks
        if (rospy.get_param("/trade_allowed",False)):
            self.change_lmks_srv = rospy.Service("/"+self.namespace+'change_landmarks', LandmarksTrade, self.change_landmarks_handle)
        
        # controllers selected by default
        self.ControllerObject = controller
        self.YawControllerObject = yaw_controller

        # SimpleTrackingYawController(gain=5.0)

        #self.command_filtered = numpy.zeros(3*5)        # the one that is sent to the quad
        #self.command_filtered[2] = 1                    # start in [0,0,1]
        #self.cam_command_filtered = numpy.zeros(2)
        #self.reference = numpy.zeros(3*5)


        self.command  = numpy.zeros(3*5)    # the one that is computed by the planner (received on "quad_speed_cmd")
        #self.cam_command = self.cam_command_filtered.copy()

        

    def initialize_state(self):
        # state of quad: position, velocity, attitude, angular velocity 
        # PITCH, ROLL, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = numpy.zeros(3+3+3+3)
        self.state = 'stop'
        self.counter = 0.
        self.changed_state = False
        self.long_rate = 0.
        self.lat_rate = 0.
        self.long = 0.
        self.lat = 0.
        self.lmks = Landmark_list([])
        # For saving data
        self.p_dot_max = [0.] *3
        self.omega_max = [0.] *3
        self.p_dot_dir = [0.] *3
        self.p_dot_star = [0.] *3
        self.omega_star = [0.] *3
        self.vision = 0.
        self.n_lmks = 0
        self.q = [0.] *3
        self.u = [0.] *3
        self.vel_buffer = [10.] * rospy.get_param("/buffer_length",50)
        self.omega_buffer = [10.] * rospy.get_param("/buffer_length",50)
        
        
    def __str__(self):
        return self.description()
        # Add the self variables
        
         
    def __del__(self):
        # Unsubscribe from all the subscribed topics
        self.rc_override.unregister()

        self.place_service.shutdown()

    def shutdown(self):
        self.stop_service.shutdown()
        self.place_service.shutdown()
        self.take_off_service.shutdown()
        self.load_lmks_srv.shutdown()
        self.start_speed_control_srv.shutdown()
        self.speed_cont.shutdown()
        self.set_yaw_srv.shutdown()
        self.reset_t0_srv.shutdown()
        if (rospy.get_param("/trade_allowed",False)):
            self.change_lmks_srv.shutdown()

    def get_quad_ea_rad(self):
        euler_rad     = self.state_quad[6:9]*math.pi/180
        euler_rad_dot = numpy.zeros(3)
        return numpy.concatenate([euler_rad,euler_rad_dot])

    def get_reference(self,time_instant):
        #p=0.05
        #self.command_filtered = (1.0-p) * self.command_filtered + p * self.command
        #self.cam_command_filtered = (1.0-p) * self.cam_command_filtered + p * self.cam_command
        # rospy.logerr("Command: ")
        # rospy.logwarn("Position: " + str(self.command_filtered[0:3]))
        # rospy.logwarn("Velocity: " + str(self.command_filtered[3:6]))
        # rospy.logwarn("Angles: " + str(self.command_filtered[9:12]))
        # rospy.logwarn("Omega: " + str(self.command_filtered[12:15]))
        return self.command

    def get_state(self):
        bodies = self.Qs.get_body(self.body_id)

        if bodies != 'off':  

            self.flag_measurements = True

            # position
            x=bodies["x"]; y=bodies["y"]; z=bodies["z"]   
            p = numpy.array([x,y,z])

            # velocity
            #v = numpy.array([data.vx,data.vy,data.vz])
            v = self.VelocityEstimator.out(p,rospy.get_time())

            # attitude: euler angles THESE COME IN DEGREES
            roll = bodies["roll"]; pitch=-bodies["pitch"]; yaw  = bodies["yaw"]
            ee = numpy.array([roll,pitch,yaw])

            # collect all components of state
            self.state_quad = numpy.concatenate([p,v,ee])  

            # Fake camera orientation
            t_sim = 1./100.         # the simulator node works at 100 hz
            self.long = ee[2]/180*pi
            self.lat += self.lat_rate * t_sim

            #rospy.logerr('Position : ' + str(p) + ' / Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))

            v_camera = unit_vec_from_lat_long(self.long,self.lat)

            cam_pose = camera_pose()
            cam_pose.px = p[0]
            cam_pose.py = p[1]
            cam_pose.pz = p[2]
            cam_pose.vx = v_camera[0]
            cam_pose.vy = v_camera[1]
            cam_pose.vz = v_camera[2]
            cam_pose.time = 0

            self.pub_cam_pose.publish(cam_pose)

            if (self.state=='position')and(norm(self.command[0:3]-p)<1e-1)and(norm(self.command[11]-self.long)<1e-1):         # TODO: add condition on orientation of camera
                rospy.logerr(self.namespace + ': Initial position reached')
                self.state = 'stop'
        else:
            # do nothing, keep previous state
            self.flag_measurements = False

        quad_pos = quad_state()
        quad_pos.x = self.state_quad[0]
        quad_pos.y = self.state_quad[1]
        quad_pos.z = self.state_quad[2]
        quad_pos.vx = self.state_quad[3]
        quad_pos.vy = self.state_quad[4]
        quad_pos.vz = self.state_quad[5]
        quad_pos.roll = self.state_quad[6]
        quad_pos.pitch = self.state_quad[7]
        quad_pos.yaw = self.state_quad[8]

        self.quad_position_publisher.publish(quad_pos)


        return self.state_quad


    def get_pv(self):
        return self.state_quad[0:6]


    def get_pv_desired(self):
        return self.command[0:6]


    def get_euler_angles(self):
        return self.state_quad[6:9]

    def get_desired_yaw_rad(self,time_instant):
        '''Get desired yaw in radians, and its time derivative'''
        return [self.command[11], self.command[14]]     # yaw angle and its derivative

    def real_publish(self,desired_3d_force_quad,yaw_rate,rc_output):

        # ORDER OF INPUTS IS VERY IMPORTANT: roll, pitch, throttle,yaw_rate
        # create message of type OverrideRCIn
        # rospy.logwarn("Yaw rate: " + str(yaw_rate))
        rc_cmd          = OverrideRCIn()
        other_channels  = numpy.array([1500,1500,1500,1500])
        channels        = numpy.concatenate([rc_output,other_channels])
        channels        = numpy.array(channels,dtype=numpy.uint16)
        rc_cmd.channels = channels

        self.rc_override.publish(rc_cmd)   

    # callback when simulator publishes states
    def get_state_from_simulator(self,simulator_message):

        # position
        p = numpy.array([simulator_message.x,simulator_message.y,simulator_message.z])
        # velocity
        p_dot = numpy.array([simulator_message.vx,simulator_message.vy,simulator_message.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([simulator_message.roll,simulator_message.pitch,simulator_message.yaw])
        # euler angles derivative (not returned in simulator messages)
        ee_dot = numpy.zeros(3)
        # collect all components of state
        self.state_quad = numpy.concatenate([p,p_dot,ee,ee_dot])

        # Fake camera orientation
        t_sim = 1./100.         # the simulator node works at 100 hz
        self.long = ee[2]/180*pi
        self.lat += self.lat_rate * t_sim

        #rospy.logerr('Position : ' + str(p) + ' / Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))

        v_camera = unit_vec_from_lat_long(self.long,self.lat)

        cam_pose = camera_pose()
        cam_pose.px = p[0]
        cam_pose.py = p[1]
        cam_pose.pz = p[2]
        cam_pose.vx = v_camera[0]
        cam_pose.vy = v_camera[1]
        cam_pose.vz = v_camera[2]
        cam_pose.time = simulator_message.time

        self.pub_cam_pose.publish(cam_pose)

        if (self.state=='position')and(norm(self.command[0:3]-p)<1e-1)and(norm(self.command[11]-self.long)<1e-1):         # TODO: add condition on orientation of camera
            rospy.logerr(self.namespace + ': Initial position reached, ready to start the mission.')
            #self.stop_the_quad()


    def set_command(self,data):

        # For saving data
        self.p_dot_star = [data.vx, data.vy, data.vz]
        self.omega_star = [data.wx,data.wy,data.wz]

        if (self.state=='speed'):
            pos = self.state_quad[0:3]
            vel = [data.vx, data.vy, data.vz]
            acc = numpy.zeros(3)
            angles = self.get_quad_ea_rad()[0:3]

            omega_xyz = [data.wx,data.wy,data.wz]
            
            psi = self.long
            self.long_rate = omega_xyz[2]
            self.lat_rate = sin(psi)*omega_xyz[0] - cos(psi)*omega_xyz[1] 
            
            omega = [0, 0, self.long_rate]

            if (self.changed_state):
                if (any(vel)) or (self.counter<0):
                    self.changed_state = False
                else:
                    self.counter-=1

            #rospy.logwarn('Velocity: ' + str(vel) + ' Yaw rate: ' + str(self.long_rate) + ' Pitch rate: ' + str(self.lat_rate))
            if (not self.changed_state):
                self.vel_buffer.append(norm(vel))
                self.omega_buffer.append(norm(omega_xyz))
                del self.vel_buffer[0]
                del self.omega_buffer[0]
                l = rospy.get_param("/buffer_length",50)
                if any(self.vel_buffer[k] < 1e-1 for k in range(l)) and any(self.omega_buffer[k] < 1e-1 for k in range(l)):
                    rospy.logerr(self.namespace + ': Final position reached')
                    self.stop_the_quad()
                    if (rospy.get_param("/trade_allowed",False)):
                        rospy.logerr(self.namespace + ': Ready to trade')
                        name_agent = self.namespace[:-1]        # remove the final character '/'
                        q, u = self.lmks.to_lists()
                        service_name = "/trade_request"
                        rospy.wait_for_service(service_name,2.0)
                        trade_request_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
                        trade_request_srv(q = q, u = u, name_agent = name_agent)
                else:
                    self.command = numpy.concatenate([pos, vel, acc, angles, omega])

            # #rospy.logwarn('Velocity: ' + str(vel) + ' Yaw rate: ' + str(self.long_rate) + ' Pitch rate: ' + str(self.lat_rate))
            # if (not self.changed_state)and(norm(vel)<1e-1)and(norm(omega_xyz)<1e-1):
            #     rospy.logerr(self.namespace + ': Final position reached')
            #     self.stop_the_quad()
            #     if (rospy.get_param("/trade_allowed",False)):
            #         rospy.logerr(self.namespace + ': Ready to trade')
            #         name_agent = self.namespace[:-1]        # remove the final character '/'
            #         q, u = self.lmks.to_lists()
            #         service_name = "/trade_request"
            #         rospy.wait_for_service(service_name,2.0)
            #         trade_request_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
            #         trade_request_srv(q = q, u = u, name_agent = name_agent)

            # else:
            #     self.command = numpy.concatenate([pos, vel, acc, angles, omega])

    def set_command_speed_test(self,data):
        if (self.state == 'test_speed_control'):
            pos = numpy.zeros(3)
            vel = [data.vx, data.vy, data.vz]
            acc = numpy.zeros(3)
            angles = self.command[9:12]
            omega = [data.wx, data.wy, data.wz] 
            self.command = numpy.concatenate([pos, vel, acc, angles, omega])

    def get_command(self):
        return self.command

    # Switch to the default controller and stop
    def stop_the_quad(self,data = None):

        self.state = 'stop'
        self.changed_state = False              # just in case it wasn't already reset
        #self.stop_planner()

        # if self.ControllerObject.__class__ != self.inner['controller']["SimplePIDSpeedController_3d"]:
        #     rospy.logwarn("Switch to velocity controller")
        #     ControllerClass      = self.inner['controller']["SimplePIDSpeedController_3d"]
        #     self.ControllerObject = ControllerClass()
        
        # if self.YawControllerObject.__class__ != self.inner['yaw_controller']["NeutralYawController"]:
        #     rospy.logwarn(self.namespace + "Switch to neutral yaw controller")
        #     ControllerClass      = self.inner['yaw_controller']["NeutralYawController"]
        #     self.YawControllerObject = ControllerClass()

        self.command[0:3] = self.state_quad[0:3]
        self.command[3:6] = numpy.array([0.,0.,0.])
        self.command[6:9] = numpy.array([0.,0.,0.])
        self.command[9:12] = self.get_quad_ea_rad()[0:3]
        self.command[12:15] = numpy.array([0.,0.,0.])
        self.lat_rate = 0
        self.long_rate = 0
        rospy.logerr(self.namespace + ": Stopping the quad: ")
        rospy.logwarn('Position: ' + str(self.command[0:3]))
        rospy.logwarn('Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))
        return {}

    # callback for service for initial positioning of camera
    def place_camera(self,data = None):

        self.changed_state = False              # just in case it wasn't already reset
        #self.start_planner()

        # if self.ControllerObject.__class__ != self.inner['controller']["SimplePIDController"]:
        #     rospy.logwarn("Switch to position controller")
        #     ControllerClass      = self.inner['controller']["SimplePIDController"]
        #     self.ControllerObject = ControllerClass()

        # if self.YawControllerObject.__class__ != self.inner['yaw_controller']["Default"]:
        #     rospy.logwarn("Switch to yaw angle controller")
        #     ControllerClass      = self.inner['yaw_controller']["Default"]
        #     self.YawControllerObject = ControllerClass()
        
        self.command[0:3] = numpy.array([data.x,data.y,data.z])
        self.command[3:6] = numpy.array([0.,0.,0.])
        self.command[6:9] = numpy.array([0.,0.,0.])
        self.command[9:12] = numpy.array([0.,0.,data.psi])
        self.command[12:15] = numpy.array([0.,0.,0.])
        #self.long = data.psi
        self.lat = data.theta
        self.long_rate = 0.
        self.lat_rate = 0.
        rospy.logerr(self.namespace + ': Placing the camera:')
        rospy.logwarn('Position: ' + str(self.command[0:3]))
        rospy.logwarn('Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))
        self.state = 'position'
        return {}

    def stop_planner(self):
        service_name = "stop_planner"
        rospy.wait_for_service(service_name,2.0)
        stop_planner_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        stop_planner_srv()

    def start_speed_control(self,data=None):

        self.counter = 50.
        self.vel_buffer = [10.] * rospy.get_param("/buffer_length",50)
        self.omega_buffer = [10.] * rospy.get_param("/buffer_length",50)
        self.changed_state = True               # to avoid stopping because of the delay in the answer of the planner

        self.state = 'speed'
        
        # if self.ControllerObject.__class__ != self.inner['controller']["SimplePIDSpeedController_3d"]:
        #     rospy.logwarn("Switch to velocity controller")
        #     ControllerClass      = self.inner['controller']["SimplePIDSpeedController_3d"]
        #     self.ControllerObject = ControllerClass()

        # if self.YawControllerObject.__class__ != self.inner['yaw_controller']["Default"]:
        #     rospy.logwarn("Switch to yaw angle controller")
        #     ControllerClass      = self.inner['yaw_controller']["Default"]
        #     self.YawControllerObject = ControllerClass()

        return{}

    def start_planner(self):
        service_name = "start_planner"
        rospy.wait_for_service(service_name,2.0)
        start_planner_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        start_planner_srv()

    def take_off(self, data = None):
        pose = GotoPose()
        pose.x = self.state_quad[0]
        pose.y = self.state_quad[1]
        pose.z = self.state_quad[2] + 0.5
        pose.psi = self.long
        pose.theta = self.lat
        self.place_camera(pose)
        return{}

    def load_lmks_callback(self, data):
        self.lmks = read_lmks_from_file(data.filename)
        q, u = self.lmks.to_lists()
        self.q = numpy.squeeze(q)
        self.u = numpy.squeeze(u)
        self.n_lmks = len(self.q)/3
        service_name = "/"+ self.namespace+'load_lmks_planner'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = q, u = u, name_agent = '')
        service_name = "/"+ self.namespace+'load_lmks_magnitude_control'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = q, u = u, name_agent = '')
        service_name = "/load_lmks_projector"
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = q, u = u, name_agent = self.namespace[:-1])
        return {}


    def change_landmarks_handle(self,data):
        rospy.logerr(self.namespace + ': Landmarks received')
        self.q = numpy.squeeze(data.q)
        self.u = numpy.squeeze(data.u)
        self.n_lmks = len(self.q)/3
        self.lmks = Landmark_list([])
        self.lmks.from_lists(q = data.q, u = data.u)
        service_name = "/"+ self.namespace+'load_lmks_planner'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = data.q, u = data.u, name_agent = '')
        service_name = "/"+ self.namespace+'load_lmks_magnitude_control'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = data.q, u = data.u, name_agent = '')
        service_name = "/load_lmks_projector"
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, LandmarksTrade,2.0)
        load_lmks_srv(q = data.q, u = data.u, name_agent = self.namespace[:-1])
        self.start_speed_control()
        return {}


    def  use_speed_controller(self,data):
        if data.data:
            self.state = 'test_speed_control'
            if self.ControllerObject.__class__ != self.inner['controller']["SimplePIDSpeedController_3d"]:
                rospy.logwarn("Switch to velocity controller")
                ControllerClass      = self.inner['controller']["SimplePIDSpeedController_3d"]
                self.ControllerObject = ControllerClass()
        else:
            self.stop_the_quad()

    def set_yaw_handle(self,data = None):
        
        self.command[9:12] = numpy.array([0.,0.,data.psi])      # given in degrees
        rospy.logwarn('Given yaw command: ' + str(data.psi) + ' degrees')
        return {}

    def reset_t0(self,data):
        if not(data):
            self.reset_initial_time()
        else:
            self.reset_initial_time(data.data)
        return {}


    def get_complementary_data(self):
        """Get complementary data that is not included 
        in get_complete_data() already
        """
        if self.state == 'stop':
            a = 1
        elif self.state == 'position':
            a = 2
        elif self.state == 'speed':
            a = 3
        elif self.state == 'test_speed_control':
            a = 4
        # by default return empty array
        return numpy.concatenate([[a],[self.long], [self.lat], self.p_dot_max, self.omega_max, self.p_dot_dir, self.p_dot_star, self.omega_star, [self.n_lmks], [self.vision], self.q, self.u])

    ############# Subscriptions only for saving data #############

    def max_speed_callback(self, data):
        self.p_dot_max = [data.vx, data.vy, data.vz]
        self.omega_max = [data.wx, data.wy, data.wz]

    def avoid_speed_callback(self, data):
        self.p_dot_dir = [data.vx, data.vy, data.vz]

    def vision_callback(self,data):
        self.vision = data.data