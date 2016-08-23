#!/usr/bin/env python
# this line is just used to define the type of document

import numpy as np

from .. import yaw_controller as yc

from utilities.utility_functions import bound

import rospy

class YawController_PI(yc.YawController):

    @classmethod
    def description(cls):
        return "Simple yaw tracking controller, based on <b>feedback linearization of yaw rate equation</b>"

    def __init__(self, k_p = 1.0, k_i = 0.5, bound = 1.0, k_a = 0.2):
        self.k_p = k_p
        self.k_i = k_i
        self.bound = bound
        self.bound_i = 0.2 * bound
        self.integrate = 0.
        self.t_old = rospy.get_time()
        self.anti_wind_up = 0.
        self.k_a = k_a
    
    
    def __str__(self):
        string = yc.YawController.__str__(self)
        string += "\nProportional gain: " + str(self.k_p)
        string += "\nIntegral gain: " + str(self.k_i)
        string += "\nBound rate (rad/s): " + str(self.bound)
        return string


    def output(self, state, state_desired):
        # state = euler_angles in RAD + euler_angles_time_derivative in RAD/SEC
        # state_desired = psi_desired in RAD + psi_desired_time_derivative in RAD/SEC
        #return self.controller(state,state_desired)

        #--------------------------------------#
        # current phi and theta and psi
        euler_angles = state[0:3]
        phi          = euler_angles[0]
        theta        = euler_angles[1]
        psi          = euler_angles[2]

        euler_angles_time_derivative = state[3:6]
        
        phi_dot   = euler_angles_time_derivative[0]
        theta_dot = euler_angles_time_derivative[1]
        psi_dot   = euler_angles_time_derivative[2]

        #--------------------------------------#
        psi_star     = state_desired[0]
        psi_star_dot = state_desired[1]

        e_psi = np.sin(psi_star - psi)

        psi_dot = psi_star_dot + self.k_p * e_psi  + self.k_i * self.integrate

        t = rospy.get_time()

        self.integrate += ( e_psi - self.anti_wind_up ) * (t - self.t_old)

        #self.integrate = bound(self.integrate,self.bound_i,-self.bound_i)
        

        self.t_old = t
        #rospy.logerr("Psi_dot:%f Psi:%f Psi_star:%f"%(psi_dot, psi, psi_star))
        yaw_rate     = 1.0/np.cos(phi)*(np.cos(theta)*psi_dot - np.sin(phi)*theta_dot)
        
        bounded_yaw_rate = bound(yaw_rate,self.bound,-self.bound)

        #self.anti_wind_up = self.k_a * (yaw_rate - bounded_yaw_rate)

        #rospy.logwarn(self.integrate)


        return bounded_yaw_rate


# """Test"""
# 
#string = TrackingYawController.parameters_to_string()
#print string
#parameters = TrackingYawController.string_to_parameters(string)
#print parameters
#controller = TrackingYawController(parameters)
#print controller
#output = controller.output(np.zeros(6), np.ones(2))
#print output
