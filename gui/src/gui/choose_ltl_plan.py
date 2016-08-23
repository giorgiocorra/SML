import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal,Qt
import PyQt4.QtCore
import pyqtgraph as pg


import matplotlib.cm as cmx
import matplotlib.colors as colors
from matplotlib import pyplot as plt
import matplotlib.animation as animation

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar)
from matplotlib.backends import qt4_compat
from matplotlib.backend_bases import key_press_handler




import subprocess

from quad_control.srv import *
from std_srvs.srv import *

import argparse


import rospkg
import roslaunch

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
import sys
import os
sys.path.insert(0, rospack.get_path('quad_control'))

from utilities.utility_functions import unit_vec

# no need to get quad_control path, since it is package; import controllers dictionary
from src.simulators import simulators_dictionary


from quad_control.srv import Filename
from quad_control.msg import quad_state
from nav_msgs.msg import Odometry
from converter_between_standards.rotorS_converter import RotorSConverter

import numpy

from utilities.coverage_giorgio import read_lmks_from_file
from math import pi as PI

class ChooseLTLPlanPlugin(Plugin):
    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = rospy.get_namespace()[1:]

        self.name_others = rospy.get_param("/" + self.namespace + "name_others", '').rsplit(' ')


        super(ChooseLTLPlanPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseLTLPlanPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'choose_ltl_plan.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseLTLPlannerUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------------------- #


        self._widget.open_file.clicked.connect(self.open_file)
        self.filename = ""
        self._widget.load_lmks.clicked.connect(self.load_lmks)
        self._widget.update_initial_pose.clicked.connect(self.update_initial_pose)
        self._widget.stop.clicked.connect(self.stop)
        self._widget.start_planner.clicked.connect(self.start_speed_control)
        self._widget.goto_initial_position.clicked.connect(self.goto_initial_position)
        self._widget.slow_take_off.clicked.connect(self.slow_take_off)
        self._widget.sim_cube_3quads.clicked.connect(self.load_sim_cube_3quads)
        self._widget.sim_1quad_sim.clicked.connect(self.load_sim_1quad_sim)
        self._widget.sim_1quad_real.clicked.connect(self.load_sim_1quad_real)

        self._widget.savedata_3quads_check.stateChanged.connect(self.save_3quads)
        self._widget.savedata_1quad_sim_check.stateChanged.connect(self.save_1quad_sim)

        self.current_position = numpy.array([0.]*3)
        self.current_yaw = 0.
        self.initial_position = numpy.array([0.,0.,1.])
        self.initial_v_psi = 0
        self.initial_v_theta = 0


        # package = 'quad_control'
        # executable = 'planner_node.py'
        # args = ""

        # self.firefly = False

        # if self.namespace:
        #     args = "--namespace "+self.namespace
        # if self.firefly:
        #     args += "firefly"
        # node = roslaunch.core.Node(package, executable,args=args,output="screen", namespace=self.namespace)

        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()

        # self.process = launch.launch(node)

        # launch collision avoidance node if it is active

        # collision_av_active = rospy.get_param("collision_avoidance_active", True)

        # if (collision_av_active):        
        #     package = 'quad_control'
        #     executable = 'collision_avoidance_node.py'
        #     node = roslaunch.core.Node(package, executable, output = "screen", namespace=self.namespace)
        #     launcher = roslaunch.scriptapi.ROSLaunch()
        #     launcher.start()
        #     launcher.launch(node)

        # package = 'quad_control'
        # executable = 'magnitude_control_node.py'
        # node = roslaunch.core.Node(package, executable, output = "screen", namespace=self.namespace)
        # launcher = roslaunch.scriptapi.ROSLaunch()
        # launcher.start()
        # launcher.launch(node)

        self.trace = []
        self.canvas = None
        self.fig = None


        self.RotorSObject = RotorSConverter()
        
        rospy.Subscriber("quad_state", quad_state, self.update_trace)


    def open_file(self):
        dir_lmks = "/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/"
        result = QtGui.QFileDialog.getOpenFileName(self._widget, 'Open file', directory = dir_lmks)
        self.filename = result[0]
        self._widget.ltl_filename.setText(os.path.basename(self.filename))

    def load_lmks(self):
        rospy.logwarn("Filename:  " + self.filename)
        service_name = "/"+ self.namespace+'load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(self.filename)

    # def get_initial_position(self):
    #     init = self.plan.env.get_all_elem_in_region("i")
    #     return self.plan.env.get_baricenter(init[0])

    def update_initial_pose(self):
        px = self._widget.px.value()
        py = self._widget.py.value()
        pz = self._widget.pz.value()
        self.initial_position = numpy.array([px,py,pz])
        self.initial_v_psi = self._widget.vpsi.value() * PI
        self.initial_v_theta = self._widget.vtheta.value() * PI

    def goto_initial_position(self):
        service_name = "/"+ self.namespace+'PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = self.initial_position.copy()
        psi = self.initial_v_psi
        theta = self.initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)

    def stop(self):
        service_name = "/"+ self.namespace+'StopTheQuad'
        rospy.wait_for_service(service_name,2.0)
        stop_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        stop_srv()
        for ns in self.name_others:
            service_name = "/"+ ns+'/StopTheQuad'
            rospy.wait_for_service(service_name,2.0)
            stopsrv = rospy.ServiceProxy(service_name, Empty,2.0)
            stop_srv()

    def start_speed_control(self):
        service_name = "/"+ self.namespace+'start_speed_control'
        rospy.wait_for_service(service_name,2.0)
        start_speed_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        start_speed_srv()
        for ns in self.name_others:
            service_name = "/"+ ns+'/start_speed_control'
            rospy.wait_for_service(service_name,2.0)
            start_speed_srv = rospy.ServiceProxy(service_name, Empty,2.0)
            start_speed_srv()

    def slow_take_off(self):
        service_name = "/"+ self.namespace+'slow_take_off'
        rospy.wait_for_service(service_name,2.0)
        slow_take_off_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        slow_take_off_srv()

    def load_sim_cube_3quads(self):
        ####### Load the landmarks #######
        # Iris 1
        filename = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/cube_3quads_real_list_0.txt'
        service_name = '/Iris1/load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(filename)
        # Iris 2
        filename = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/cube_3quads_real_list_1.txt'
        service_name = '/Iris2/load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(filename)
        # Iris 3
        filename = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/cube_3quads_real_list_2.txt'
        service_name = '/Iris3/load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(filename)
        ####### Place the quads #######
        # Iris 1
        initial_position = numpy.array([-1.,-1.,0.75])
        initial_v_psi = 0. * PI
        initial_v_theta = 0. * PI
        service_name = '/Iris1/PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = initial_position.copy()
        psi = initial_v_psi
        theta = initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)
        # Iris 2
        initial_position = numpy.array([-1.,1.5,2.])
        initial_v_psi = 0. * PI
        initial_v_theta = 0. * PI
        service_name = '/Iris2/PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = initial_position.copy()
        psi = initial_v_psi
        theta = initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)
        # Iris 3
        initial_position = numpy.array([1.,1.5,0.75])
        initial_v_psi = 1 * PI
        initial_v_theta = 0. * PI
        service_name = '/Iris3/PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = initial_position.copy()
        psi = initial_v_psi
        theta = initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)

    def load_sim_1quad_sim(self):
        ####### Load the landmarks #######
        # Iris 1
        filename = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/collav_test.txt'
        service_name = '/load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(filename)
        ####### Place the quad #######
        # Iris 1
        initial_position = numpy.array([0.,1.5,1.5])
        initial_v_psi = -0.
        initial_v_theta = 0. * PI
        service_name = '/PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = initial_position.copy()
        psi = initial_v_psi
        theta = initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)

    def load_sim_1quad_real(self):
        ####### Load the landmarks #######
        # Iris 1
        filename = '/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/sim_1quad_real.txt'
        service_name = '/load_lmks_mission'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        load_lmks_srv(filename)
        ####### Place the quad #######
        # Iris 1
        initial_position = numpy.array([-1.2,-1.5,1.5])
        initial_v_psi = 0. * PI
        initial_v_theta = 0. * PI
        service_name = '/PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = initial_position.copy()
        psi = initial_v_psi
        theta = initial_v_theta
        place_srv(x=p[0],y=p[1],z=p[2],psi = psi, theta = theta)


    # def redraw(self,d):
    #     if self.trace and self.ax_trace:
    #         p = numpy.array(self.trace)
    #         self.ax_trace.set_data(p[:,0],p[:,1])
    #     return self.ax_trace,

    # def on_key_press(self, event):
    #     key_press_handler(event, self.canvas, self.mpl_toolbar)

    # def onclick(self,event):
    #     if event.dblclick:
    #         rospy.logwarn('goto %f %f' % (event.xdata, event.ydata))
    #         position = numpy.array([event.xdata, event.ydata])
    #         self.stop(position)

    def update_trace(self,position):
        self.current_position = numpy.array([position.x,position.y,position.z])

    def get_state_from_rotorS_simulator(self,odometry_rotor_s):
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)

        self.current_position = state_quad[0:3]
        self.current_yaw = state_quad[5]

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)
        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""
    
    def shutdown_plugin(self):
        self.process.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    def save_3quads(self):

        filename = self._widget.input_filename_3quads.text()
        
        if not filename:
            filename = 'temporary_file'

        t_0 = rospy.get_time()
        reset_service_1 = '/Iris1/reset_t0'
        reset_service_2 = '/Iris2/reset_t0'
        reset_service_3 = '/Iris3/reset_t0'
        save_service_1 = '/Iris1/SaveDataFromGui'
        save_service_2 = '/Iris2/SaveDataFromGui'
        save_service_3 = '/Iris3/SaveDataFromGui'
        # Iris1
        rospy.wait_for_service(reset_service_1,1.0)
        reset_srv = rospy.ServiceProxy(reset_service_1, Time)
        reset_srv(data = t_0)


        rospy.wait_for_service(save_service_1,1.0)
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(save_service_1,1.0)
            try:
                AskForSavingOrNot = rospy.ServiceProxy(save_service_1, SaveData)
                if self._widget.savedata_3quads_check.isChecked():
                    reply = AskForSavingOrNot(flag_save = True, file_name = filename)
                    if reply.Saving == True:
                        print('Iris1: saving')
                else:
                    reply = AskForSavingOrNot(flag_save = False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Iris1: Stopped saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
            pass
        # Iris2
        rospy.wait_for_service(reset_service_2,1.0)
        reset_srv = rospy.ServiceProxy(reset_service_2, Time)
        reset_srv(data = t_0)
        rospy.wait_for_service(save_service_2,1.0)
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(save_service_2,1.0)
            try:
                AskForSavingOrNot = rospy.ServiceProxy(save_service_2, SaveData)
                if self._widget.savedata_3quads_check.isChecked():
                    reply = AskForSavingOrNot(flag_save = True, file_name = filename)
                    if reply.Saving == True:
                        print('Iris2: saving')
                else:
                    reply = AskForSavingOrNot(flag_save = False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Iris2: Stopped saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
            pass
        # Iris3
        rospy.wait_for_service(reset_service_3,1.0)
        reset_srv = rospy.ServiceProxy(reset_service_3, Time)
        reset_srv(data = t_0)
        rospy.wait_for_service(save_service_3,1.0)
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(save_service_3,1.0)
            try:
                AskForSavingOrNot = rospy.ServiceProxy(save_service_3, SaveData)
                if self._widget.savedata_3quads_check.isChecked():
                    reply = AskForSavingOrNot(flag_save = True, file_name = filename)
                    if reply.Saving == True:
                        print('Iris3: saving')
                else:
                    reply = AskForSavingOrNot(flag_save = False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Iris3: Stopped saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
            pass

    def save_1quad_sim(self):

        filename = self._widget.input_filename_1quad_sim.text()
        
        if not filename:
            filename = 'temporary_file'

        if self._widget.savedata_1quad_sim_check.isChecked():
            t_0 = rospy.get_time()
            reset_service = '/reset_t0'
            rospy.wait_for_service(reset_service,1.0)
            reset_srv = rospy.ServiceProxy(reset_service, Time)
            reset_srv(data = t_0)

        save_service = '/SaveDataFromGui'
        rospy.wait_for_service(save_service,1.0)
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(save_service,1.0)
            try:
                AskForSavingOrNot = rospy.ServiceProxy(save_service, SaveData)
                if self._widget.savedata_1quad_sim_check.isChecked():
                    reply = AskForSavingOrNot(flag_save = True, file_name = filename)
                    if reply.Saving == True:
                        print('Saving')
                else:
                    reply = AskForSavingOrNot(flag_save = False)
                    if  reply.Saving == True:
                        # if controller receives message, we know it
                        print('Stopped saving')

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
        except: 
            print "Service not available ..."        
            pass

        