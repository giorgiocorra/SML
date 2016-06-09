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
        self.namespace = self._parse_args(context.argv())


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

        self.current_position = numpy.array([0.]*3)
        self.current_yaw = 0.
        self.initial_position = numpy.array([0.,0.,1.])
        self.initial_orientation_vector = numpy.array([0.,-1.,0.])


        package = 'quad_control'
        executable = 'planner_node.py'
        args = ""

        self.firefly = False

        if self.namespace:
            args = "--namespace "+self.namespace
        if self.firefly:
            args += "firefly"
        node = roslaunch.core.Node(package, executable,args=args,output="screen")

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.process = launch.launch(node)

        self.trace = []
        self.canvas = None
        self.fig = None


        self.RotorSObject = RotorSConverter()
        if self.firefly:
            self.sub_odometry = rospy.Subscriber(
                "/firefly/ground_truth/odometry",
                Odometry,
                self.get_state_from_rotorS_simulator
                )
        else:
            rospy.Subscriber("quad_state", quad_state, self.update_trace)


    def open_file(self):
        dir_lmks = "/home/giorgiocorra/sml_ws/src/quad_control/experimental_data/landmark_examples/"
        result = QtGui.QFileDialog.getOpenFileName(self._widget, 'Open file', directory = dir_lmks)
        self.filename = result[0]
        self._widget.ltl_filename.setText(os.path.basename(self.filename))

    def load_lmks(self):
        service_name = "/"+ self.namespace+'load_lmks'
        rospy.wait_for_service(service_name,2.0)
        load_lmks_srv = rospy.ServiceProxy(service_name, Filename,2.0)
        rospy.logwarn("Filename:  " + self.filename)
        load_lmks_srv(self.filename)

    # def get_initial_position(self):
    #     init = self.plan.env.get_all_elem_in_region("i")
    #     return self.plan.env.get_baricenter(init[0])

    def update_initial_pose(self):
        px = self._widget.px.value()
        py = self._widget.py.value()
        pz = self._widget.pz.value()
        vpsi = self._widget.vpsi.value() * PI
        vtheta = self._widget.vtheta.value() * PI
        self.initial_position = numpy.array([px,py,pz])
        self.initial_orientation_vector = unit_vec(vpsi,vtheta)

    def goto_initial_position(self):
        service_name = "/"+ self.namespace+'PlaceTheCamera'
        rospy.wait_for_service(service_name,2.0)
        place_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)
        p = self.initial_position.copy()
        v = self.initial_orientation_vector.copy()
        place_srv(x=p[0],y=p[1],z=p[2],v0=v[0],v1=v[1],v2=v[2])

    def stop(self):
        service_name = "/"+ self.namespace+'StopTheQuad'
        rospy.wait_for_service(service_name,2.0)
        stop_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        stop_srv()

    def start_speed_control(self):
        service_name = "/"+ self.namespace+'start_speed_control'
        rospy.wait_for_service(service_name,2.0)
        start_speed_srv = rospy.ServiceProxy(service_name, Empty,2.0)
        start_speed_srv()

    def slow_take_off(self):
        ref = self.current_position
        ref[2] += 0.5
        #self.stop()

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
        self.trace.append(self.current_position)

    def get_state_from_rotorS_simulator(self,odometry_rotor_s):
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)

        self.current_position = state_quad[0:3]
        self.current_yaw = state_quad[5]
        self.trace.append(numpy.array(self.current_position))


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
