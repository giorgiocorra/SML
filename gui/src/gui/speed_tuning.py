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

# no need to get quad_control path, since it is package; import controllers dictionary
from src.simulators import simulators_dictionary


from quad_control.srv import PlannerStart
from quad_control.msg import quad_state
from quad_control.msg import quad_speed_cmd_3d
from nav_msgs.msg import Odometry
from converter_between_standards.rotorS_converter import RotorSConverter


import numpy

class SpeedTuningPlugin(Plugin):
    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = rospy.get_namespace()[1:]

        super(SpeedTuningPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SpeedTuningPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'speed_tuning.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SpeedTuningUi')
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


        self._widget.pvx.clicked.connect(lambda : self.speed_command(numpy.array([1,0,0])))
        self._widget.mvx.clicked.connect(lambda : self.speed_command(numpy.array([-1,0,0])))
        self._widget.pvy.clicked.connect(lambda : self.speed_command(numpy.array([0,1,0])))
        self._widget.mvy.clicked.connect(lambda : self.speed_command(numpy.array([0,-1,0])))
        self._widget.pvz.clicked.connect(lambda : self.speed_command(numpy.array([0,0,1])))
        self._widget.mvz.clicked.connect(lambda : self.speed_command(numpy.array([0,0,-1])))
        self._widget.v0.clicked.connect(lambda : self.speed_command(numpy.array([0,0,0])))
        self._widget.goto_init.clicked.connect(self.goto_initial_position)
        self._widget.stop.clicked.connect(lambda : self.stop())

        self._widget.b_speed_cont.clicked.connect(self.switch_to_speed_cont)

        self.current_position = numpy.array([0]*3)
        self.initial_position = None

        self.sub_state = rospy.Subscriber("/"+self.namespace+"quad_state", quad_state, self.update_position)

        speed_command_topic = "/"+self.namespace+"quad_speed_3d_test"

        self.speed_controller = rospy.Publisher(speed_command_topic, quad_speed_cmd_3d, queue_size=10)


    def speed_command(self,v):
        s = self._widget.speed.value()
        self.speed_controller.publish(quad_speed_cmd_3d(vx=s*v[0],vy=s*v[1],vz=s*v[2],wx=0.,wy=0.,wz=0.,time=0.))

    def goto_initial_position(self):
        offset = numpy.array([0,0,self._widget.z_value.value()])
        self.stop(self.initial_position + offset)

    def stop(self,position=numpy.array([0,0,1])):

        service_name = "/"+ self.namespace+'PlaceTheCamera'
        rospy.logerr("Call:" + service_name + " to " + str(position))
        rospy.wait_for_service(service_name,2.0)
        stop_srv = rospy.ServiceProxy(service_name, GotoPose,2.0)

        p = position.copy()

        stop_srv(x=p[0],y=p[1],z=p[2], theta = 0, psi = 0)

    def use_speed_controller(self,val):
        service_name = "/"+ self.namespace+'use_speed_controller'
        
        rospy.wait_for_service(service_name,2.0)
        use_speed_controller = rospy.ServiceProxy(service_name, SetBool,2.0)

        try:
            use_speed_controller(val)
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

    def slow_take_off(self):
        ref = self.current_position
        ref[2] += 0.5
        self.stop(ref)

    def update_position(self,position):
        self.current_position = numpy.array([position.x,position.y,position.z])

        if self.initial_position == None:
            self.initial_position = self.current_position[0:3]

    def get_state_from_rotorS_simulator(self,odometry_rotor_s):
        self.RotorSObject.rotor_s_attitude_for_control(odometry_rotor_s)
        state_quad = self.RotorSObject.get_quad_state(odometry_rotor_s)

        position = state_quad[0:3]
        self.current_position = state_quad[0:3]
        self.trace.append(numpy.array([position[0],position[1],position[2]]))


    def switch_to_speed_cont(self):
        service_name = "/" + self.namespace+'use_speed_controller'
        
        rospy.wait_for_service(service_name,2.0)
        use_speed_controller = rospy.ServiceProxy(service_name, SetBool,2.0)

        try:
            use_speed_controller(True)
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

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
