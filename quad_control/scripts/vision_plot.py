#!/usr/bin/env python
"""This module implements a ROS node that
plots the data relative to the coverage task."""


import matplotlib.pyplot as plt
import geometry_msgs.msg as gms
import quad_control.msg as qms
import std_msgs.msg as sms
import rospy as rp
import utilities.coverage_utilities as cov




def __pose_callback(vis):
    # come faccio a plottare in real time?



for name in cov.PLANNERS_NAMES:
    rp.Subscriber('vision', sms.Float64, __vision_callback)



