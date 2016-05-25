#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

database = {}

from fixed_point_trajectory.fixed_point_trajectory import FixedPointTrajectory
database['StayAtRest'] = FixedPointTrajectory

from circle_trajectory.circle_trajectory import CircleTrajectory
database['DescribeCircle'] = CircleTrajectory

from constant_xy_speed_trajectory.constant_xy_speed_trajectory import ConstantXYSpeedTrajectory
database['ConstantXYSpeedTrajectory'] = ConstantXYSpeedTrajectory


database["Default"] = database[rospy.get_param("TrajectoryDefault","StayAtRest")]