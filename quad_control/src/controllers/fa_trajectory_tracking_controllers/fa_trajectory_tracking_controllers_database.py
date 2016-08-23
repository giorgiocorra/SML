#TODO add an abstract fully-actuated controller to import

import rospy

database = {}

# from neutral_controller.neutral_controller import NeutralController
# database["NeutralController"] = NeutralController

from simple_pid_controller.simple_pid_controller import SimplePIDController
database["SimplePIDController"] = SimplePIDController

# from simple_pid_speed_controller.simple_pid_controller2 import SimplePIDSpeedController
# database["SimplePIDSpeedController"] = SimplePIDSpeedController

from simple_pid_speed_controller_3d.simple_pid_controller_3d import SimplePIDSpeedController
database["SimplePIDSpeedController_3d"] = SimplePIDSpeedController

# from simple_pid_speed_controller_3d.simple_pid_controller_3d_2 import SimplePIDSpeedController_2
# database["SimplePIDSpeedController_3d_2"] = SimplePIDSpeedController_2

# from abstract_pid_controller.abstract_pid_controller import ThreeDPIDController
# database["AbstractPIDController"] = ThreeDPIDController


database["Default"] = database[rospy.get_param("ControllerDefault","SimplePIDController")]