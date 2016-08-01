

class MyNode:
    def __init__(self):

        self.agents_namespaces = ["Iris1","Iris2"]
        self.place_service = rospy.Service('PlaceTheCamera', GotoPose, self.place_camera)
        
    
        self.pub_cam_pose = {}
        self.SubToSim = {}
        self.sub_speed_command = {}
        for ns in self.agents_namespaces:
            self.pub_cam_pose[ns] = rospy.Publisher(ns + '/camera_pose', camera_pose, queue_size = 10)

            self.SubToSim[ns] = rospy.Subscriber(ns + "/quad_state", quad_state, lambda data,ns=ns: self.get_state(data,ns) )

            self.sub_speed_command[ns] = rospy.Subscriber(ns + "/quad_speed_magnitude",quad_speed_cmd_3d, lambda data,ns=ns: self.set_command(data,ns) )


        

    def initialize_state(self):
        # state of quad: position, velocity, attitude, angular velocity 
        # PITCH, ROLL, AND YAW (EULER ANGLES IN DEGREES)
        self.state_quad = {}
        self.state = {}
        self.changed_state = {}
        self.long_rate = {}
        self.lat_rate = {}
        self.long = {}
        self.lat = {}
        
        for ns in self.agents_namespaces:
            self.state_quad[ns] = numpy.zeros(3+3+3+3)
            self.state[ns] = 'stop'
            self.changed_state[ns] = False
            self.long_rate[ns] = 0.
            self.lat_rate[ns] = 0.
            self.long[ns] = 0.
            self.lat[ns] = 0.
            self.pos_command[ns] = numpy.zeros(3)

    
    # callback when simulator publishes states
    def get_state(self,data,ns):
        # position
        p = numpy.array([data.x,data.y,data.z])
        # velocity
        p_dot = numpy.array([data.vx,data.vy,data.vz])
        #v = self.VelocityEstimator.out(p,rospy.get_time())
        # attitude: euler angles
        ee = numpy.array([data.roll,data.pitch,data.yaw])
        # euler angles derivative (not returned in simulator messages)
        ee_dot = numpy.zeros(3)
        # collect all components of state
        self.state_quad[ns] = numpy.concatenate([p,p_dot,ee,ee_dot])


        # Fake camera orientation
        t_sim = 1./100.         # the simulator node works at 100 hz
        self.long[ns] += self.long_rate[ns] * t_sim
        self.lat[ns] += self.lat_rate[ns] * t_sim

        #rospy.logerr('Position : ' + str(p) + ' / Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))

        v_camera = unit_vec_from_lat_long(self.long[ns],self.lat[ns])

        cam_pose = camera_pose()
        cam_pose.px = p[0]
        cam_pose.py = p[1]
        cam_pose.pz = p[2]
        cam_pose.vx = v_camera[0]
        cam_pose.vy = v_camera[1]
        cam_pose.vz = v_camera[2]
        cam_pose.time = data.time

        self.pub_cam_pose[ns].publish(cam_pose)

        if (self.state[ns]=='position') and (norm(self.pos_command[ns]-p)<1e-2):         # TODO: add condition on orientation of camera
            rospy.logerr('Initial position reached')
            self.stop_the_quad(ns)


    def set_command(self,data):
        if (self.state=='speed'):
            pos = self.state_quad[0:3]
            vel = [data.vx, data.vy, data.vz]
            acc = numpy.zeros(3)
            angles = self.state_quad[6:9]

            omega_xyz = [data.wx,data.wy,data.wz]
            
            psi = self.long
            self.long_rate = omega_xyz[2]
            self.lat_rate = sin(psi)*omega_xyz[0] - cos(psi)*omega_xyz[1] 
            
            omega = [0, 0, 0]          # useless, because the yaw controller is not working in simulation

            if (self.changed_state)and (any(vel)):
                self.changed_state = False

            #rospy.logwarn('Velocity: ' + str(vel) + ' Yaw rate: ' + str(self.long_rate) + ' Pitch rate: ' + str(self.lat_rate))
            if (not self.changed_state)and(norm(vel)<1e-2)and(norm(omega_xyz)<1e-2):
                rospy.logerr('Final position reached')
                self.stop_the_quad()
            else:
                self.command = numpy.concatenate([pos, vel, acc, angles, omega])

    def get_command(self):
        return self.command

    # Switch to the default controller and stop
    def stop_the_quad(self,data = None):

        self.state = 'stop'
        self.changed_state = False              # just in case it wasn't already reset
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
        self.command[3:6] = numpy.array([0.,0.,0.])
        self.command[6:9] = numpy.array([0.,0.,0.])
        self.command[9:12] = numpy.array([0.,0.,self.state_quad[8]])
        self.command[12:15] = numpy.array([0.,0.,0.])
        rospy.logerr("Stopping the quad: ")
        rospy.logwarn('Position: ' + str(self.command[0:3]))
        rospy.logwarn('Yaw: ' + str(self.long) + ' / Pitch: ' + str(self.lat))
        return {}

    # callback for service for initial positioning of camera
    def place_camera(self,data = None):

        self.changed_state = False              # just in case it wasn't already reset
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
        self.command[6:9] = numpy.array([0.,0.,0.])
        self.command[9:12] = numpy.array([0.,0.,data.psi])
        self.command[12:15] = numpy.array([0.,0.,0.])
        self.long = data.psi
        self.lat = data.theta
        self.long_rate = 0.
        self.lat_rate = 0.
        rospy.logerr('Placing the camera:')
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

        self.state = 'speed'
        self.changed_state = True               # to avoid stopping because of the delay in the answer of the planner
        self.start_planner()

        if self.ControllerObject.__class__ != self.inner['controller']["SimplePIDSpeedController_3d"]:
            rospy.logwarn("Switch to velocity controller")
            ControllerClass      = self.inner['controller']["SimplePIDSpeedController_3d"]
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

    def take_off(self, data = None):
        pose = GotoPose()
        pose.x = self.state_quad[0]
        pose.y = self.state_quad[1]
        pose.z = self.state_quad[2] + 0.5
        pose.psi = self.long
        pose.theta = self.lat
        self.place_camera(pose)

