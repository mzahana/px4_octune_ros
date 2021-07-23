#!/usr/bin/env python
"""
BSD 3-Clause License
Copyright (c) 2020, Mohamed Abdelkader Zahana
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 """

from pandas.core.resample import resample
import rospy
import numpy as np
import tf

from math import pi, sqrt, atan2
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse, Trigger, TriggerResponse
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget, State, Altitude, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandTOLResponse
from psu_delivery_drone_control.srv import PIDGains,PIDGainsRequest, PIDGainsResponse, Takeoff, TakeoffResponse, MaxVel, MaxVelResponse

#
# TODO Implement a yaw controller that respects a desired yaw rate
#

class FCUModes:
    def __init__(self):
	    pass    

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setHoldMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LOITER')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Auto Hold Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e
    
    def setAutoTakeoffMode(self, latitude=0.0, longitude=0.0, amsl_alt=10.0):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffModeService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
            takeoffModeService(latitude=latitude, longitude=longitude, altitude = amsl_alt)
        except rospy.ServiceException, e:
            print "service mavros/cmd/takeoff call failed: %s. AutoTakeoff could not be set."%e
    
##########################################################################################
"""
This is a PI controller which takes target positions (ex_, ey_, ez_) in body directions (i.e. relative to body).
It ouputs velocity commands (body_vx_cmd_, body_vx_cmd_, body_vz_cmd_) in body directions.
"""
class PositionController:
    def __init__(self):
        # Error in body x direction (+x is drone's right)
        self.ex_ = 0.0
        # Error in body y direction (+y is  drone's front)
        self.ey_ = 0.0
        # Error in body z direction (+z is  up)
        self.ez_ = 0.0
        # Error integral in x
        self.ex_int_ = 0.0
        # Error integral in y
        self.ey_int_ = 0.0
        # Error integral in z
        self.ez_int_ = 0.0

        # Proportional gain for horizontal controller
        self.kP_xy_ = rospy.get_param('~horizontal_controller/kP', 1.5)
        # Integral gain for horizontal controller
        self.kI_xy_ = rospy.get_param('~horizontal_controller/kI', 0.01)
        # Integral gain for vertical controller
        self.kP_z_ = rospy.get_param('~vertical_controller/kP', 2.0)
        # Integral gain for vertical controller
        self.kI_z_ = rospy.get_param('~vertical_controller/kI', 0.01)

        # Controller outputs. Velocity commands in body sudo-frame
        self.body_vx_cmd_ = 0.0
        self.body_vy_cmd_ = 0.0
        self.body_vz_cmd_ = 0.0

        # Controller outputs in local frame
        self.local_vx_cmd_ = 0.0
        self.local_vy_cmd_ = 0.0
        self.local_vz_cmd_ = 0.0
        
        # Maximum horizontal velocity (m/s)
        self.vXYMAX_ = rospy.get_param('~horizontal_controller/vMAX', 1.0)
        # Maximum upward velocity (m/s)
        self.vUpMAX_ = rospy.get_param('~vertical_controller/vUpMAX', 1.0)
        # Maximum downward velocity (m/s)
        self.vDownMAX_ = rospy.get_param('~vertical_controller/vDownMAX', 0.5)


        # Compute yaw towards next setpoint, or lock it
        self.lock_heading_=rospy.get_param('~lock_heading', False)
        # Yaw error, rad
        self.eyaw_ = 0.0
        # Yaw rate setpoint
        self.yawrate_cmd_=0.0
        # Yaw setpoint, radian
        self.yaw_cmd_=0.0

        # Yaw proportional gain
        self.kP_yaw_=rospy.get_param('~yaw_controller/kP', 1.0)
        # Yaw radius  (meters) before disabling yaw control
        self.yawRadius_ = rospy.get_param('~yaw_controller/yaw_radius', 1.0)
        # Maximum yaw rate, rad/s
        self.maxYawRate_=rospy.get_param('~yaw_controller/max_yaw_rate', 50.0*pi/180.0)
        # Send yaw vs/ yaw rate setpoints
        self.use_yaw_rate_=rospy.get_param('~use_yaw_rate', False)

        # Flag for FCU state. True if vehicle is armed and ready
        # Prevents building controller integral part when vehicle is idle on ground
        self.engaged_ = False

        # Is drone armed
        self.isArmed_ = False

        # Service for modifying horizontal PI controller gains 
        rospy.Service('offboard_controller/horizontal/pid_gains', PIDGains, self.setHorizontalPIDCallback)
        # Service for modifying vertical PI controller gains 
        rospy.Service('offboard_controller/vertical/pid_gains', PIDGains, self.setVerticalPIDCallback)
        # Service for modifying yaw P controller gain
        rospy.Service('offboard_controller/yaw/pid_gains', PIDGains, self.setYawPIDCallback)
        # Service for modifying velocity contraints
        rospy.Service('offboard_controller/max_vel', MaxVel, self.setMaxVelCb)
        # Lock heading
        rospy.Service('offboard_controller/lock_heading', SetBool, self.lockHeadingCb)

    def lockHeadingCb(self, req):
        self.lock_heading_=req.data

        resp=SetBoolResponse()
        resp.success=True
        if (req.data):
            resp.message="Heading is locked"
        else:
            resp.message="Heading is not locked"
        
        return resp

    def setMaxVelCb(self, req):
        resp=MaxVelResponse()

        if (req.xy_vel>0):
            self.vXYMAX_=req.xy_vel
        if (req.up_vel>0):
            self.vUpMAX_=req.up_vel
        if (req.down_vel>0):
            self.vDownMAX_=req.down_vel
        

        rospy.loginfo("[OffboarController::setMaxVel] Velocity constraints are updated. XY_VEL=%s, UP_VEL=%s, DOWN_VEL=%s", self.vXYMAX_, self.vUpMAX_, self.vDownMAX_)
        resp.success=True
        resp.message="Velocity constraints are updated. XY_VEL={}, UP_VEL={}, DOWN_VEL={}".format(self.vXYMAX_, self.vUpMAX_, self.vDownMAX_)

        return resp

    def setHorizontalPIDCallback(self, req):
        resp=PIDGainsResponse()

        if (req.p>=0.):
            self.kP_xy_ = req.p
        if (req.i >=0.):
            self.kI_xy_ = req.i

        rospy.loginfo("Horizontal controller gains are set to P=%s I=%s", self.kP_xy_, self.kI_xy_)
        resp.success=True
        resp.message="Horizontal controller gains are set to P={}, I={}".format(self.kP_xy_, self.kI_xy_)

        return resp

    def setVerticalPIDCallback(self, req):
        resp=PIDGainsResponse()

        if (req.p>=0.):
            self.kP_z_ = req.p
        if (req.i>=0.0):
            self.kI_z_ = req.i

        rospy.loginfo("Vertical controller gains are set to P=%s I=%s", self.kP_z_, self.kI_z_)
        resp.success=True
        resp.message="Vertical controller gains are set to P={}, I={}".format(self.kP_z_, self.kI_z_)

        return resp

    def setYawPIDCallback(self, req):
        resp=PIDGainsResponse()
        if req.p < 0. or req.i < 0.0 or req.d < 0:
            resp.success=False
            resp.message="Can not set negative PID gains"
            rospy.logerr("Can not set negative PID gains.")
            return resp

        self.kP_yaw_ = req.p

        rospy.loginfo("Yaw controller P gain are set to P=%s", self.kP_yaw_)
        resp.success=True
        resp.message="Yaw controller gains are set to P={}, I={}".format(req.p, req.i)

        return resp

    def resetIntegrators(self):
        self.ex_int_ = 0.
        self.ey_int_ = 0.
        self.ez_int_ = 0.

    def computeXYVelSetpoint(self):
        """
        Computes XY velocity setpoint in body sudo-frame using a PI controller
        """
        # Compute commands
        self.body_vx_cmd_ = self.kP_xy_*self.ex_ + self.kI_xy_*self.ex_int_
        self.body_vy_cmd_ = self.kP_xy_*self.ey_ + self.kI_xy_*self.ey_int_
        self.body_vz_cmd_ = self.kP_z_*self.ez_ + self.kI_z_*self.ez_int_

        # Horizontal velocity constraints
        vel_magnitude = sqrt(self.body_vx_cmd_**2 + self.body_vy_cmd_**2)
        if vel_magnitude > self.vXYMAX_ : # anti-windup scaling      
            scale = self.vXYMAX_/vel_magnitude
            self.body_vx_cmd_ = self.body_vx_cmd_ * scale
            self.body_vy_cmd_ = self.body_vy_cmd_ * scale
        else:
            if self.engaged_: # if armed & offboard
                self.ex_int_ = self.ex_int_ + self.ex_
                self.ey_int_ = self.ey_int_ + self.ey_

        return self.body_vx_cmd_, self.body_vy_cmd_

    def computeZVelSetpoint(self):
        """
        Computes Z velocity setpoint in body sudo-frame using a PI controller
        """
        # Vertical velocity constraints
        if self.body_vz_cmd_ > self.vUpMAX_ : # anti-windup scaling      
            self.body_vz_cmd_ = self.vUpMAX_
        elif self.body_vz_cmd_ < -self.vDownMAX_:
            self.body_vz_cmd_ = -self.vDownMAX_
        else:
            if self.engaged_: # if armed & offboard
                self.ez_int_ = self.ez_int_ + self.ez_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_z_ for now!

        return self.body_vz_cmd_

    def computeVelSetpoint(self):
        """
        Computes XYZ velocity setpoint in body sudo-frame using a PI controller
        """
        # Compute commands
        self.body_vx_cmd_ = self.kP_xy_*self.ex_ + self.kI_xy_*self.ex_int_
        self.body_vy_cmd_ = self.kP_xy_*self.ey_ + self.kI_xy_*self.ey_int_
        self.body_vz_cmd_ = self.kP_z_*self.ez_ + self.kI_z_*self.ez_int_

        # Horizontal velocity constraints
        vel_magnitude = sqrt(self.body_vx_cmd_**2 + self.body_vy_cmd_**2)
        if vel_magnitude > self.vXYMAX_ : # anti-windup scaling      
            scale = self.vXYMAX_/vel_magnitude
            self.body_vx_cmd_ = self.body_vx_cmd_ * scale
            self.body_vy_cmd_ = self.body_vy_cmd_ * scale
        else:
            if self.engaged_: # if armed & offboard
                self.ex_int_ = self.ex_int_ + self.ex_
                self.ey_int_ = self.ey_int_ + self.ey_
        
        # Vertical velocity constraints
        if self.body_vz_cmd_ > self.vUpMAX_ : # anti-windup scaling      
            self.body_vz_cmd_ = self.vUpMAX_
        elif self.body_vz_cmd_ < -self.vDownMAX_:
            self.body_vz_cmd_ = -self.vDownMAX_
        else:
            if self.engaged_: # if armed & offboard
                self.ez_int_ = self.ez_int_ + self.ez_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_z_ for now!

        return self.body_vx_cmd_, self.body_vy_cmd_, self.body_vz_cmd_

    def computeYawRateSp(self):
        """Computes the yaw rate setpoint based on the yaw error between target and current drone positions
        """
        if (not self.lock_heading_):
            yaw_err = atan2(self.ey_,self.ex_)
            radius = sqrt(self.ex_**2 + self.ey_**2)
            if radius < self.yawRadius_:
                self.yawrate_cmd_ = 0.0
            else:
                self.yawrate_cmd_ = self.kP_yaw_*yaw_err    # proportional yaw controller
            
            # Cap yaw rate
            if self.yawrate_cmd_ > self.maxYawRate_:
                self.yawrate_cmd_ = np.sign(self.yawrate_cmd_) * self.maxYawRate_

        return self.yawrate_cmd_

    def computeYawSp(self):
        """Computes the required yaw setpoint to point towards the next target position
        """
        if (not self.lock_heading_):
            radius = sqrt(self.ex_**2 + self.ey_**2)
            if radius > self.yawRadius_:
                self.yaw_cmd_ = atan2(self.ey_,self.ex_)

        return self.yaw_cmd_



##########################################################################################

class Commander:
    def __init__(self):
        # Instantiate a setpoint topic structure
        self.setpoint_ = PositionTarget()

        # use velocity and yaw setpoints
        self.setBodyVelMask()

        # Velocity setpoint by user
        self.vel_setpoint_ = Vector3()

        # Position setpoint by user
        self.pos_setpoint_ = Point()

        # Current local velocity
        self.local_vel_ = TwistStamped()

        # Current body velocity
        self.body_vel_ = TwistStamped()

        # Yaw setpoint by user (degrees); will be converted to radians before it's published
        self.yaw_setpoint_ = 0.0

        # Yaw rate setpoint, rad/s
        self.yawrate_setpoint_ = 0.0

        # Current drone position (local frame)
        self.drone_pos_ = Point()

        # Current drone's GPS position
        self.drone_global_pos_ = NavSatFix()

        # Current drone mavros/altitude
        self.drone_mavros_alt_ = Altitude()

        # Is drone in the air?
        self.isInAir_ = False

        # Is armed?
        self.isArmed_=False
        self.isOFFBOARD_=False

        # FCU modes
        self.fcu_mode_ = FCUModes()

        #------------------------------------
        # setpoints in local frame
        self.local_xSp_  = 0.0
        self.local_ySp_  = 0.0
        self.local_zSp_  = 0.0

        # Relative setpoints (i.e. with respect to body horizontal-frame)
        self.relative_xSp_  = 0.0
        self.relative_ySp_  = 0.0
        self.relative_zSp_  = 0.0

        # Flag to select between local vs. relative tracking
        # set False for relative target tracking
        self.local_tracking_ = True

        self.bypass_controller_ = False

        # Controller object to calculate velocity commands
        self.controller_ = PositionController()

        # Subscriber for user setpoints (local position)
        rospy.Subscriber('offboard_controller/setpoint/local_pos', Point, self.localPosSpCallback)

        # Subscriber for user setpoints (relative position)
        #rospy.Subscriber('setpoint/relative_pos', Point, self.relativePosSpCallback)

        # Publisher for velocity errors in body frame
        self.bodyVel_err_pub_ = rospy.Publisher('offboard_controller/analysis/body_vel_err', PointStamped, queue_size=10)

        # Publisher for velocity errors in local frame
        self.localVel_err_pub_ = rospy.Publisher('offboard_controller/analysis/local_vel_err', PointStamped, queue_size=10)

        # Publisher for position errors in local frame
        self.localPos_err_pub_ = rospy.Publisher('offboard_controller/analysis/local_pos_err', PointStamped, queue_size=10)

        # Publisher for position error between drone and target
        self.relativePos_err_pub_ = rospy.Publisher('offboard_controller/analysis/relative_pos_err', PointStamped, queue_size=10)
        #------------------------------------

        # setpoint publisher (velocity to Pixhawk)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Subscriber for user setpoints (body velocity)
        #rospy.Subscriber('setpoint/body_vel', Vector3, self.velSpCallback)
        rospy.Subscriber('offboard_controller/setpoint/body_xy_vel', Vector3, self.velSpCallback)
        rospy.Subscriber('offboard_controller/setpoint/local_xy_vel', Vector3, self.localVelSpCallback)

        # Subscriber for user setpoints (local position)
        #rospy.Subscriber('setpoint/local_pos', Point, self.posSpCallback)

        # Subscriber for user setpoints (yaw in degrees)
        rospy.Subscriber('offboard_controller/setpoint/yaw_deg', Float32, self.yawSpCallback)

        # Subscriber to current drone's local position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        # Subscriber to current drone's GPS position
        rospy.Subscriber('mavros/global_position/global', NavSatFix, self.droneGPSCallback)

        # Subscriber to mavros/altitude
        rospy.Subscriber('mavros/altitude', Altitude, self.mavrosAltCallback)

        # Subscriber to body velocity
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.bodyVelCallback)

        # Subscriber to local velocity
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.localVelCallback)

        # Subscribe to drone FCU state
        rospy.Subscriber('mavros/state', State, self.cbFCUstate)

        # Mavros Extended state, to get landing state
        rospy.Subscriber('mavros/extended_state', ExtendedState, self.mavrosExtStateCallback)

        # Service for arming and setting OFFBOARD flight mode
        rospy.Service('offboard_controller/arm_and_offboard', Empty, self.armAndOffboard)

        # Service for autoland
        rospy.Service('offboard_controller/auto_land', Empty, self.autoLand)

        # Service for auto Takeoff
        rospy.Service('offboard_controller/auto_takeoff', Takeoff, self.autoTakeoffSrv)

        # Service for Hold flight mode
        rospy.Service('offboard_controller/auto_hold', Empty, self.autoHoldSrv)

        # Service to bypass position controller, and send velocity setpoints directly
        # TODO Maybe not needed ; you can just use self.velSpCallback()
        rospy.Service('offboard_controller/bypass_controller', SetBool, self.bypassCntSrv)

        # sets the current drone local position as local setpoint
        rospy.Service('offboard_controller/use_current_pos', Trigger, self.useCurrentPosSrv)

        # Activate local tracking
        rospy.Service('offboard_controller/set_local_tracking', Trigger, self.setLocalTrackingSrv)


    def setLocalTrackingSrv(self, req):
        self.local_tracking_=True
        self.bypass_controller_=False
        self.controller_.resetIntegrators()
        resp=TriggerResponse()
        resp.success=True
        resp.message="Local tracking is set to {}".format(self.local_tracking_)

        return resp

    def useCurrentPosSrv(self, req):
        self.local_xSp_=self.drone_pos_.x
        self.local_ySp_=self.drone_pos_.y
        self.local_zSp_=self.drone_pos_.z
        
        resp=TriggerResponse()
        resp.success=True
        resp.message="Current local setpoint is set to current position x={}, y={}, z={}".format(self.local_xSp_, self.local_ySp_, self.local_zSp_)

        return resp

    def bypassCntSrv(self, req):
        self.bypass_controller_=req.data

        resp=SetBoolResponse()
        resp.success=True
        resp.message="bypass_controller_ = {}".format(self.bypass_controller_)
        
        return resp

    def bodyVelCallback(self, msg):
        self.body_vel_ = msg

    def localVelCallback(self, msg):
        self.local_vel_ = msg

    def autoLand(self, req):
        self.fcu_mode_.setAutoLandMode()

        return EmptyResponse()

    def autoHoldSrv(self, req):
        self.fcu_mode_.setHoldMode()
        self.local_xSp_=self.drone_pos_.x
        self.local_ySp_=self.drone_pos_.y
        self.local_zSp_=self.drone_pos_.z
        self.local_tracking_=True
        self.bypass_controller_=False
        rospy.loginfo("[Offboard controller] Setting Auto Hold flight mode")

    def autoTakeoffSrv(self, req):
        if self.isInAir_:
            rospy.logwarn("[Offboard controller] Drone is already in the air. Ignoring Takeoff command.")
            return []
        if not self.isArmed_:
            self.fcu_mode_.setArm()

        self.local_xSp_=self.drone_pos_.x
        self.local_ySp_=self.drone_pos_.y
        self.local_zSp_=req.altitude
        self.fcu_mode_.setAutoTakeoffMode(latitude=self.drone_global_pos_.latitude, longitude=self.drone_global_pos_.longitude, amsl_alt=self.drone_mavros_alt_.amsl + req.altitude)
        rospy.loginfo("[Offboard controller] Takeoff command is sent")
        return []

    def armAndOffboard(self, req):
        if not self.isArmed_:
            self.fcu_mode_.setArm()
        if not self.isOFFBOARD_:
            self.fcu_mode_.setOffboardMode()
        if self.isArmed_ and self.isOFFBOARD_:
            rospy.loginfo("[Offboard controller] Drone is Armed and OFFBOARD mode is set")
        
        return EmptyResponse()
    
    def cbFCUstate(self, msg):
        if msg is None:
            return

        self.isArmed_ = msg.armed
        if msg.mode == 'OFFBOARD':
            self.isOFFBOARD_=True
        else:
            self.isOFFBOARD_=False

    def mavrosExtStateCallback(self, msg):
        if msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self.isInAir_ = True
        else:
            self.isInAir_ = False

    def dronePosCallback(self, msg):
        self.drone_pos_.x = msg.pose.position.x
        self.drone_pos_.y = msg.pose.position.y
        self.drone_pos_.z = msg.pose.position.z

    def droneGPSCallback(self, msg):
        if msg is None:
            return

        self.drone_global_pos_ = msg

    def mavrosAltCallback(self, msg):
        if msg is None:
            return

        self.drone_mavros_alt_ = msg

    def velSpCallback(self, msg):
        """
        Body velocity setpoint callback
        """
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y
        #self.vel_setpoint_.z = msg.z
        self.local_zSp_=msg.z

        self.setBodyVelMask()
        self.bypass_controller_=True
        self.local_tracking_=False

    def localVelSpCallback(self, msg):
        """
        Local velocity setpoint callback
        """
        if not self.isInAir_:
            return
            
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y
        #self.vel_setpoint_.z = msg.z
        self.local_zSp_=msg.z

        self.setLocalVelMask()
        self.bypass_controller_=True
        self.local_tracking_=False
    
    def posSpCallback(self, msg):
        """
        Position setpoint callback
        """
        self.pos_setpoint_.x = msg.x
        self.pos_setpoint_.y = msg.y
        self.pos_setpoint_.z = msg.z

        self.setLocalPositionMask()

    def yawSpCallback(self, msg):
        """
        Yaw setpoint callback
        """
        self.yaw_setpoint_ = msg.data*pi/180.0


    def setLocalPositionMask(self):
        """
        Sets type_mask for position setpoint in local frame +  yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE
        

    def setBodyVelMask(self):
        """
        Sets type_mask for velocity setpoint in body frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setLocalVelMask(self):
        """
        Sets type_mask for velocity setpoint in local frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def publishSetpoint(self):
        self.setpoint_.header.stamp = rospy.Time.now()

        # Only one type of the following setpoints will be consumed based on the type_mask
        self.setpoint_.position.x = self.pos_setpoint_.x
        self.setpoint_.position.y = self.pos_setpoint_.y
        self.setpoint_.position.z = self.pos_setpoint_.z

        self.setpoint_.velocity.x = self.vel_setpoint_.x
        self.setpoint_.velocity.y = self.vel_setpoint_.y
        self.setpoint_.velocity.z = self.vel_setpoint_.z

        self.setpoint_.yaw = self.yaw_setpoint_ 
        self.setpoint_.yaw_rate = self.yawrate_setpoint_

        self.setpoint_pub_.publish(self.setpoint_)

    #--------------------------------------------------
    def computeControlOutput(self):
        if not self.bypass_controller_:
            if self.local_tracking_:
                self.controller_.ex_ = self.local_xSp_ - self.drone_pos_.x
                self.controller_.ey_ = self.local_ySp_ - self.drone_pos_.y
                self.controller_.ez_ = self.local_zSp_ - self.drone_pos_.z
                self.setLocalVelMask()
            else: # relative tracking
                self.controller_.ex_ = self.relative_xSp_
                self.controller_.ey_ = self.relative_ySp_
                self.controller_.ez_ = self.relative_zSp_
                self.setBodyVelMask()

            self.controller_.engaged_= self.isOFFBOARD_ and self.isOFFBOARD_
            
            
            self.vel_setpoint_.x, self.vel_setpoint_.y, self.vel_setpoint_.z = self.controller_.computeVelSetpoint()
            #self.commander_.yawrate_setpoint_ = self.controller_.controlYaw()
            self.yaw_setpoint_ = self.controller_.computeYawSp()

        else: # Don't control XY, just control the altitude (to avoid loosing altitude), and bypass xy velocities (by user)
            self.controller_.ez_ = self.local_zSp_ - self.drone_pos_.z
            zsp=self.controller_.computeZVelSetpoint()
            self.vel_setpoint_.z = zsp



    def localPosSpCallback(self, msg):
        if not self.controller_.engaged_:
            rospy.logwarn_throttle(1.0, "[Offboard controller] Drone is either not armed OR not in OFFBOARD mode !!")

        self.local_xSp_ = msg.x
        self.local_ySp_ = msg.y
        self.local_zSp_ = msg.z

        # In case we are switching from relative to local tracking
        # to avoid jumps caused by accumulation in the integrators
        if not self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = True

        # Don't bypass the controller (don't accept direct velocity setpoint that are already set)
        self.bypass_controller_=False

    def relativePosSpCallback(self, msg):
        self.relative_xSp_ = msg.x
        self.relative_ySp_ = msg.y
        self.relative_zSp_ = msg.z

        # In case we are switching from local to relative tracking
        # to avoid jumps caused by accumulation in the integrators
        if self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = False

    def publishErrorSignals(self):
        """
        Publishes all error signals for debugging and tuning
        """

        # Local velocity errors
        localVelErr_msg = PointStamped()
        localVelErr_msg.header.stamp = rospy.Time.now()
        localVelErr_msg.point.x = self.setpoint_.velocity.x - self.local_vel_.twist.linear.x
        localVelErr_msg.point.y = self.setpoint_.velocity.y - self.local_vel_.twist.linear.y
        localVelErr_msg.point.z = self.setpoint_.velocity.z - self.local_vel_.twist.linear.z
        
        self.localVel_err_pub_.publish(localVelErr_msg)

        # Body velocity errors
        # this message uses convention of +x-right, +y-forward, +z-up
        # the setpoint msg follows the same convention
        # However, the feedback signal (actual body vel from mavros) follows +x-forward, +y-left, +z-up
        # Required conversion is done below
        bodyVelErr_msg = PointStamped()
        bodyVelErr_msg.header.stamp = rospy.Time.now()
        bodyVelErr_msg.point.x = self.setpoint_.velocity.x - (-self.body_vel_.twist.linear.y)
        bodyVelErr_msg.point.y = self.setpoint_.velocity.y - self.body_vel_.twist.linear.x
        bodyVelErr_msg.point.z = self.setpoint_.velocity.z - self.body_vel_.twist.linear.z

        self.bodyVel_err_pub_.publish(bodyVelErr_msg)

        # Local position errors
        localPosErr_msg = PointStamped()
        localPosErr_msg.header.stamp = rospy.Time.now()
        localPosErr_msg.point.x = self.local_xSp_ - self.drone_pos_.x
        localPosErr_msg.point.y = self.local_ySp_ - self.drone_pos_.y
        localPosErr_msg.point.z = self.local_zSp_ - self.drone_pos_.z

        self.localPos_err_pub_.publish(localPosErr_msg)

        # Relative position errors
        relPosErr_msg = PointStamped()
        relPosErr_msg.header.stamp = rospy.Time.now()
        relPosErr_msg.point.x = self.relative_xSp_
        relPosErr_msg.point.y = self.relative_ySp_
        relPosErr_msg.point.z = self.relative_zSp_

        self.relativePos_err_pub_.publish(relPosErr_msg)



if __name__ == '__main__':
    rospy.init_node('Offboard_control_node', anonymous=True)
    
    # tracker = Tracker()
    cmd = Commander()

    loop = rospy.Rate(20)

    while not rospy.is_shutdown():

        """
        # Example of how to use the PositionController
        K = PositionController() # Should be created outside ROS while loop
        
        # The following should be inside ROS while loop
        # update errors
        K.ex_ = relative_position_in_x # body directions
        K.ey_ = relative_position_in_y
        K.ez_ = relative_position_in_z
        cmd.vel_setpoint_.x, cmd.vel_setpoint_.y, cmd.vel_setpoint_.z = K.computeVelSetpoint()
        cmd.setBodyVelMask()
        # Then, publish command as below (publishSetpoint)
        """
        cmd.computeControlOutput()
        cmd.publishSetpoint()
        cmd.publishErrorSignals()
        #cmd.publishSetpoint()
        loop.sleep()