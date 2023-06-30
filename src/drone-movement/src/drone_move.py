#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *

class Drone():
    def __init__(self):
        self.position_subscriber = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.globalPositionCallback)
        self.latitude = 0
        self.longitude = 0

    def globalPositionCallback(self, globalPositionCallback):
        self.latitude = globalPositionCallback.latitude
        self.longitude = globalPositionCallback.longitude
        rospy.loginfo("longitude: %.7f" %self.longitude)
        rospy.loginfo("latitude: %.7f" %self.latitude)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def takeoff(self, alt):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
            takeoffService(altitude = alt, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)

    def land(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException as e:
            print("service land call failed: %s. The vehicle cannot land "%e)

    def setMode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode=mode) #'GUIDED', 'STABALIZE'
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s."%e)

if __name__ == '__main__':
    rospy.init_node('drone_move')
    drone = Drone()
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()