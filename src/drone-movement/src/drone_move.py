#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from mavros_msgs.msg import *

class Drone():
    def __init__(self):
        self.position_subscriber = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.global_position_callback)
        self.movement_publisher = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        rate = rospy.Rate(5)
        self.latitude = 0
        self.longitude = 0

    def global_position_callback(self, position_data):
        self.latitude = position_data.latitude
        self.longitude = position_data.longitude
        #rospy.loginfo("longitude: %.7f" %self.longitude)
        #rospy.loginfo("latitude: %.7f" %self.latitude)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            arm_service(True)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            disarm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            disarm_service(False)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def takeoff(self, alt):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
            takeoff_service(altitude = alt, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)

    def land(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            is_landing = land_service(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException as e:
            print("service land call failed: %s. The vehicle cannot land "%e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            is_mode_changed = flight_mode_service(custom_mode=mode) #'GUIDED', 'STABALIZE'
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s."%e)

    def move(self):
        rospy.loginfo('Start')
        pos = PositionTarget()
        pos.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        pos.header.frame_id = 'Drone'
        pos.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
                        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                        | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        
        pos.velocity.x = -0.5
        pos.velocity.y = 0.0  
        pos.velocity.z = 0.0

        pos.acceleration_or_force.x = 0.0
        pos.acceleration_or_force.y = 0.0
        pos.acceleration_or_force.z = 0.0

        pos.position.x = 0.0
        pos.position.y = 0.0
        pos.position.z = 0.0

        pos.yaw = 0.0
        pos.yaw_rate = 0.0

        while not rospy.is_shutdown():
            self.movement_publisher.publish(pos)



def my_loop(drone: Drone):
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7'])):
        menu()
        x = input("Enter your input: ")
        if (x=='1'):
            drone.set_mode(mode='GUIDED')
        elif(x=='2'):
            drone.set_mode(mode='STABILIZE')
        elif(x=='3'):
            drone.arm()
        elif(x=='4'):
            drone.disarm()
        elif(x=='5'):
            drone.takeoff(alt=3)
        elif(x=='6'):
            drone.land()
        elif(x=='7'):
            drone.move()
        else: 
            print ("Exit")


def menu():
    print ("Press")
    print ("1: to set mode to GUIDED")
    print ("2: to set mode to STABILIZE")
    print ("3: to set mode to ARM the drone")
    print ("4: to set mode to DISARM the drone")
    print ("5: to set mode to TAKEOFF")
    print ("6: to set mode to LAND")
    print ("7: Move")
    print ("8: Exit")

if __name__ == '__main__':
    rospy.init_node('drone_move')
    drone = Drone()
    # spin() simply keeps python from exiting until this node is stopped
    my_loop(drone)
    rospy.spin()