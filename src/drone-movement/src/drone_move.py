#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from geometry_msgs.msg import Twist

class Drone():
    def __init__(self):
        self.coordinate_subscriber = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.global_position_callback)
        self.movement_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        frame_service = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)
        self.is_frame_set = frame_service(9)
        self.latitude = 0
        self.longitude = 0

    def global_position_callback(self, coordinates):
        self.latitude = coordinates.latitude
        self.longitude = coordinates.longitude
        #rospy.loginfo("longitude: %.7f" %self.longitude)
        #rospy.loginfo("latitude: %.7f" %self.latitude)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_service(True)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_service(False)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s"%e)

    def takeoff(self, alt):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL) 
            # takeoff_service(altitude=alt, latitude=0, longitude=0, min_pitch=0, yaw=0)
            takeoff_service(altitude=alt)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)

    def land(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            # land_service(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            land_service()
        except rospy.ServiceException as e:
            print("service land call failed: %s. The vehicle cannot land "%e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            is_mode_changed = flight_mode_service(custom_mode=mode) #'GUIDED', 'STABILIZE'
            if is_mode_changed == True:
                print("Mode Changed to:", mode)
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s."%e)

    def move(self):
        rospy.loginfo('Start')

        if self.is_frame_set:
            pos = Twist()
            pos.linear.x = 0.0
            pos.linear.y = 0.0
            pos.linear.z = 0.0

            pos.angular.z = 1.0
            
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.movement_publisher.publish(pos)
                rate.sleep()

def menu(drone: Drone):
    while not rospy.is_shutdown():
        print ("1: mode GUIDED")
        print ("2: mode STABILIZE")
        print ("3: ARM the drone")
        print ("4: DISARM the drone")
        print ("5: TAKEOFF")
        print ("6: LAND")
        print ("7: Move")
        print ("8: Exit")
        
        x = input("Choose: ")
        if (x == '1'):
            drone.set_mode(mode='GUIDED')
        elif(x == '2'):
            drone.set_mode(mode='STABILIZE')
        elif(x == '3'):
            drone.arm()
        elif(x == '4'):
            drone.disarm()
        elif(x == '5'):
            drone.takeoff(alt=3)
        elif(x == '6'):
            drone.land()
        elif(x == '7'):
            drone.move()
        else: 
            print("Exiting...")
            exit(0)

if __name__ == '__main__':
    rospy.init_node('drone_move')
    drone = Drone()
    menu(drone)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()