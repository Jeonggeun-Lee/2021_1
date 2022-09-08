#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scout_msgs.msg  import ScoutStatus
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np

lidar_data = []
LIDAR_ERR = 0.1

class scoutTranslationTest :

    def __init__(self):
        print("in")
        rospy.init_node('translationTest', anonymous = True)
        self.ctrl_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scout_status',ScoutStatus, self.status_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        ctrl_pub_msg = Twist() # creating a Twist message type obj

        rate = rospy.Rate(20) #20hz
        while not rospy.is_shutdown():
            
            #key = getKey(key_timeout) 
            #if key == 'i':

            # go forward
            ctrl_pub_msg.angular.z = 0.0
            ctrl_pub_msg.linear.x = 0.5

            if stop_sign == True:
                ctrl_pub_msg.angular.z = 0.0
                ctrl_pub_msg.linear.x = 0.0
            # go backward 
            #ctrl_pub_msg.angular.z =0.0
            #ctrl_pub_msg.linear.x =-1.0

            #turn left 
            #ctrl_pub_msg.angular.z =0.3
            #ctrl_pub_msg.linear.x =-1.0

            #turn right
            #ctrl_pub_msg.angular.z = -0.3
            #ctrl_pub_msg.linear.x =1.0


            '''
            if stop_sign == True:
                # for i in range(5):
                #    ctrl_pub_msg.angular.z = 0.0
                #    ctrl_pub_msg.linear.x = =-1.0
                ctrl_pub_msg.angular.z = 0.0
                ctrl_pub_msg.linear.x = 0.0 
                if rotation_sign == "LEFT" and try == False:
                    for i in range(5):
                        ctrl_pub_msg.angular.z = 1.0
                    rotation_sign = "RIGHT"
                    try = True
            
                elif rotation_sign == "RIGHT" and try == False:
                    for i in range(5):
                        ctrl_pub_msg.angular.z = -1.0    
                    rotation_sign = "LEFT"
                    try = True
                
                elif rotation_sign == "LEFT" and try == True:
                    for i in range(10):
                        ctrl_pub_msg.angular.z = 1.0
                    rotation_sign = "RIGHT"

                elif rotation_sign == "RIGHT" and try == True:
                    for i in range(10):
                        ctrl_pub_msg.angular.z = -1.0
                    rotation_sign = "LEFT"

                print("stopped")
            else:
                ctrl_pub_msg.angular.z = 0.0
                ctrl_pub_msg.linear.x =1.0 '''

            self.ctrl_pub.publish(ctrl_pub_msg)
            
            rate.sleep()

    def status_callback(self, msg):
        #linear, angular 
        #print(msg.linear_velocity, msg.angular_velocity)
        pass

    def laser_callback(self, data):

        global lidar_data
        global LIDAR_ERR
        global stop_sign

        lidar_data = []

        for i in range(360):
            if i <= 30 or i > 330:
                if data.ranges[i] >= LIDAR_ERR:
                    lidar_data.append(data.ranges[i])
                    # print(lidar_data)

        if min(lidar_data) < 2:
            stop_sign = True
            print('true')

        else:
            stop_sign = False
            print('false')


    '''
    def getKey(key_timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    '''

if __name__ == '__main__':

    test = scoutTranslationTest()