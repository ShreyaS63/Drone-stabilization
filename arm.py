#!/usr/bin/python

import rospy
import mavros
from mavros_msgs.msg import State
from mavros import setpoint as SP
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped 
#from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandBoolResponse, CommandTOL, CommandTOLRequest
import time
from tf.transformations import *
import numpy as np
from math import *
from time import sleep
from mavros import command
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *

import thread
import threading
import time
from mavros.utils import *
from tf.transformations import quaternion_from_euler

#Publisher to  mavros/setpoint_position/local
def arm():
    pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('arm', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    msg = SP.PoseStamped(
        header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
            stamp=rospy.Time.now()),    # stamp should update
    )

    mavros.set_namespace()
    command.arming(True)

    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 2
	#print msg.pose.position

        #For demo purposes we will lock yaw/heading to north.
    yaw_degrees = 0  # North
    yaw = radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation = SP.Quaternion(*quaternion)


    for i in range(100):
	pub.publish(msg)
        rate.sleep()



    print("Requesting OFFBOARD mode...")
    try:
        set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
        ret = set_mode(custom_mode="OFFBOARD")
    except rospy.ServiceException as ex:
        fault(ex)
    #if not ret.success:
        print("Request failed. Check mavros logs")
    else:
        print("OFFBOARD mode is set.")


    start_time = time.time()
    print ("Go up 2 meters")
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    	if time.time() - start_time > 5:
    	   break

	
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 2
	#print msg.pose.position

    print ("go up and forward 2 meters")
    start_time = time.time()
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        if time.time() - start_time > 5:
           break


    msg.pose.position.x = 2
    msg.pose.position.y = 0
    msg.pose.position.z = -2
    #print msg.pose.position

    print ("go forward and down 2 meters")
    start_time = time.time()
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        if time.time() - start_time > 5:
           break
    
   
            
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

if __name__ == '__main__':
    try:
	arm()        
        setDisarm()
    except rospy.ROSInterruptException:
        pass

