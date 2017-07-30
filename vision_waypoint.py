#!/usr/bin/env python


import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import*
import time
from tf.transformations import *
import numpy as np
from math import *
from time import sleep
from mavros import command
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import thread
import threading
import time
from mavros.utils import *
from tf.transformations import *

import message_filters
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

data_marker = AlvarMarkers()
data_imu = Imu()

data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)


def callback_imu(data):
	global data_imu	
	data_imu = data

def callback_marker(data):
	data_marker = data
	

	rospy.Subscriber('mavros/imu/data', Imu, callback_imu)
	quaternion = (
    		data_imu.orientation.x,
    		data_imu.orientation.y,
    		data_imu.orientation.z,
    		data_imu.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	#math.tan(euler[0])

	if not data_marker.markers:
		print "no markers"
	else:
		print "\nMarker position:\n"
		for i in range(len(data_marker.markers)):
			if data_marker.markers[i].id == 0:
				data_to_publish.header = data_marker.markers[i].header
				data_to_publish.pose = data_marker.markers[i].pose.pose
				marker_dist = abs(data_marker.markers[i].pose.pose.position.z)
				print "marker_dist:", marker_dist
				data_to_publish.pose.position.z = - data_to_publish.pose.position.z +1.04 
				data_to_publish.pose.position.y = data_to_publish.pose.position.y  - tan(euler[1])*marker_dist
				data_to_publish.pose.position.x = data_to_publish.pose.position.x - tan(euler[0])*marker_dist
				# TODO: take yaw into account for x, y calculations
				#print "popravka: ", tan(euler[0])*marker_dist
				print data_to_publish.pose
				pub.publish(data_to_publish)
		
		
		
		print "\nImu data:\n"
		#print data_imu.orientation
		#print "roll: ", euler[0]
		print "roll: ", degrees(euler[0])
		#print "pitch: ", euler[1]
		print "pitch: ", degrees(euler[1])
		print

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

	
    msg.pose.position.x = 2
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


def vision_waypoint():
	
	rospy.init_node('vision_waypoint', anonymous=True)
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback_marker)
	rospy.spin()
	mavros.set_namespace()  # initialize mavros module with default namespace
    	rate = rospy.Rate(10)
    


if __name__ == '__main__':
	try:
		#command.arming(True)
	        vision_waypoint()
		arm()        
                setDisarm()
		
	except rospy.ROSInterruptException:
		pass
