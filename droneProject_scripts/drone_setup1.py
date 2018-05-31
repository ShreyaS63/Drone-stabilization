#!/usr/bin/env python


import cv2
import numpy as np
import time
import rospy
import tf
from cv_bridge import*
from geometry_msgs.msg import PoseStamped
from math import *
from mavros.utils import *
import message_filters
from sensor_msgs.msg import*
from mavros import*
from std_msgs.msg import String

msg = PoseStamped()
#pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

## pattern noise filtering
def ok(a,arr):
    for index,i in enumerate(arr):
        if((i[0]-a[0])**2+(i[1]-a[1])**2)<20:
            if(i[2]>a[2]):
                return
            else:
                arr.pop(index)
                arr.append(a)
                return
    arr.append(a)
    return


def marker_points(image,COEF = 0.3):
    # image should be in rgb
    stamp = time.time()
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.GaussianBlur(gray_image, (11,11),0)  
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray_image)
    bound = maxVal*COEF
    raw_pixels = np.argwhere(gray_image >= bound)
    values = np.reshape(gray_image[gray_image>=bound],(len(raw_pixels),1))
    raw_pixels =  np.concatenate((raw_pixels,values),1)
    #print raw_pixels
    median = np.median(raw_pixels, axis=0)
    pixels = []
    # clear from crap pixels
    for i in raw_pixels:
        if((i[0]-median[0])**2+(i[1]-median[1])**2)<600:
            ok(i,pixels)
    # show?
    print ('marker time')
    print (time.time()-stamp)
    if len(pixels)==6:
            #cv2.imshow("IMAGE", image)
            #for i in pixels:
            #    cv2.circle(image, (i[1],i[0]), 3, (0, 255, 0), -1)
            #    cv2.imshow("IMAGE", image)
            #print (pixels)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            #return np.array(pixels,dtype = "double")
            return np.array(pixels,dtype="double")[:,:-1]
    elif len(pixels)>6:
        return marker_points(image,COEF+0.05)
    else:
        return marker_points(image,COEF-0.05)



def pose_estimation(image):
    stamp =  time.time()

    #3D marker template
    model_points = np.array([
                            (0.0, 0.0, 0.0),             
                            (0.0, 140.0, 0.0),       
                            (70.0, 70.0, 0.0),
                            (140.0, 0.0, 0.0),
                            (140.0, 70.0, 0.0), 
                            (140.0, 140.0, 0.0),
                            ])
    size = image.shape

    #2D image template
    image_points = marker_points(image)
    print image_points
    focal_length = size[1]
    center = (size[1]/2, size[0]/2)
    camera_matrix = np.array(
                             [[focal_length, 0, center[0]],
                             [0, focal_length, center[1]],
                             [0, 0, 1]], dtype = "double"
                             )
    camera_matrix = np.array(
                             [[1.10616906e+03*1.08,0.00000000e+00,6.35668602e+02],
                             [0.00000000e+00,1.10819769e+03*1.08,4.96993394e+02],
                             [0, 0, 1]], dtype = "double"
                             )
    
    #print "Camera Matrix :\n {0}".format(camera_matrix)
    image_points = np.array(image_points,dtype = "double")
    #dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    dist_coeffs = np.array([-0.20523672,0.11735183,-0.00085103,-0.0005939,0.00980464])
    stamp2 = time.time()
    #flag = cv2.SOLVEPNP_ITERATIVE
    (success, rVec, tVec) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
    print ('solvePnPtime:')
    print (time.time()-stamp2)
    print ('time elapsed:')
    print (time.time()-stamp)
    #x-right y-down
    #print translation and rotation vectors
    print ('translation vector:')
    print (tVec)
    print ('rotation vector')
    print (rVec)
    
    #get rotation matrix using Rogrigues function
    Rmat = cv2.Rodrigues(rVec)[0] 

    R = - np.transpose(Rmat)
    #position in world coordinates
    position = R*tVec/1000

    #calculate the euler angles https://stackoverflow.com/questions/16265714/camera-pose-estimation-opencv-pnp
    rad = 180/pi
    roll = atan2(-R[2][1], R[2][2])
    pitch = asin(R[2][0])
    yaw = atan2(-R[1][0], R[0][0])
    #print roll, pitch,yaw
   
    worldFrame = "world"
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = position[0][0]
    msg.pose.position.y = position[1][1]
    msg.pose.position.z = position[2][2]
    quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]
    msg.header.seq += 1
    msg.header.stamp = rospy.Time.now()
    return msg  
     
 
# paths = ['exposure=10_distance=6m_num0.png','exposure=10_distance=9m_num0.png','exposure=10_distance=9m_num1.png',
#          'exposure=10_distance=9m_num2.png','exposure=10_distance=12m_num0.png','exposure=10_distance=12m_num1.png','exposure=10_distance=12m_num2.png']

 
print ('hello world')

#def drone_setup(cf_goal_pos, cf_goal_yaw, cf_name):
def drone_setup1():
    
    rospy.init_node('drone_setup1', anonymous=True)
    
    paths = ['exposure=10_distance=6m_num0.png','exposure=10_distance=9m_num0.png','exposure=10_distance=9m_num1.png',
             'exposure=10_distance=9m_num2.png','exposure=10_distance=12m_num0.png','exposure=10_distance=12m_num1.png','exposure=10_distance=12m_num2.png']

    for path in paths:
         print (path)
         img = cv2.imread(path,1)
         pose_estimation(img)

    
    rate = rospy.Rate(10)

    while( not rospy.is_shutdown() ): 
        
        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        #pub = rospy.Publisher('/mavros/vision_pose/pose', PoseWithCovarianceStamped, queue_size=10)
        pub.publish(msg)
        print "publishing"
        
        rate.sleep()
        #rospy.spin()

        # import time
        # time.sleep(0.1)

        #cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        drone_setup1()
    #except rospy.ROSInterruptException:
    except KeyboardInterrupt:
        pass
