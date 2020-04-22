#! /usr/bin/env python

# import ros stuff
import rospy
import cv2
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import numpy as np 
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
###############wall follower variables#########################################
active_ = True
pub_ = None
bool_pub = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

######################dectecting object variables#####################################
ball_color = 'red'

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              }
counters_ = 0
####################wall follower#################################
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:288]), 10),
        'front':  min(min(msg.ranges[289:433]), 10),
        'fleft':  min(min(msg.ranges[434:578]), 10),
        'left':   min(min(msg.ranges[579:719]), 10),
    }
    #print len(msg.ranges)
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.8
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(2)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3#if on left, 0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3#if on right,-0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def wall_follower():
    global pub_, active_,counters_,bool_pub
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/turtlebot/laser/scan', LaserScan, clbk_laser)
    
    bool_pub  = rospy.Publisher('object', Bool, queue_size=10)
    Image_converter()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():#when rospy.is_shutdown==null,then loop
        if not active_:   #if active==false,sleep
            rate.sleep()
            continue
   
        msg = Twist()
        detect_obj = Bool()
	

        if state_ == 0: #judge the state of turtlebot
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
	#publish velocity 
        pub_.publish(msg)
        print(counters_)
        #rospy.loginfo(msg)
	if(counters_<20): 
	    rospy.loginfo("detecting...")
	    detect_obj = False
            bool_pub.publish(detect_obj)
	    #print(counters_)   
	if(counters_>=20):
	    detect_obj = True
	    print("object has been found,then go to the corresponding room")
            bool_pub.publish(detect_obj)	
	    rospy.signal_shutdown("go to the terminal room")    
        rate.sleep()
        #rospy.spin()
################################detect object part#################################
class Image_converter:
    def __init__(self):

	self.bridge = CvBridge()

	self.image_pub = rospy.Publisher('table_detect_test',Image,queue_size = 10)
	
	self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)

    	# Allow up to one second to connection
        #rospy.sleep(1)

    def callback(self,data):

		# Convert image to OpenCV format
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")#it's a convertion between rosimage and opencv
		except CvBridgeError as e:
		    print e

		detect_image = self.detect_table(cv_image)

		try:
	            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, "bgr8"))
	        except CvBridgeError as e:
	            print e


    def detect_table(self,image):

		global counters_
		if counters_<20:
		    g_image = cv2.GaussianBlur(image, (5, 5), 0)	#gaussianBlur	
		    hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)  #convert BGR to HSV    
                    erode_hsv = cv2.erode(hsv, None, iterations=2)  #erode               
                    inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])#set a range of color to delete the background,modify the color here!!!!!!
                    cnts = cv2.findContours(inRange_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		#print cnts
		    if len(cnts):                            #judge if the object is detected  
                        c = max(cnts, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        cv2.drawContours(image, [np.int0(box)], -1, (0, 255, 255), 2)
		        print("object detected")
                        counters_= counters_+1
		else:
		    pass
                                 
		return image

###############################  main  ###########################################################
if __name__ == '__main__':
    wall_follower()








