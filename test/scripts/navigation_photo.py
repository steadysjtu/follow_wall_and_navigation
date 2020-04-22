#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script
from __future__ import print_function
from std_msgs.msg import Bool
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
find_object = False

def handle_bool(msg):
    global find_object
    find_object = msg.data
	
class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(120)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
	    print(img_title)
            return True
        else:
            return False

if __name__ == '__main__':
        rospy.init_node('nav_test', anonymous=False)
        bool_sub = rospy.Subscriber('/object', Bool, handle_bool)    
      
        while find_object==False:
            pass
        try:
 
	    for i in range(1,4):
    	        navigator = GoToPose()
	
    	    # Customize the following values so they are appropriate for your location
    	        if i==1:
	    	    position = {'x': -5.85, 'y' : -8.42}
	    	#position = {'x': 4.59, 'y' : 4.91}
    	    	    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    	            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    	            success = navigator.goto(position, quaternion)
    	    	if i==2:
    		    camera = TakePhoto()
    # Take a photo
    # Use '_image_title' parameter from command line
    # Default value is 'photo.jpg'
    		    img_title = rospy.get_param('~image_title', 'photo.jpg')
    		    if camera.take_picture(img_title):
        	        rospy.loginfo("Saved image " + img_title)
    		    else:
        	        rospy.loginfo("No images received")
    # Sleep to give the last log messages time to be sent
    		        rospy.sleep(1)
		if i==3:
		   position = {'x': -4.43, 'y' : -12.1}
	    	#position = {'x': 4.59, 'y' : 4.91}
    	    	   quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    	           rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    	           success = navigator.goto(position, quaternion)
	
    	        if success:
    	            rospy.loginfo("Hooray, reached the desired pose")
    	        else:
    	            rospy.loginfo("The base failed to reach the desired pose")
    	    # Sleep to give the last log messages time to be sent
                rospy.sleep(1)

        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")

