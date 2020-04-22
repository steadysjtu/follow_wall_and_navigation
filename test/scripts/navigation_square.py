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
from std_msgs.msg import Bool
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
from math import radians
import time
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
class DrawASquare():
    def __init__(self):
    
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
	# 5 HZ
        r = rospy.Rate(5);

	# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
	# by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45); #45 deg/s in radians/s

	#two keep drawing squares.  Go forward for 2 seconds (10 x 5 HZ) then turn for 2 second
	count = 0
        while not rospy.is_shutdown():
	    # go forward 0.4 m (2 seconds * 0.2 m / seconds)
	    rospy.loginfo("Going Straight")
            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
	    # turn 90 degrees
	    rospy.loginfo("Turning")
            for x in range(0,10):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()            
	    count = count + 1
	    if(count == 4): 
                count = 0
	    if(count == 0): 
		break
		

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
		    print("draw a square")
		    DrawASquare()
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

