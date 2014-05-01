#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 3002 Final Project
"""
import rospy
import copy
from nav_msgs.msg import OccupancyGrid

"""
The purpose of this node is to expand obstacles from the /map topic and
publish the new map to /map_OE. All references to OE stand for 
Obstacle Expansion, Obstacle Expanded or something along those lines. 
"""
class Rotate:
    
    def rotate(angle):
        twist = Twist()
        twist.linear.x = 0;
        twist.linear.y = 0; twist.angular.y = 0
        twist.linear.z = 0; twist.angular.x = 0
        twist.angular.z = 1
        
        now = rospy.get_rostime()
        while(now.secs + 2 < rospy.get_rostime() and not rospy.is_shutdown()):
            self.pub.publish(twist)
        twist.angular.z = 0
        self.pub.publish(twist)
            
        #=======================================================================
        # global odom_list
        # global pose
        # 
        # print angle
        # start_angle = robotyaw
        # 
        # if angle > 0:
        #     twist.angular.z = 1;
        # else:
        #     twist.angular.z = -1;
        #     
        # print "basic calcs"
        # current_angle = robotyaw
        # while(not(abs(current_angle - start_angle) < abs(angle) + 0.2 and abs(current_angle - start_angle) > abs(angle) - 0.2)):
        #     pub.publish(twist)
        #     current_angle = robotyaw
        #     rospy.sleep(rospy.Duration(.2, 0))
        # print "Done while loop"
        # twist.angular.z = 0;
        # pub.publish(twist)
        # return
        #=======================================================================
    
    def timerCallback(self, event):
            pass # Delete this 'pass' once implemented
            
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_Rotate')
        
        # Setup publisher and Subscriber
        # If you need something to happen repeatedly at a fixed interval, write the code here.
        # Start the timer with the following line of code: 
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
        rospy.Timer(rospy.Duration(30), self.timerCallback)
        
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution
        
# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    rotate = Rotate()
    rospy.spin()