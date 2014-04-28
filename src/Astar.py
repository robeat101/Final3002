#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 3002 Final Project
"""
import rospy
import copy
from nav_msgs.msg import OccupancyGrid
import tf

"""
The purpose of this node is to expand obstacles from the /map topic and
publish the new map to /map_OE. All references to OE stand for 
Obstacle Expansion, Obstacle Expanded or something along those lines. 
"""
class Astar:
    
    
    
    def getOdomTransform(self):
        self.odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
        return self.odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

    # set and print initialpose
    def set_initial_pose(self):
        
        (trans, rot) = self.getOdomTransform()
        
        #rospy.spin()
        self.start.point.x = trans[0]
        self.start.point.y = trans[1]
        self.start.orientation = rot
        
        pose_x = trans[0]
        pose_y = trans[1]
     
        
        start_pos_x = pose_x
        start_pos_y = pose_y
        
        #set initial pose values
        if pose_x % self.robotResolution < (self.robotResolution / 2.0):
            start_pos_x = start_pos_x - (pose_x % (self.robotResolution)) 
        else:
            start_pos_x = start_pos_x - (pose_x % self.robotResolution) + self.robotResolution 
        
        if pose_y % self.robotResolution < (self.robotResolution / 2.0):
            start_pos_y = start_pos_y - (pose_y % (self.robotResolution))
        else:
            start_pos_y = start_pos_y - (pose_y % self.robotResolution) + self.robotResolution
        
        #we have not checked scaling or anything for these:
        #start_orient_x = msg.pose.pose.orientation.x
        #start_orient_y = msg.pose.pose.orientation.y
        #start_orient_z = msg.pose.pose.orientation.z
        #start_w = msg.pose.pose.orientation.w
        
        #print initial pose values
        self.start.point.x = round(start_pos_x, 3)
        self.start.point.y = round(start_pos_y, 3)
        print ""
        print "Initial Pose Values:"
        print "start_pos_x = ", self.start.point.x
        print "start_pos_y = ", self.start.point.y
        
    
    # Define goal pose
    def set_goal_pose (self, msg):
        print "set_goal_pos"
        
        end_pos_x = msg.point.x
        end_pos_y = msg.point.y
        
        #set initial pose values
        if msg.point.x % self.robotResolution < (self.robotResolution / 2.0):
            end_pos_x = msg.point.x - (msg.point.x % self.robotResolution)
        else:
            end_pos_x = msg.point.x - (msg.point.x % self.robotResolution) + self.robotResolution
        
        if msg.point.y % self.robotResolution < (self.robotResolution / 2.0):
            end_pos_y = msg.point.y - (msg.point.y % self.robotResolution)
        else:
            end_pos_y = msg.point.y - (msg.point.y % self.robotResolution) + self.robotResolution
            
            
        self.end.point.x = round(end_pos_x,3)
        self.end.point.y = round(end_pos_y,3)
        
        print "end_pos_x = ", self.end.point.x
        print "end_pos_y = ", self.end.point.y 
        self.set_initial_pose()
        
    
    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_astar')
        
        # Setup publisher and Subscriber
        self.pub_start    = rospy.Publisher('/start', GridCells) # Publisher for start Point
        self.pub_end      = rospy.Publisher('/end'  , GridCells) # Publisher for End Point
        self.pub_path     = rospy.Publisher('/path' , GridCells) # Publisher for Final Path
        self.pub_explored = rospy.Publisher('/explored', GridCells) # Publisher explored GridCells
        self.pub_frontier = rospy.Publisher('/frontier', GridCells) # Publisher explored GridCells
        
        
        print "Starting Astar"
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution

        self.end = AStarNode(1,1.8);
        self.start = AStarNode(-1,-1.8)
        
        # Set up odometry listener 
        self.odom_list = tf.TransformListener()
        
        sub = rospy.Subscriber('/clicked_point', PointStamped, self.set_goal_pose, queue_size=1)  
        sub = rospy.Subscriber('/map_Opt', OccupancyGrid, self.map_function, queue_size=1)

        
        # Use this command to make the program wait for some seconds
        rospy.sleep(rospy.Duration(1, 0))
        print "AStar set up."





# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    astar = Astar(robotResolution)
    rospy.spin()