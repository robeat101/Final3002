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
class Astar:
    
    def set_goal_pose (self, msg):
        print "set_goal_pos"
        
        self.map_scaled_cell_width = self.robotResolution
        self.map_scaled_cell_height = self.robotResolution
        end_pos_x = msg.point.x
        end_pos_y = msg.point.y
        
        #set initial pose values
        if msg.point.x % map_scaled_cell_width < (map_scaled_cell_width / 2.0):
            end_pos_x = msg.point.x - (msg.point.x % map_scaled_cell_width)
        else:
            end_pos_x = msg.point.x - (msg.point.x % map_scaled_cell_width) + map_scaled_cell_width
        
        if msg.point.y % map_scaled_cell_height < (map_scaled_cell_height / 2.0):
            end_pos_y = msg.point.y - (msg.point.y % map_scaled_cell_height)
        else:
            end_pos_y = msg.point.y - (msg.point.y % map_scaled_cell_height) + map_scaled_cell_height
            
            
        end.point.x = round(end_pos_x,3)
        end.point.y = round(end_pos_y,3)
        
        print "end_pos_x = ", end.point.x
        print "end_pos_y = ", end.point.y 
        set_initial_pose()
    
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