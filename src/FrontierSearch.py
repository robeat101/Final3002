#!/usr/bin/env python

import rospy, tf
import Movement
import copy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from numpy import ma
from math import sqrt
from __builtin__ import pow
from genpy.rostime import Duration
import numpy
import MapOE
import MapOpt


global Searched_Flag
global Front_set


Front_set = set()

Searched_Flag = True

"""
The purpose of this node is to find out where the frontier is and  
generate a path towards it. 

"""

class FrontSearch:

   def readMap():


 





    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_MapOE')
        
        # Setup publisher and Subscriber
        self.OEmap = rospy.Publisher('/front_search', OccupancyGrid, latch=True)
        #self.map = rospy.Subscriber('/map', OccupancyGrid, self.beginOE, queue_size=1)
        
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution
        
# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    map_OE = MapOE(robotResolution)
    rospy.spin()


    


