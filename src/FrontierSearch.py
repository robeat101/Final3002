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


"""
The purpose of this node is to find out where the frontier is and  
generate a path towards it. 

"""

class FrontierSearch:

    def beginFS(self):
        pass
    
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_FrontierSearch')
        
        # Setup publisher and Subscriber
        self.map = rospy.Subscriber('/rbefinal/MapOpt', OccupancyGrid, self.beginFS, queue_size=1)
        
        
# This is the program's main function
if __name__ == '__main__':
    
    # Create MapOE object
    node = FrontierSearch()
    rospy.spin()


    


