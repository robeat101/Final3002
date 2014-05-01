#!/usr/bin/env python

import rospy, tf
import copy
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from numpy import ma
from math import sqrt
from __builtin__ import pow
from genpy.rostime import Duration
import numpy

class Centroid:
    def getCentroid(self, msg):
        

        x_c = y_c = count = 0

        for cell in  msg.cells:
            x_c = x_c + cell.x
            y_c = y_c + cell.y
            count = count + 1

        x_c = x_c / count
        y_c = y_c / count

        centroid = PointStamped()
        centroid.point.x = x_c
        centroid.point.y = y_c
        centroid.header.frame_id = 'map'
        #publish centroid point
        self.centroid_publish.publish(centroid)
        print "centroid x = " + str(centroid.point.x)
        print "centroid y = " + str(centroid.point.y)




    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_Centroid')
        
        
        self.msg = None
        self.labels = []
        
        # Setup publisher and Subscriber
        sub = rospy.Subscriber('/rbefinal/blob', GridCells, self.getCentroid, queue_size=1)
        self.centroid_publish = rospy.Publisher('/rbefinal/centroidgoal', PointStamped, latch = True)
        
# This is the program's main function
if __name__ == '__main__':
    
    node = Centroid()
    rospy.spin()

