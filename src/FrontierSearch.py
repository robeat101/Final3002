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
    
    xy = [(-1, -1), (-1, 0), (0, -1),
         (-1, -1), (-1, 0),
         (-1, 1), (0, -1), (0, 1),
         (1, -1), (1, 0),(1, 1)]
    
    def getXYindex(self, index):
        x = index % self.msg.info.width
        y = index - x
        y = y / self.msg.info.width
        x = round(x, 0)
        y = round(y, 0)
        return x, y
    
    def getX(self, index):
        x, y = self.getXYindex(index)
        
    def getLabels(self, neighbors):
        label_values = []
        for neighbor in neighbors:
            label_val = self.labels[self.getMapIndex(neighbor[0], neighbor[1])]
            if label_val not in label_values:
                label_values.append(label_val)
        return label_values
        
    def isValidPoint(self,x,y):
        if(x >= 0 and y >= 0):
            if(x < self.msg.info.width and y < self.msg.info.height):
                return True
        return False
    
    def getMapIndex(self, x, y):
        return (y * self.msg.info.width) + x
    
    # Gets all connected cells with labels
    def getConnected(self, x, y):
        neighbors = []
        xy = FrontierSearch.xy
        # check 8 connectivity cells around the current xy
        for i in xrange(0, len(xy)):
            # check if current x, y are in range
            if(self.isValidPoint(x + xy[0], y + xy[1])):
                #check if the cell has a label
                a_label = self.labels(self.getMapIndex(x + xy[0], y + xy[1]))
                if(a_label != -1):
                    if(self.labels[self.getMapIndex(x,y)] == a_label):
                        neighbors.append((x + xy[0], y + xy[1]))
        return neighbors
            
    def beginFS(self, msg):
        Background = -1
        linked = []
        NextLabel = 0
        self.labels = [Background] * len(msg.info.width) * len(msg.info.height)
        self.msg = msg
        
        data = msg.data
        
        # First pass
        for x in xrange(0, len(msg.info.width)):
            for y in xrange(0, len(msg.info.height)):
                if data[self.getMapIndex(x, y)] is not Background:
                    neighbors = self.getConnected(x,y)
                    if neighbors != []:
                        linked.append(NextLabel)                   
                        labels[self.getMapIndex(x,y)] = NextLabel
                        NextLabel = NextLabel + 1
                    else:
                       # Find the smallest label
                       L = self.getLabels(neighbors)
                       labels[self.getMapIndex(x,y)] = min(L)
                       for label in L:
                           linked[label] = union(linked[label], L)
        # Second pass
        for x in xrange(0, len(msg.info.width)):
            for y in xrange(0, len(msg.info.height)):
                if data[self.getMapIndex(x,y)] is not Background:         
                     labels[self.getMapIndex(x,y)] = find(labels[self.getMapIndex(x,y)])    
        print "Done with both passes."
        self.publishGridCells(labels)
        
        
    def publishGridCells(self, node):
        #Initialize gridcell
        gridcells = GridCells()
        gridcells.header.frame_id = 'map'
        gridcells.cell_width = self.map.info.resolution
        gridcells.cell_height = self.map.info.resolution
        
        if nodes == None:
            return
        #Iterate through list of nodes
        for i in xrange(0, len(nodes)): 
            point = Point()
            point.x = self.getX(i)
            point.y = self.getY(i)
            #Ensure z axis is 0 (2d Map)
            point.z = node.point.z = 0
            gridcells.cells.append(point)        
        publisher.publish(gridcells)
        #rospy.sleep(rospy.Duration(0.05,0))
        
        
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_FrontierSearch')
        
        
        self.msg = None
        self.labels = []
        
        # Setup publisher and Subscriber
        sub = rospy.Subscriber('/rbefinal/MapOpt', OccupancyGrid, self.beginFS, queue_size=1)
        self.blob_publish = rospy.Publisher('/rbefinal/blob', GridCells, latch = True)
        
# This is the program's main function
if __name__ == '__main__':
    
    # Create MapOE object
    node = FrontierSearch()
    rospy.spin()


    


