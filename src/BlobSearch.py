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


"""
The purpose of this node is to find out where the frontier is and  
generate a path towards it. 

"""

class BlobSearch:
    
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
        return x * self.msg.info.resolution + self.msg.info.origin.position.x
    
    def getY(self, index):
        x, y = self.getXYindex(index)
        return y * self.msg.info.resolution + self.msg.info.origin.position.y

    def getLabels(self, neighbors):
        label_values = []
        for neighbor in neighbors:

            label_val = self.labels[self.getMapIndex(neighbor[0], neighbor[1])]
            if label_val not in [-1, 100]:
                label_values.append(label_val)
        return label_values
        
    def isValidPoint(self,x,y):
        if(x >= 0 and y >= 0):
            if(x < self.msg.info.width and y < self.msg.info.height):
                return True
        return False
    
    def getMapIndex(self, x, y):
        return int(round((y * self.msg.info.width) + x, 0))
    
    # Gets all connected cells with labels
    def getConnected(self, x, y):
        neighbors = []
        xy = BlobSearch.xy
        # check 8 connectivity cells around the current xy
        for i in xrange(0, len(xy)):
            # check if current x, y are in range
            if(self.isValidPoint(x + xy[i][0], y + xy[i][1])):
                #check if the cell has a label
                a_label = self.labels[self.getMapIndex(x + xy[i][0], y + xy[i][1])]
                if(a_label != -1):
                    neighbors.append((x + xy[i][0], y + xy[i][1]))
        return neighbors
    
    def min(self, L):
        min_val = -1
        nodes = []
        node = copy.deepcopy(L)
        min_val= min(node).pop()
        for node in L:
            if(node.__contains__(min_val)):
                nodes.append(node)
        min_val = copy.deepcopy(nodes[0])
        for node in nodes:
            if(node.__len__() < min_val.__len__()):
                min_val = copy.deepcopy(node)
        return min_val
            
            
    def beginFS(self, msg):
        Background = -1
        linked = []
        NextLabel = 0
        self.labels = [Background] * msg.info.width * msg.info.height
        self.msg = msg
        
        data = msg.data
        print "begin"
        # First pass
        for x in xrange(0, msg.info.width):
            for y in xrange(0, msg.info.height):
                if data[self.getMapIndex(x, y)] not in [100,-1]:
                    neighbors = self.getConnected(x,y)
                    if neighbors == []:
                        linked.append(set([NextLabel]))                   
                        self.labels[self.getMapIndex(x,y)] = set([NextLabel])
                        NextLabel = NextLabel + 1
                    else:
                       # Find the smallest label
                       L = self.getLabels(neighbors)
                       self.labels[self.getMapIndex(x,y)] = self.min(L)
                       L = self.f7(L)
                       for label in L:
                             linked[linked.index(label)] = set.union(linked[linked.index(label)] , *L)
        #print self.labels
        #=======================================================================
        # # Second pass
        # for x in xrange(0, msg.info.width):
        #     for y in xrange(0, msg.info.height):
        #         if data[self.getMapIndex(x,y)] !=  Background:
        #             label = self.labels[self.getMapIndex(x,y)]        
        #             self.labels[self.getMapIndex(x,y)] = set.update(self.labels[self.getMapIndex(x,y)])    
        #=======================================================================
        
        print "Done with both passes."
        print "publishing"
        self.publishGridCells(self.labels)
        
    def f7(self, seq):
        seen = []
        for i in xrange(0, len(seq)):
            flag = False
            for j in xrange(i, len(seq)):
                if(seq[i] == seq[j]):
                    flag = True
                    break
            if not flag:
                seen.append(seq[i])
        return seen
                           
    def publishGridCells(self, nodes):
        #Initialize gridcell
        gridcells = GridCells()
        gridcells.header.frame_id = 'map'
        gridcells.cell_width = self.msg.info.resolution
        gridcells.cell_height = self.msg.info.resolution
        
        if nodes == None:
            return
        #Iterate through list of nodes
        for i in xrange(0, len(nodes)): 
            if(nodes[i] == -1):
                continue
            elif(self.msg.data[i] == 100):
                continue
            else:
                
                is_internal_count = True
                x, y = self.getXYindex(i)
                
                for j in xrange(0, len(self.xy)): 
                    coord = self.xy[j]
                    coord = (x + coord[0], y + coord[1])
                    coord = (round(coord[0]), round(coord[1]))
                    if(self.isValidPoint(coord[0], coord[1])):
                        if(self.labels[self.getMapIndex(coord[0], coord[1])] == -1 and self.msg.data[self.getMapIndex(coord[0], coord[1])] != 100):
                            is_internal_count = False
                            break
            
            if(is_internal_count):
                continue
            
            label = copy.deepcopy(self.labels[i])
            z = label.pop()
            print "getting point"
            point = Point()
            point.x = self.getX(i)
            point.y = self.getY(i)
            point.z = z
            #Ensure z axis is 0 (2d Map)
            
            gridcells.cells.append(point)
        print self.labels    
        self.blob_publish.publish(gridcells)
        #rospy.sleep(rospy.Duration(0.05,0))
        print "Done Blob Search"
        
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_BlobSearch')
        
        
        self.msg = None
        self.labels = []
        
        # Setup publisher and Subscriber
        sub = rospy.Subscriber('/rbefinal/map_Opt', OccupancyGrid, self.beginFS, queue_size=1)
        self.blob_publish = rospy.Publisher('/rbefinal/blob', GridCells, latch = True)
        
# This is the program's main function
if __name__ == '__main__':
    
    # Create MapOE object
    node = BlobSearch()
    rospy.spin()


    


