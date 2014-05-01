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

        #initialize some stuff
        numberOfFrontiers = 0

        closestCell = PointStamped()
        closestCell.point.x = None
        closestCell.point.y = None
        closestCell.header.frame_id = 'map'

        currentCell = PointStamped()
        currentCell.point.x = None
        currentCell.point.y = None
        currentCell.header.frame_id = 'map'

        closestCentroid = PointStamped()
        closestCentroid.point.x = None
        closestCentroid.point.y = None
        closestCentroid.header.frame_id = 'map'

        #determine number of frontiers
        for cell in  msg.cells:
            if cell.z > numberOfFrontiers:
                numberOfFrontiers = cell.z
        
        print numberOfFrontiers
        #determine closest frontier centroid
        for frontierNumber in range(int(numberOfFrontiers) + 1):
            currentFrontier = []
            for cell in msg.cells:
                if cell.z == frontierNumber:
                    currentFrontier.append(cell)
                print cell
            print currentFrontier
            #calculate centroid of frontier being examined
            currentCentroid = self.calcCentroid(currentFrontier)

            #figure out which frontier cell is closest to its centroid
            closestCell.point.x = None
            closestCell.point.y = None
            for cell in currentFrontier:
                currentCell.point.x = cell.x
                currentCell.point.y = cell.y
                if closestCell.point.x == None and closestCell.point.y == None:
                    closestCell.point.x = currentCell.point.x
                    closestCell.point.y = currentCell.point.y
                if self.calcDistance(currentCell,currentCentroid) < self.calcDistance(closestCell,currentCentroid):
                    closestCell.point.x = currentCell.point.x
                    closestCell.point.y = currentCell.point.y

            #set centroid to the location of the closest frontier cell to its centroid
            currentCentroid.point.x = closestCell.point.x
            currentCentroid.point.y = closestCell.point.y

            #determine distance from centroid to where robot is
            #check to see if this is closest so far to robot
            if closestCentroid.point.x == None and closestCentroid.point.y == None:
                closestCentroid.point.x = currentCentroid.point.x
                closestCentroid.point.y = currentCentroid.point.y    
            if self.calcDistance(currentCentroid,self.get_robot_position()) < self.calcDistance(closestCentroid,self.get_robot_position()):
                closestCentroid.point.x = currentCentroid.point.x
                closestCentroid.point.y = currentCentroid.point.y

        #publish centroid point
        self.centroid_publish.publish(closestCentroid)
        print "closest centroid:"
        print "x = " + str(closestCentroid.point.x)
        print "y = " + str(closestCentroid.point.y)

    #calculate centroid of a frontier
    def calcCentroid(self, aFrontier):
        x_c = y_c = count = 0
        print aFrontier
        for cell in aFrontier:
            print "I'm in the for loop ahahhahah"
            x_c = x_c + cell.x
            y_c = y_c + cell.y
            count = count + 1

        x_c = x_c / count
        y_c = y_c / count

        centroid = PointStamped()
        centroid.point.x = x_c
        centroid.point.y = y_c
        centroid.header.frame_id = 'map'

        return centroid
        #print "centroid: " + str(centroid.point.z)
        print "x = " + str(centroid.point.x)
        print "y = " + str(centroid.point.y)

    def calcDistance(self, PointStamped1, PointStamped2):
        #change in x
        a = PointStamped2.point.x - PointStamped1.point.x
        #change in y
        b = PointStamped2.point.y - PointStamped2.point.y
        #pythagorean theorem
        c = sqrt(a**2 + b**2)
        
        return c


    #copied set_initial_pose function for this
    def get_robot_position(self):
        robotResolution = 0.2
        """
        try:
            (trans, rot) = self.getOdomTransform()
        except:
            rospy.logwarn("Odometry Data unavailable. Using default start")
            return
        
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

        self.start.point.x = round(start_pos_x, 3)
        self.start.point.y = round(start_pos_y, 3)
        """
        robot_pos = PointStamped()
        robot_pos.point.x = 0
        robot_pos.point.y = 0
        robot_pos.header.frame_id = 'map'

        return robot_pos



    def getOdomTransform(self):
        self.odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
        return self.odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_Centroid')
        
        
        self.msg = None
        self.labels = []
        self.odom_list = tf.TransformListener()
        # Setup publisher and Subscriber
        sub = rospy.Subscriber('/rbefinal/blob', GridCells, self.getCentroid, queue_size=1)
        self.centroid_publish = rospy.Publisher('/rbefinal/centroidgoal', PointStamped, latch = True)

        
# This is the program's main function
if __name__ == '__main__':
    
    node = Centroid()
    rospy.spin()

