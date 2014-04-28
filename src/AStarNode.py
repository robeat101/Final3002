#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 3002 Final Project
"""
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist, PointStamped
from Direction import *


"""
The purpose of this class is to provide a definition for each node in the  A*
algorithm. 
"""

   
class AStarNode():   
    def __init__(self, x, y, dir = None):
        direction = Direction()
        self.point = Point()
        self.point.x, self.point.y = x, y
        self.g = 0
        self.h = 0
        self.parent = None
        if(dir == None):
            self.step_direction = direction.start
        else:
            self.step_direction = dir

    def poseEqual(self, node):
        return_val = round(node.point.x, 3) == round(self.point.x,3)
        return_val = return_val and round(node.point.y, 3) == round(self.point.y,3)     
        return return_val
