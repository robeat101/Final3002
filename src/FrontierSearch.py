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
global 

Searched_Flag = True

"""
The purpose of this node is to read MapOE and Map. 
"""
def FrontSearc():

    


