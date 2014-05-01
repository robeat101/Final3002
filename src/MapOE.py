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
class MapOE:
    
    def beginOE(self, map):
        
        # Check to see if Obstacle Expansion is necessary
        if(self.robotResolution < round(map.info.resolution, 3)):
            new_map = map
        else:
            # Calculate number of blocks to expand obstacles by
            target_exp = int(round(self.robotResolution/map.info.resolution, 0))
            
            # Create a copy of current map message
            new_map = copy.copy(map)
            
            
            # Figure out the obstacle values for different x,y coordinates
            MapPosStatus = {}
            for x in xrange(0, map.info.width):
                for y in xrange(0, map.info.height):
                    index = y * map.info.width + x
                    MapPosStatus[(x,y)] = map.data[index]
            
            # create xy pairs for changes in cell locations. 
            # example, if target_exp = 1, xy = 
            # [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 0), (0, 1), (1, -1), (1, 0), (1, 1)]
            xy = []
            for i in xrange(-target_exp, target_exp+1):
                for j in xrange(-target_exp, target_exp+1):
                    xy.append((i,j));
            
            # Store Map occupancy data as a list to allow modification
            new_data = list(new_map.data)
            
            # Expand obstacles
            for x in xrange(0, map.info.width):
                for y in xrange(0, map.info.height):
                    if(MapPosStatus[x,y] == 100):
                        for element in xy:
                             index = (y + element[1]) * map.info.width + (x + element[0])
                             if(index >= 0 and index < len(new_data)):
                                    new_data[index] =  100
            # Convert and store data as tuples. 
            new_map.data = tuple(new_data)
        
        self.OEmap.publish(new_map)
        rospy.sleep(rospy.Duration(1,0))
    
    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_MapOE')
        
        # Setup publisher and Subscriber
        self.OEmap = rospy.Publisher('/rbefinal/map_OE', OccupancyGrid, latch=True)
        self.map = rospy.Subscriber('/map', OccupancyGrid, self.beginOE, queue_size=1)
        
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution
        
# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    map_OE = MapOE(robotResolution)
    rospy.spin()