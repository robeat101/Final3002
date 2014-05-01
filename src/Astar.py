#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 3002 Final Project
"""
import rospy
import copy
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import tf
from math import sqrt
from AStarNode import AStarNode
from Direction import Direction

"""
The purpose of this node is to expand obstacles from the /map topic and
publish the new map to /map_OE. All references to OE stand for 
Obstacle Expansion, Obstacle Expanded or something along those lines. 
"""
class Astar:
    
    def heuristic(self, current, end):
        self.h_const
        x_2 = pow((current.point.x - end.point.x), 2)
        y_2 = pow((current.point.y - end.point.y), 2)
        h = sqrt(x_2+y_2)
        return 20 * h / self.h_const
    
       
    
    def getOdomTransform(self):
        self.odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
        return self.odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))

    # set and print initialpose
    def set_initial_pose(self):
        
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
        
        #we have not checked scaling or anything for these:
        #start_orient_x = msg.pose.pose.orientation.x
        #start_orient_y = msg.pose.pose.orientation.y
        #start_orient_z = msg.pose.pose.orientation.z
        #start_w = msg.pose.pose.orientation.w
        
        #print initial pose values
        self.start.point.x = round(start_pos_x, 3)
        self.start.point.y = round(start_pos_y, 3)
        print ""
        print "Initial Pose Values:"
        print "start_pos_x = ", self.start.point.x
        print "start_pos_y = ", self.start.point.y
        
    
    # Define goal pose
    def set_goal_pose (self, msg):
        print "set_goal_pos"
        
        end_pos_x = msg.point.x
        end_pos_y = msg.point.y
        
        #set initial pose values
        if msg.point.x % self.robotResolution < (self.robotResolution / 2.0):
            end_pos_x = msg.point.x - (msg.point.x % self.robotResolution)
        else:
            end_pos_x = msg.point.x - (msg.point.x % self.robotResolution) + self.robotResolution
        
        if msg.point.y % self.robotResolution < (self.robotResolution / 2.0):
            end_pos_y = msg.point.y - (msg.point.y % self.robotResolution)
        else:
            end_pos_y = msg.point.y - (msg.point.y % self.robotResolution) + self.robotResolution
            
            
        self.end.point.x = round(end_pos_x,3)
        self.end.point.y = round(end_pos_y,3)
        
        print "end_pos_x = ", self.end.point.x
        print "end_pos_y = ", self.end.point.y 
        self.set_initial_pose()
        self.goal_set = True
        
        if(self.map_available):
            self.run_Astar()    
      
    def run_Astar(self):
        self.goal_set = False
        
        self.PublishGridCells(self.pub_start, [self.start])
        self.PublishGridCells(self.pub_end,   [self.end])
        self.PublishGridCells(self.pub_path, [])
        
        # Use this command to make the program wait for some seconds
        rospy.sleep(rospy.Duration(.1, 0))
        
        path = self.AStar_search()
        print "h, change_x, change_y"
        print self.start.h
        print self.end.point.x - self.start.point.x
        print self.end.point.y - self.start.point.y
        
        if path == [] or path == None:
            print "No Path!"
            return None
        else:
            print "Displaying Path"
            self.PublishGridCells(self.pub_path, path)
            rospy.sleep(rospy.Duration(1, 0))
            self.PublishGridCells(self.pub_path, [])
            print "Showing Waypoints"
            waypoints = self.getWaypoints(path)
            self.PublishWayPoints(self.pub_path, waypoints)
            print "Publishing nav goal"
            self.publishGoal()
            
    def publishGoal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = self.end.point.x
        goal.pose.position.y = self.end.point.y
        goal.pose.position.z = 0
        if(self.orientation != None):
            goal.pose.orientation = self.orientation
        self.pub_goal.publish(goal)
        rospy.sleep(0.5)
        
    
    def getWaypoints(self, path):
        waypoints = []
        direction = Direction()
        previous = self.start
        previous.direction = direction.start
        
        for node in path:
            if(node.step_direction != previous.step_direction):
                waypoints.append(node)
                previous = node
            else:
                continue
        waypoints.append(self.end)
        
        return waypoints    

    def AStar_search(self):
        #Set Start and End values
        self.start.g = 0
        self.start.h = self.heuristic(self.start, self.end)
        rospy.sleep(rospy.Duration(0.1,0))
        
        #FrontierSet is the set of nodes to be opened, i.e. Frontier
        FrontierSet = set()
        #ExpandedSet is the set of nodes already expanded
        ExpandedSet = set()
        
        #Add Start to frontier
        FrontierSet.add(self.start)
        
        while FrontierSet:
            self.PublishGridCells(self.pub_frontier, FrontierSet)
            self.PublishGridCells(self.pub_explored, ExpandedSet)
            self.PublishGridCells(self.pub_start, [self.start])
            self.PublishGridCells(self.pub_end, [self.end])
            
            #find the node in FrontierSet with the minimum heuristic value
            current = min(FrontierSet, key=lambda o:o.g + o.h)
            
            # Check if we should abort search and navigate to unknown
            if(self.map.data[self.getMapIndex(current)] == -1):
                self.end = current
                print "Found shortcut to frontier"
            #If the goal is being expanded
            if current.poseEqual(self.end):
                #Construct path
                path = []
                repeatedNode_flag = False
                FrontierSet.remove(current)
                
                while current.parent:
                    path.append(current)
                    current = current.parent
                    
                    #remove current node from ExpandedSet
                    for expanded in ExpandedSet:
                        if current.poseEqual(expanded):
                            ExpandedSet.remove(expanded)
                            break
                    path.append(current)
                
                
                for frontier in FrontierSet:
                    if self.end.poseEqual(frontier):
                        FrontierSet.remove(frontier)

                rospy.sleep(rospy.Duration(.2,0))
                #update gridcells
                self.PublishGridCells(self.pub_explored, ExpandedSet)
                self.PublishGridCells(self.pub_frontier, FrontierSet)
                
                #Return path (less one garbage node that is appended and less the start node)
                return path[1:-1]
            #Else, move node from frontier to explored
            FrontierSet.remove(current)
            
            if not (current.poseEqual(self.start) or current.poseEqual(self.end)):
                ExpandedSet.add(current)
                
            #Check for possible 8-directional moves
            repeatedNode_flag = False
            somethingWasUpdatedFlag = False
            
            nodesToGo = self.WhereToGo(current)
            if nodesToGo == []:
                return []
                
            for node in nodesToGo:
                #Ignore if node is expanded
                for expanded in ExpandedSet:
                    if node.poseEqual(expanded):
                        new_g = current.g + self.move_cost(current,node)
                        if node.g > new_g:
                            node.g = new_g
                            node.h = self.heuristic(node, self.end)
                            node.parent = current
                        repeatedNode_flag = True
                        somethingWasUpdatedFlag = True
                        break   
            
                #Try to update cost of traveling to node if already exists
                for frontier in FrontierSet:
                    if node.poseEqual(frontier) and repeatedNode_flag == False:
                        new_g = current.g + self.move_cost(current,node)
                        if node.g > new_g:
                            node.g = new_g
                            node.h = self.heuristic(node, self.end)
                            node.parent = current
                            FrontierSet.remove(frontier)
                            FrontierSet.add(node)
                        somethingWasUpdatedFlag = True
                        break
                    if somethingWasUpdatedFlag == True:
                        break
                #Add to frontier and update costs and heuristic values
                if somethingWasUpdatedFlag == False:
                    node.g = current.g + self.move_cost(current, node)
                    node.h = self.heuristic(node, self.end)
                    node.parent = current
                    FrontierSet.add(node)
                repeatedNode_flag = False
                somethingWasUpdatedFlag = False
        return None
    
    #this needs to be universalized for other maps
    def getMapIndex(self, node):        
        a = (((node.point.y - self.map.info.origin.position.y) / self.robotResolution) * self.map.info.width)
        a = a + ((node.point.x - self.map.info.origin.position.x) / self.robotResolution)
        return int(round(a,2))

    def move_cost(self, node, next):        
        
        map_data = self.map.data
        if(node.step_direction == next.step_direction):
            return round(self.robotResolution * (20 + map_data[self.getMapIndex(next)]), 3)
        else:
            step_diff = abs(node.step_direction - next.step_direction)
            if step_diff >= 4: 
                step_diff = 8 - step_diff
            return round(self.robotResolution * (20 + map_data[self.getMapIndex(next)]), 3) + step_diff * 2
        
    #Takes in current node, returns list of possible directional movements
    def WhereToGo(self, node):
        
        diagonal_distance = sqrt(2) * self.robotResolution
        map_data = self.map.data
        possibleNodes = []
    
        direction = Direction()
        #Hacky code begins
        North = AStarNode(round(node.point.x,3), round(node.point.y+self.robotResolution,3))
        North.g = node.g + self.robotResolution
        North.step_direction = direction.n
        
        NorthEast = AStarNode(round(node.point.x+self.robotResolution,3), round(node.point.y+self.robotResolution,3))
        NorthEast.g = node.g + diagonal_distance
        NorthEast.step_direction = direction.ne
        
        East = AStarNode(round(node.point.x+self.robotResolution,3), round(node.point.y, 3))
        East.g = node.g + self.robotResolution
        East.step_direction = direction.e
        
        SouthEast = AStarNode(round(node.point.x+self.robotResolution,3), round(node.point.y-self.robotResolution,3))
        SouthEast.g = node.g + diagonal_distance
        SouthEast.step_direction = direction.se
        
        South = AStarNode(round(node.point.x, 3), round(node.point.y-self.robotResolution,3))
        South.g = node.g + self.robotResolution
        South.step_direction = direction.s
        
        SouthWest = AStarNode(round(node.point.x-self.robotResolution,3), round(node.point.y-self.robotResolution,3))
        SouthWest.g = node.g + diagonal_distance
        SouthWest.step_direction = direction.sw
        
        West = AStarNode(round(node.point.x-self.robotResolution,3), round(node.point.y, 3))
        West.g = node.g + self.robotResolution
        West.step_direction = direction.w
        
        NorthWest = AStarNode(round(node.point.x-self.robotResolution,3), round(node.point.y+self.robotResolution,3))
        NorthWest.g = node.g + diagonal_distance
        NorthWest.step_direction = direction.nw
        
        if(self.getMapIndex(North) < len(map_data) and self.getMapIndex(North) > 0):
            if (map_data[self.getMapIndex(North)] not in [100] ):
                possibleNodes.append(North)
                
        if(self.getMapIndex(NorthEast) < len(map_data) and self.getMapIndex(NorthEast) > 0):
            if (map_data[self.getMapIndex(NorthEast)] not in [100]):
                possibleNodes.append(NorthEast)
                
        if(self.getMapIndex(East) < len(map_data) and self.getMapIndex(East) > 0):
            if (map_data[self.getMapIndex(East)] not in [100]):
                possibleNodes.append(East)
                
        if(self.getMapIndex(SouthEast) < len(map_data) and self.getMapIndex(SouthEast) > 0):
            if (map_data[self.getMapIndex(SouthEast)] not in [100]):
                possibleNodes.append(SouthEast)
                
        if(self.getMapIndex(South) < len(map_data) and self.getMapIndex(South) > 0):
            if (map_data[self.getMapIndex(South)] not in [100]):
                possibleNodes.append(South)
                
        if(self.getMapIndex(SouthWest) < len(map_data) and self.getMapIndex(SouthWest) > 0):
            if (map_data[self.getMapIndex(SouthWest)] not in [100]):
                possibleNodes.append(SouthWest)
                
        if(self.getMapIndex(West) < len(map_data) and self.getMapIndex(West) > 0):
            if (map_data[self.getMapIndex(West)] not in [100]):
                possibleNodes.append(West)
                
        if(self.getMapIndex(NorthWest) < len(map_data) and self.getMapIndex(NorthWest) > 0):
            if (map_data[self.getMapIndex(NorthWest)] not in [100]):
                possibleNodes.append(NorthWest)
    
        return possibleNodes

    def map_function(self, map):
        
        self.map_available = True
        self.map = map
        if(self.goal_set):
            self.run_Astar()   
            
            
    def PublishWayPoints(self, publisher, nodes):
        rospy.sleep(rospy.Duration(0.1,0))
        #Initialize gridcell
        gridcells = GridCells()
        gridcells.header.frame_id = 'map'
        gridcells.cell_width = self.robotResolution
        gridcells.cell_height = self.robotResolution
        
        if nodes == None:
            return
        #Iterate through list of nodes
        for node in nodes: 
            point = Point()
            point.x = node.point.x
            point.y = node.point.y
            #Ensure z axis is 0 (2d Map)
            point.z = node.point.z = 0
            gridcells.cells.append(point)        
        publisher.publish(gridcells)
        rospy.sleep(rospy.Duration(0.1,0))
    
    #Publish Grid Cells function
    def PublishGridCells(self, publisher, nodes):
        #Initialize gridcell
        gridcells = GridCells()
        gridcells.header.frame_id = 'map'
        gridcells.cell_width = self.robotResolution
        gridcells.cell_height = self.robotResolution
        
        if nodes == None:
            return
        #Iterate through list of nodes
        for node in nodes: 
            point = Point()
            point.x = node.point.x
            point.y = node.point.y
            #Ensure z axis is 0 (2d Map)
            point.z = node.point.z = 0
            gridcells.cells.append(point)        
        publisher.publish(gridcells)
        #rospy.sleep(rospy.Duration(0.05,0))

    def readOdom(self, msg):
        self.orientation = msg.pose.pose.orientation
        
    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_astar')
        
        # Setup publisher and Subscriber
        self.pub_start    = rospy.Publisher('/rbefinal/astar/start', GridCells) # Publisher for Start Point
        self.pub_end      = rospy.Publisher('/rbefinal/astar/end'  , GridCells) # Publisher for End Point
        self.pub_path     = rospy.Publisher('/rbefinal/astar/path' , GridCells) # Publisher for Final Path
        self.pub_explored = rospy.Publisher('/rbefinal/astar/explored', GridCells) # Publisher explored GridCells
        self.pub_frontier = rospy.Publisher('/rbefinal/astar/frontier', GridCells) # Publisher Frontier GridCells
        self.pub_goal     = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True) #Publisher for Nav Goal

        print "Starting Astar"
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution

        self.end = AStarNode(1,1.8);
        self.start = AStarNode(-1,-1.8)
        
        self.map_available = False
        self.goal_set = False
        self.h_const = .4
        self.map = None
        self.orientation = None
        
        
        # Set up odometry listener 
        self.odom_list = tf.TransformListener()
        
        sub = rospy.Subscriber('/rbefinal/centroidgoal', PointStamped, self.set_goal_pose, queue_size=1)  
        sub = rospy.Subscriber('/rbefinal/map_Opt', OccupancyGrid, self.map_function, queue_size=1)
        sub = rospy.Subscriber('/odom', Odometry, self.readOdom, queue_size=5)
    
        
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