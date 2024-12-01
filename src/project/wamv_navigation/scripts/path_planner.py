#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import numpy as np

'''
slightly based on methods seen in AtsushiSakai/PythonRobotics A_star
'''

class Node:
    
    """
    creates an object for each Node 
    lt dunder method is used to compare cost of each object cost when called later
    => self.cost(Node1) < self.cost(Node2)
    """
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __lt__(self, other):
        return self.cost < other.cost


class AStarPlannerNode:
    def __init__(self):
        
        """
        subscribes to previously created map which is launched prior to the script
        
        Map and planner parameters are initialized as to get the metadata in the yaml file
        
        goal position is set at desired location from 2d nav goal in rviz
        """
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_publisher = rospy.Publisher('/planned_path', Path, queue_size=10)

       
        self.resolution = None
        self.origin_x, self.origin_y = None, None
        self.x_width, self.y_width = None, None
        self.obstacle_map = None       
        self.start = (0, 0)  
        self.goal = (32, 101)
        self.path_msg = None



    def map_callback(self, msg):
        
        """
        Gets parameters from pgm and yaml file
        
        As the map is saved as a 1D array in [x,y,x,y] format, changed
        to 2D map to index coordinates
        """
        
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.x_width = msg.info.width
        self.y_width = msg.info.height
        self.obstacle_map = np.zeros((self.y_width, self.x_width), dtype=bool)
        
        data_array = np.array(msg.data).reshape(self.y_width, self.x_width)
        self.obstacle_map = data_array >= 100

        self.plan_path()
        


    def plan_path(self):
        
        """
        start and goal positions set to the grid position, 
        Set to be updated based on the map origin positon but is redundant in this case as origin is set to 0,0
        
        call A* to determine path before iterate over x,y positions and post in map
        Iterate over path and publish pose to map
        
        """
        start_x, start_y = self.start
        goal_x, goal_y = self.goal
    
        start_x = round((start_x - self.origin_x) / self.resolution)
        start_y = round((start_y - self.origin_y) / self.resolution)
        goal_x = round((goal_x - self.origin_x) / self.resolution)
        goal_y = round((goal_y - self.origin_y) / self.resolution)
    
       
        pos_map_x, pos_map_y = self.graph_search(start_x, start_y, goal_x, goal_y)
    
    
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        pos_x_y = zip(pos_map_x, pos_map_y)
        
        for x, y in pos_x_y:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * self.resolution + self.origin_x
            pose.pose.position.y = y * self.resolution + self.origin_y
            pose.pose.position.z = 0.0
            self.path_msg.poses.append(pose)



    def graph_search(self, start_x, start_y, goal_x, goal_y):
        
        """ 
        Closed set a dict of the already processed nodes with their coordinates
        
        heappush adds a start node tuple to the open set heap where initial cost+heuristic is 0 (heappush(heap, (cost, node)))
        
        motion model is created with a loop iterating over to fill the open_set with all movement/cost
        
        while open set
           pop next node in heap with smallest cost+heuristic ()
           continue if current node is in closed set and add current to closed set if not
           Add new nodes to heap based on motion model

        """
        
        start_node = Node(start_x, start_y, 0.0, -1)
        goal_node = Node(goal_x, goal_y, 0.0, -1)

        open_set = []
        closed_set = {}
        
        heapq.heappush(open_set, (0, start_node))

        motion = self.motion()
        

        while open_set:
            node_heap, current_node = heapq.heappop(open_set)
            current_node_cords = (current_node.x, current_node.y)

            if current_node_cords in closed_set:
                continue

            if current_node.x == goal_x and current_node.y == goal_y:
                rospy.loginfo("Goal reached!")
                return self.calculate_final_path(current_node, closed_set)
            
            closed_set[current_node_cords] = current_node
           
            for m in motion:
                new_x = current_node.x + m[0]
                new_y = current_node.y + m[1]
                new_cost = current_node.cost + m[2]
                new_node = Node(new_x, new_y, new_cost, current_node_cords)

                if not self.verify_node(new_node):
                    continue

                if (new_x, new_y) not in closed_set:
                    heapq.heappush(open_set, (new_node.cost + self.heuristic(new_node, goal_node), new_node))


    def verify_node(self, node):
        """
        ennsure node is in map and not occupied
        """
        if node.x < 0 or node.y < 0 or node.x >= self.x_width or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.y, node.x]:
            return False
        return True


    def heuristic(self, new_node, goal_node):
        return np.sqrt((new_node.x - goal_node.x) ** 2 + (new_node.y - goal_node.y) ** 2)


    def calculate_final_path(self, goal_node, closed_set):
        """
        Iterate over path xy from closed set until start index
        """

        
        pos_map_x, pos_map_y = [goal_node.x], [goal_node.y]
        parent = goal_node.parent_index
        while parent != -1:
            node = closed_set[parent]
            pos_map_x.append(node.x)
            pos_map_y.append(node.y)
            parent = node.parent_index
        return pos_map_x[::-1], pos_map_y[::-1]


    def motion(self):
        """
        each tuple index 0 and 1 xy-movement
        modify index 2 for turning cost
        """

        return [
            (1, 0, 1),
            (0, 1, 1),
            (-1, 0, 1),
            (0, -1, 1),
            (-1, -1, 2),
            (-1, 1, 2),
            (1, -1, 2),
            (1, 1, 2),
        ]


    def path_pub(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.path_msg:
                self.path_publisher.publish(self.path_msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("a_star_path_planner")
    node = AStarPlannerNode()
    node.path_pub()
