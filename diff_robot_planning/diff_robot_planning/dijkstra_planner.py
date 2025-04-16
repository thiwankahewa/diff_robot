#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue

class GraphNode:    #custom class for graph node
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev
    
    def __lt__(self, other):
        return self.cost < other.cost   #comparison operator for custom type

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y    #comparison operator for custom type
    
    def __hash__(self):
        return hash((self.x, self.y))   #to used in set and dict
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])   #addition operator for custom type

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  #get the current transform of the robot

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   #setting the durability policy to transient local

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )  #current map
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )# get the goal pose 
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)  #all the nodes visited during the search

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)  #initialize the visited map

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width) #reset the visited map

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            ) #get the transform from map to base_footprint
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation  #convert the transform to pose of the robot current position

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):
        # Define possible movement directions in a grid
        explore_directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        # Priority queue with custom comparison for A* based on cost + heuristic
        pending_nodes = PriorityQueue()  
        visited_nodes = set() # Set to keep track of visited nodes

        start_node = self.world_to_grid(start)   #convert the pose to grid coordinates
        pending_nodes.put(start_node)   

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            # Goal found!
            if active_node == self.world_to_grid(goal):
                break
            
            # Explore neighbors
            for dir_x, dir_y in explore_directions:
                new_node: GraphNode = active_node + (dir_x, dir_y)
                
                if (new_node not in visited_nodes and self.pose_on_map(new_node) and 
                    self.map_.data[self.pose_to_cell(new_node)] == 0):  # Check if the cell is free
                    
                    new_node.cost = active_node.cost + 1
                    new_node.prev = active_node

                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map_.data[self.pose_to_cell(active_node)] = 10 # mark the cell as visited
            self.map_pub.publish(self.visited_map_)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grid_to_world(active_node)  #convert the grid coordinates to pose
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)  #add the pose to the path
            active_node = active_node.prev #get the previous node

        path.poses.reverse()  #reverse the path to get the correct order
        return path

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()