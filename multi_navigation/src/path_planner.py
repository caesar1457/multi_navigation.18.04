#!/usr/bin/env python3

import rospy
import math
import cv2 as cv  # OpenCV2
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import copy
import random
import matplotlib.cm


class Node:
    def __init__(self, x, y, idx):
        # Index of the node in the graph
        self.idx = idx

        # Position of node
        self.x = x
        self.y = y

        # Neighbouring edges
        self.neighbours = []
        self.neighbour_costs = []

        # Search parameters
        self.cost = 9999999  # A large number
        self.parent_node = None  # Invalid parent

    def distance_to(self, other_node):
        return math.sqrt((self.x - other_node.x) ** 2 + (self.y - other_node.y) ** 2)

    def is_connected(self, img, other_node):
        p1 = [self.x, self.y]
        p2 = [other_node.x, other_node.y]
        return not is_occluded(img, p1, p2)


class Graph:
    def __init__(self, map,id):

        self.map_ = map

        self.nodes_ = []

        self.grid_step_size_ = rospy.get_param("~grid_step_size")  # Grid spacing
        self.groups_ = None

        self.path_pub_ = {}
        self.path_smooth_pub_ = {}
        # Publishers
        for i in range(1, id + 1):
            self.path_pub_[i] = rospy.Publisher(f'/robot_{i}/path_planner/plan', Path, queue_size=1)
            self.path_smooth_pub_[i] = rospy.Publisher(f'/robot_{i}/path_planner/plan_smooth', Path, queue_size=1)

        # Visualisation Marker (you can ignore this)
        self.marker_nodes_ = Marker()
        self.marker_nodes_.header.frame_id = "map"
        self.marker_nodes_.ns = "nodes"
        self.marker_nodes_.id = 0
        self.marker_nodes_.type = Marker.POINTS
        self.marker_nodes_.action = Marker.ADD
        self.marker_nodes_.pose.position.x = 0.0
        self.marker_nodes_.pose.position.y = 0.0
        self.marker_nodes_.pose.position.z = 0.0
        self.marker_nodes_.pose.orientation.x = 0.0
        self.marker_nodes_.pose.orientation.y = 0.0
        self.marker_nodes_.pose.orientation.z = 0.0
        self.marker_nodes_.pose.orientation.w = 1.0
        self.marker_nodes_.scale.x = .03
        self.marker_nodes_.scale.y = .03
        self.marker_nodes_.scale.z = .03
        self.marker_nodes_.color.a = 1.0
        self.marker_nodes_.color.r = 1.0
        self.marker_nodes_.color.g = 0.2
        self.marker_nodes_.color.b = 0.2

        self.marker_start_ = Marker()
        self.marker_start_.header.frame_id = "map"
        self.marker_start_.ns = "start"
        self.marker_start_.id = 0
        self.marker_start_.type = Marker.POINTS
        self.marker_start_.action = Marker.ADD
        self.marker_start_.pose.position.x = 0.0
        self.marker_start_.pose.position.y = 0.0
        self.marker_start_.pose.position.z = 0.0
        self.marker_start_.pose.orientation.x = 0.0
        self.marker_start_.pose.orientation.y = 0.0
        self.marker_start_.pose.orientation.z = 0.0
        self.marker_start_.pose.orientation.w = 1.0
        self.marker_start_.scale.x = .08
        self.marker_start_.scale.y = .08
        self.marker_start_.scale.z = .08
        self.marker_start_.color.a = 1.0
        self.marker_start_.color.r = 1.0
        self.marker_start_.color.g = 1.0
        self.marker_start_.color.b = 0.2

        self.marker_visited_ = Marker()
        self.marker_visited_.header.frame_id = "map"
        self.marker_visited_.ns = "visited"
        self.marker_visited_.id = 0
        self.marker_visited_.type = Marker.POINTS
        self.marker_visited_.action = Marker.ADD
        self.marker_visited_.pose.position.x = 0.0
        self.marker_visited_.pose.position.y = 0.0
        self.marker_visited_.pose.position.z = 0.0
        self.marker_visited_.pose.orientation.x = 0.0
        self.marker_visited_.pose.orientation.y = 0.0
        self.marker_visited_.pose.orientation.z = 0.0
        self.marker_visited_.pose.orientation.w = 1.0
        self.marker_visited_.scale.x = .05
        self.marker_visited_.scale.y = .05
        self.marker_visited_.scale.z = .05
        self.marker_visited_.color.a = 1.0
        self.marker_visited_.color.r = 0.2
        self.marker_visited_.color.g = 0.2
        self.marker_visited_.color.b = 1.0

        self.marker_unvisited_ = Marker()
        self.marker_unvisited_.header.frame_id = "map"
        self.marker_unvisited_.ns = "unvisited"
        self.marker_unvisited_.id = 0
        self.marker_unvisited_.type = Marker.POINTS
        self.marker_unvisited_.action = Marker.ADD
        self.marker_unvisited_.pose.position.x = 0.0
        self.marker_unvisited_.pose.position.y = 0.0
        self.marker_unvisited_.pose.position.z = 0.0
        self.marker_unvisited_.pose.orientation.x = 0.0
        self.marker_unvisited_.pose.orientation.y = 0.0
        self.marker_unvisited_.pose.orientation.z = 0.0
        self.marker_unvisited_.pose.orientation.w = 1.0
        self.marker_unvisited_.scale.x = .06
        self.marker_unvisited_.scale.y = .06
        self.marker_unvisited_.scale.z = .06
        self.marker_unvisited_.color.a = 1.0
        self.marker_unvisited_.color.r = 0.3
        self.marker_unvisited_.color.g = 1.0
        self.marker_unvisited_.color.b = 0.3

        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.pose.orientation.x = 0.0
        self.marker_edges_.pose.orientation.y = 0.0
        self.marker_edges_.pose.orientation.z = 0.0
        self.marker_edges_.pose.orientation.w = 1.0
        self.marker_edges_.scale.x = 0.008
        self.marker_edges_.scale.y = 0.008
        self.marker_edges_.scale.z = 0.008
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4

        self.marker_pub_ = rospy.Publisher('marker', Marker, queue_size=10)

        # Create grid
        self.create_grid()

        self.visualise_graph()

    def create_grid(self):

        # Create nodes
        idx = 0
        for x in range(self.map_.min_x_, self.map_.max_x_ - 1, self.grid_step_size_):
            for y in range(self.map_.min_y_, self.map_.max_y_ - 1, self.grid_step_size_):

                if rospy.is_shutdown():
                    return

                # Check if it is occupied
                occupied = self.map_.is_occupied(x, y)

                # Create the node
                if not occupied:
                    self.nodes_.append(Node(x, y, idx))
                    idx = idx + 1

        # Create edges
        count = 0
        # distance_threshold = math.sqrt(2*(self.grid_step_size_*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
        distance_threshold = self.grid_step_size_ * 1.01  # only 4 connected
        for node_i in self.nodes_:
            count = count + 1
            print(count, "of", len(self.nodes_))
            if rospy.is_shutdown():
                return

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.image_, node_j):
                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(distance)

    def get_closest_node(self, xy):
        # input: xy is a point in the form of an array, such that x=xy[0] and y=xy[1].
        # output: return the index of the node in self.nodes_ that has the lowest Euclidean distance to the point xy.

        best_dist = 999999999
        best_index = None

        for i in range(len(self.nodes_)):

            # Calculate the Euclidean distance from the point to the current node
            dist = math.sqrt((self.nodes_[i].x - xy[0]) ** 2 + (self.nodes_[i].y - xy[1]) ** 2)

            # Check if this is the closest node we've found so far
            if dist < best_dist:
                best_dist = dist
                best_index = i

        return best_index

    def visualise_graph(self):
        # Create and publish visualisation markers for the graph
        print("Visualizing graph...")

        rospy.sleep(0.5)

        if self.groups_ == None:
            self.marker_nodes_.points = []
            for node_i in self.nodes_:
                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(p[0], p[1], 0.01)
                self.marker_nodes_.points.append(point)
            self.marker_pub_.publish(self.marker_nodes_)
        else:
            # Plot each group with a different colour
            cmap = matplotlib.cm.get_cmap('Set1')
            colors = cmap.colors[0:-2]

            # print(self.groups_)

            self.marker_nodes_.points = []
            self.marker_nodes_.colors = []

            if rospy.get_param("~show_connectivity"):
                self.marker_nodes_.scale.x = .06
                self.marker_nodes_.scale.y = .06
                self.marker_nodes_.scale.z = .06

            for node_idx in range(len(self.nodes_)):
                # print("node_idx", node_idx)
                node_i = self.nodes_[node_idx]
                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(p[0], p[1], 0.01)
                self.marker_nodes_.points.append(point)

                # Set a colour
                group_id = self.groups_[node_idx]
                c = colors[group_id % len(colors)]
                col = ColorRGBA(c[0], c[1], c[2], 1.0)
                self.marker_nodes_.colors.append(col)
            self.marker_pub_.publish(self.marker_nodes_)

        rospy.sleep(0.5)

        self.marker_edges_.points = []
        for node_i in self.nodes_:
            for node_j in node_i.neighbours:
                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(p[0], p[1], 0)
                self.marker_edges_.points.append(point)
                p = self.map_.pixel_to_world(node_j.x, node_j.y)
                point = Point(p[0], p[1], 0)
                self.marker_edges_.points.append(point)
        self.marker_pub_.publish(self.marker_edges_)

        rospy.sleep(0.5)

    def visualise_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        # Visualise the nodes with these node indexes
        self.marker_visited_.points = []
        for i in visited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(p[0], p[1], 0.05)
            self.marker_visited_.points.append(point)
        self.marker_pub_.publish(self.marker_visited_)

        self.marker_unvisited_.points = []
        for i in unvisited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(p[0], p[1], 0.05)
            self.marker_unvisited_.points.append(point)
        self.marker_pub_.publish(self.marker_unvisited_)

        self.marker_start_.points = []
        node_i = self.nodes_[start_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(p[0], p[1], 0.05)
        self.marker_start_.points.append(point)

        node_i = self.nodes_[goal_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(p[0], p[1], 0.05)
        self.marker_start_.points.append(point)
        self.marker_pub_.publish(self.marker_start_)

    def visualise_path(self, path, id):
        msg = Path()
        msg.header.frame_id = 'map'
        for node in path:
            p = self.map_.pixel_to_world(node.x, node.y)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.1
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        self.path_pub_[id].publish(msg)

    def visualise_path_smooth(self, path, id):
        msg = Path()
        msg.header.frame_id = 'map'
        for node in path:
            p = self.map_.pixel_to_world(node.x, node.y)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.12
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        self.path_smooth_pub_[id].publish(msg)


class Map:
    def __init__(self):

        # Extract the image from a file
        filename = rospy.get_param('~filename')
        self.image_ = cv.imread(filename, cv.COLOR_BGR2GRAY)

        shape = self.image_.shape
        self.min_x_ = 0
        self.min_y_ = 0
        self.max_x_ = shape[0]
        self.max_y_ = shape[1]

        if len(shape) == 3:
            self.image_ = self.image_[:, :, 0]

        # Rviz subscriber
        self.rviz_goal_sub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback,
                                               queue_size=1)
        self.rviz_goal = None

    def pixel_to_world(self, x, y):
        resolution = 0.01
        return [y * resolution, (self.max_x_ - x) * resolution]

    def world_to_pixel(self, x, y):
        resolution = 0.01
        return [self.max_x_ - (y / resolution), x / resolution]

    def rviz_goal_callback(self, msg):
        goal = self.world_to_pixel(msg.pose.position.x, msg.pose.position.y)
        self.rviz_goal = goal  # Save it into global variable
        print("New goal received from rviz!")
        print(self.rviz_goal)

    def is_occupied(self, x, y):

        shape = self.image_.shape

        # Out of bounds
        if x < 0 or x >= shape[0] or y < 0 or y >= shape[1]:
            return True

        if self.image_[x, y] > 235:
            return False
        else:
            return True


def is_occluded(img, p1, p2, threshold=235):
    # Draws a line from p1 to p2
    # Stops at the first pixel that is a "hit", i.e. above the threshold
    # Returns the pixel coordinates for the first hit

    # Extract the vector
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])

    step = 1.0

    dx = x2 - x1
    dy = y2 - y1
    l = math.sqrt(dx ** 2. + dy ** 2.)
    if l == 0:
        return False
    dx = dx / l
    dy = dy / l

    max_steps = int(l / step)

    for i in range(max_steps):

        # Get the next pixel
        x = int(round(x1 + dx * i))
        y = int(round(y1 + dy * i))

        # Check if it's outside the image
        if x < 0 or x >= img.shape[0] or y < 0 or y >= img.shape[1]:
            return False

        # Check for "hit"
        if img[x, y] <= threshold:
            return True

    # No hits found
    return False


class GraphSearch:
    def __init__(self, graph, id, start_xy=None, goal_xy=None):
        self.graph_ = graph
        self.id = id

        self.heuristic_weight_ = rospy.get_param("~heuristic_weight")

        if start_xy == None or goal_xy == None:
            # Don't do a search
            print("No start or goal specified, not searching.")
            pass
        else:

            self.start_idx_ = self.graph_.get_closest_node(start_xy)
            self.goal_idx_ = self.graph_.get_closest_node(goal_xy)

            self.search(self.id, self.start_idx_, self.goal_idx_)

            self.path_ = self.generate_path(self.goal_idx_)
            self.graph_.visualise_path(self.path_, id)

    def search(self,id, start_idx, goal_idx):

        # Set all parents and costs to zero
        for n in self.graph_.nodes_:
            n.cost = 9999999  # a large number
            n.parent_node = None  # invalid to begin with

        # Setup sets.  These should contain indices (i.e. numbers) into the self.graph_.nodes_ array
        unvisited_set = []
        visited_set = []

        # Add start node to unvisited set
        unvisited_set.append(start_idx)
        self.graph_.nodes_[start_idx].cost = 0

        # Loop until solution found or graph is disconnected
        while len(unvisited_set) > 0:

            # Select a node with the minimum cost from the unvisited set
            node_idx_idx = self.get_minimum_cost_node(unvisited_set)
            node_idx = unvisited_set[node_idx_idx]
            node = self.graph_.nodes_[node_idx]

            # Move the current node from the unvisited set to the visited set
            visited_set.append(node_idx)
            if node_idx in unvisited_set:
                unvisited_set.remove(node_idx)

            # Termination criteria: If the current node is the goal, we have found a solution
            if node_idx == goal_idx:
                rospy.loginfo("Goal found!")
                return
            # For each neighbour of the node
            for neighbour_idx in range(len(self.graph_.nodes_[node_idx].neighbours)):
                # For convenience, extract the neighbour and the edge cost from the arrays
                neighbour = self.graph_.nodes_[node_idx].neighbours[neighbour_idx]
                neighbour_cost = self.graph_.nodes_[node_idx].neighbour_costs[neighbour_idx]
                # Check if neighbours is already in visited
                if neighbour.idx in visited_set:

                    # Do nothing
                    pass
                else:

                    # Compute the total cost for this neighbour node
                    heuristic_score = neighbour.distance_to(
                        self.graph_.nodes_[goal_idx])  # Euclidean distance to goal node
                    total_cost = node.cost + neighbour_cost + self.heuristic_weight_ * heuristic_score
                    # Check if neighbours is already in unvisited
                    if neighbour.idx in unvisited_set:

                        # If the cost is lower than the previous cost for this node, then update it to the new cost
                        # Also, update the parent pointer to point to the new parent
                        if total_cost < neighbour.cost:
                            neighbour.parent_node = node
                            neighbour.cost = total_cost

                    else:

                        # Add it to the unvisited set
                        unvisited_set.append(neighbour.idx)

                        # Initialise the cost and the parent pointer with the new total cost and current node as parent
                        neighbour.parent_node = node
                        neighbour.cost = total_cost

            # Visualise the current search status in RVIZ
            self.visualise_search(visited_set, unvisited_set, start_idx, goal_idx)
            # rospy.sleep(0.01)

    def get_minimum_cost_node(self, unvisited_set):
        # Find the vertex with the minimum cost

        # There's more efficient ways of doing this...
        min_cost = 99999999
        min_idx = None
        for idx in range(len(unvisited_set)):
            cost = self.graph_.nodes_[unvisited_set[idx]].cost
            if cost < min_cost:
                min_cost = cost
                min_idx = idx
        return min_idx

    def generate_path(self, goal_idx):
        # Generate the path by following the parents from the goal back to the start

        path = []

        current = self.graph_.nodes_[goal_idx]
        path.append(current)



        # Follow the parent nodes until we reach the start node
        while current.parent_node is not None:
            # Move to the parent node
            current = current.parent_node
            # Prepend the current node to the path
            path.append(current)

        # The path is currently from goal to start, so reverse it to be from start to goal
        path.reverse()

        return path

    def visualise_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        self.graph_.visualise_search(visited_set, unvisited_set, start_idx, goal_idx)


class PathSmoother():
    def __init__(self, graph, path, id):
        self.graph_ = graph
        self.path_ = self.smooth_path(path)
        self.graph_.visualise_path_smooth(self.path_, id)

    def smooth_path(self, path_nodes):
        # Convert into into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = node.x
            p.y = node.y
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        alpha = rospy.get_param("~alpha")
        beta = rospy.get_param("~beta")

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint

        epsilon = 0.0001

        while True:
            total_delta = 0.0

            for i in range(1, len(path_smooth) - 1):
                old_position = path_smooth[i]

                path_smooth[i].x = path_smooth[i].x - (alpha + 2 * beta) * path_smooth[i].x + alpha * path[i].x + beta * \
                                   path_smooth[i - 1].x + beta * path_smooth[i + 1].x
                path_smooth[i].y = path_smooth[i].y - (alpha + 2 * beta) * path_smooth[i].y + alpha * path[i].y + beta * \
                                   path_smooth[i - 1].y + beta * path_smooth[i + 1].y

                delta = (path_smooth[i].x - old_position.x) ** 2 + (path_smooth[i].y - old_position.y) ** 2
                total_delta += delta

                blocked_1 = is_occluded(self.graph_.map_.image_, [path_smooth[i - 1].x, path_smooth[i - 1].y],
                                        [path_smooth[i].x, path_smooth[i].y])
                blocked_2 = is_occluded(self.graph_.map_.image_, [path_smooth[i].x, path_smooth[i].y],
                                        [path_smooth[i + 1].x, path_smooth[i + 1].y])

                if blocked_1 or blocked_2:
                    path_smooth[i] = old_position

            if total_delta < epsilon:
                break

        return path_smooth


class Robot:
    def __init__(self, robot_id, initial_position, goal_position, initial_path=None):
        self.robot_id = robot_id
        self.position = initial_position
        self.goal_position = goal_position
        self.path = initial_path if initial_path else []

    def set_goal(self, goal_position):
        self.goal_position = goal_position

    def update_position(self, new_position):
        self.position = new_position

    def update_path(self, new_path):
        self.path = new_path

class MultiRobotSystem:
    def __init__(self):
        self.robots = {}  # Dictionary of robots, keyed by robot ID
        self.robot_ids = []  # List of robot IDs
        self.path_cost_matrix = []  # Cost matrix for each robot to each path

    def add_robot(self, robot_id, initial_position, goal_position, initial_path=None):
        if robot_id in self.robots:
            raise ValueError("Robot ID already exists")
        robot = Robot(robot_id, initial_position, goal_position, initial_path)
        self.robots[robot_id] = robot
        self.robot_ids.append(robot_id)

    def remove_robot(self, robot_id):
        if robot_id in self.robots:
            del self.robots[robot_id]
            self.robot_ids.remove(robot_id)
        else:
            raise ValueError("Robot ID does not exist")

    def update_robot_position(self, robot_id, new_position):
        self.validate_robot_exists(robot_id)
        self.robots[robot_id].update_position(new_position)

    def set_robot_goal(self, robot_id, goal_position):
        self.validate_robot_exists(robot_id)
        self.robots[robot_id].set_goal(goal_position)

    def update_robot_path(self, robot_id, new_path):
        self.validate_robot_exists(robot_id)
        self.robots[robot_id].update_path(new_path)

    def get_robot_info(self, robot_id):
        self.validate_robot_exists(robot_id)
        robot = self.robots[robot_id]
        return (robot.position, robot.goal_position, robot.path)

    def validate_robot_exists(self, robot_id):
        if robot_id not in self.robots:
            raise ValueError("Robot ID not found")

    def calculate_path_cost(self, path):
        cost = 0
        for i in range(len(path) - 1):
            # 计算相邻节点之间的距离
            cost += path[i].distance_to(path[i + 1])
        return cost

    def calculate_path_cost_matrix(self):
        """Calculate the cost matrix for each robot to each path they hold"""
         # Reset the path cost matrix
        self.path_cost_matrix = []
        # For each robot
        for rid in self.robot_ids:
           # Prepare the cost row, with the first column as the robot ID
            row = [rid]
             # Calculate the path cost from the current robot to the targets of all other robots
            for target_rid in self.robot_ids:
                if rid != target_rid:
                    path_cost = self.calculate_path_cost(self.robots[target_rid].path)
                    row.append(path_cost)
                else:
                    row.append(0)  # The cost from itself to itself is 0
            # Add the completed row to the cost matrix
            self.path_cost_matrix.append(row)


if __name__ == '__main__':
    # Create the ROS node
    rospy.init_node('path_planner')

    # Create a map from image
    map = Map()

    # Sleep added to give rviz a chance to wake up
    rospy.sleep(3.0)

    num_robots = rospy.get_param("/num_robots")

    # Create a graph from the map
    graph = Graph(map, num_robots)

    # cost_matrix_pub = rospy.Publisher('/path_planner/cost_matrix', String, queue_size=10)
    # task_sub = rospy.Subscriber('/task_assignment/tasks', String, task_callback)
    
    if not rospy.get_param("~show_connectivity"):

        # Create a multi-robot system   
        multi_robot_system = MultiRobotSystem()

        print(f"Number of robots: {num_robots}")

        # Add robots, start points, goal points and paths
        for i in range(1, num_robots + 1):
            startx = rospy.get_param(f'/robot_{i}/startx')
            starty = rospy.get_param(f'/robot_{i}/starty')
            goalx = rospy.get_param(f'/robot_{i}/goalx')
            goaly = rospy.get_param(f'/robot_{i}/goaly')   
            startxy = (startx, starty)
            goalxy = (goalx, goaly)
            graph_search = GraphSearch(graph, i, (startx, starty), (goalx, goaly))
            multi_robot_system.add_robot(f'robot_{i}', (startx, starty), (goalx, goaly), graph_search.path_)

            # Smooth the path
            PathSmoother(graph, graph_search.path_, i)

            print(f"Plan for robot {i} finished! Click a new goal in rviz 2D Nav Goal.")

        # Calculate and print the path cost matrix
        multi_robot_system.calculate_path_cost_matrix()
        print("Path Cost Matrix:")
        for row in multi_robot_system.path_cost_matrix:
            print(row)

        # Re-plan indefinitely when rviz goals received
        while not rospy.is_shutdown():

            for i in range(1, num_robots + 1):
                robot_id = f'robot_{i}'
                if map.rviz_goal is None:
                    # Do nothing, waiting for goal
                    rospy.sleep(0.01)
                else:

                    # Extract the goal
                    startx, starty = multi_robot_system.robots[robot_id].goal_position
                    goalx, goaly = map.rviz_goal
                    map.rviz_goal = None  # Clear it so a new goal can be set

                    # Do the graph search
                    graph_search = GraphSearch(graph, i, (startx, starty), (goalx, goaly))

                    # Update robot's position and path in the system
                    multi_robot_system.update_robot_position(robot_id, (startx, starty))
                    multi_robot_system.set_robot_goal(robot_id, (goalx, goaly))
                    multi_robot_system.robots[robot_id].update_path(graph_search.path_)

                    # Smooth the path
                    PathSmoother(graph, graph_search.path_, i)

                    print(f"Plan for robot {i} finished! Click a new goal in rviz 2D Nav Goal.")

    # Loop forever while processing callbacks
    rospy.spin()