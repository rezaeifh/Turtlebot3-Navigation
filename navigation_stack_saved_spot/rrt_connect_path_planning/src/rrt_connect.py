#! /usr/bin/env python3

import rospy
from bresenham import bresenham
from math import atan2, cos, sin
from random import randrange as rand

# Node class
class Node:
    def __init__(self, coordinates, parent=None):
        # coordinates: list with [x,y] values of grid cell coordinates
        self.coordinates = coordinates
        # parent: Node object
        self.parent = parent


class Planning:
    def __init__(self, initial_position, target_position, width, height, map):
        
        self.initial_position = initial_position
        self.target_position = target_position
        self.width = width
        self.height = height
        self.map = map
        
    def rrt_connect(self):

        # Create the initial root node
        root_node_i = Node(self.initial_position)
        # Create target root node
        root_node_t = Node(self.target_position)
        # A list to keep all nodes started from initial position 
        nodes_i = [root_node_i]
        # A list to keep all nodes started from target position 
        nodes_t = [root_node_t]
        # Iteration counter
        iterations = 0
        # Iterations limit
        max_iterations = 10000
        # Incremental distance (for simplicity value is expressed in grid cells)
        max_branch_lenght = 5
        # Tolerance margin allowed around target position (value in grid cells)
        goal_tolerance = 5
        # A list to hold the output path from start to goal
        path = []

        while True:
            # Increment the number of iterations
            iterations += 1
            # Check if we have exeeded max iterations
            if iterations > max_iterations:
                rospy.logwarn("RRT-Connect: Max iterations exceeded")
                return path
            # initial tree
            # Generate a new random point anywhere in the map
            random_point_i = [rand(self.width), rand(self.height)]
            
            # Find the closest node
            closest_node_i = self.find_closest_node(random_point_i, nodes_i)
            
            # Create a new point at the max. branch distance towards the random point
            candidate_point_i = self.create_new_branch_point(closest_node_i.coordinates, random_point_i, max_branch_lenght)

            # Verifiy that the new branch is collision free
            if not self.collision_detected(closest_node_i.coordinates, candidate_point_i, self.map, self.width):
                # Create a new instance of a Node object and add it to the tree
                latest_node_i = Node(candidate_point_i, closest_node_i)
                nodes_i.append(latest_node_i)
		
                self.test_i = self.test_goal(latest_node_i.coordinates, nodes_t, goal_tolerance)
                # Check if the goal has been reached
                if self.test_i[0]:
                    rospy.loginfo('RRT-Connect: Goal reached')
                    self.end_t = 0;
                    self.end_i = 1;
                    self.last_t = self.test_i[1]
                    break
            
            # target tree
            # Generate a new random point anywhere in the map
            random_point_t = [rand(self.width), rand(self.height)]
            
            # Find the closest node
            closest_node_t = self.find_closest_node(random_point_t, nodes_t)
            
            # Create a new point at the max. branch distance towards the random point
            candidate_point_t = self.create_new_branch_point(closest_node_t.coordinates, random_point_t, max_branch_lenght)

            # Verifiy that the new branch is collision free
            if not self.collision_detected(closest_node_t.coordinates, candidate_point_t, self.map, self.width):
                # Create a new instance of a Node object and add it to the tree
                latest_node_t = Node(candidate_point_t, closest_node_t)
                nodes_t.append(latest_node_t)
		
                self.test_t = self.test_goal(latest_node_t.coordinates, nodes_i, goal_tolerance)
                # Check if the goal has been reached
                if self.test_t[0]:
                    rospy.loginfo('RRT-Connect: Goal reached')
                    self.end_t = 1;
                    self.end_i = 0;
                    self.last_i = self.test_t[1]
                    break

        rospy.loginfo('RRT-Connect: Path search ended')

        # Reconstruct path by working backwards from target
        if self.end_t == 1 :
            node = latest_node_t
            while node.parent:
                path.append(node.coordinates)
                node = node.parent
            path.append(node.coordinates)
            # Reverse list
            path.reverse()
            node = nodes_i[self.last_i]
            while node.parent:
                path.append(node.coordinates)
                node = node.parent
            path.append(node.coordinates)
            # Reverse list
            path.reverse()
        
        elif self.end_t == 0 :
            node = nodes_t[self.last_t]
            while node.parent:
                path.append(node.coordinates)
                node = node.parent
            path.append(node.coordinates)
            # Reverse list
            path.reverse()
            node = latest_node_i
            while node.parent:
                path.append(node.coordinates)
                node = node.parent
            path.append(node.coordinates)
            # Reverse list
            path.reverse()
        
        
        rospy.loginfo('RRT-Connect: Done reconstructing path')

        return path
    
    def calculate_distance(self, p1, p2):

        x = p2[0] - p1[0]
        y = p2[1] - p1[1]
        return (x ** 2 + y ** 2)**0.5

    def calculate_angle(self, p1, p2):

        x = p2[0] - p1[0]
        y = p2[1] - p1[1]
        return atan2(y, x)

    def collision_detected(self, p1, p2, map, map_width):

        # Compute cells covered by the line p1-p2 using the Bresenham ray tracing algorithm
        covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
        # Check if any of the cells is an obstacle cell
        for cell in covered_cells:
            # Access an element in a 1D-array (map) providing index = x + map_width*y
            if map[cell[0]+map_width*cell[1]]:
                # Detects a collision if map has a 1
                return True
        # No collision
        return False

    def find_closest_node(self, random_pt, node_list):

        nearest_distance = float('inf')
        for n in node_list:
            current_distance = self.calculate_distance(random_pt, n.coordinates)
            if current_distance < nearest_distance:
                nearest_node = n
                nearest_distance = current_distance
        return nearest_node

    def create_new_branch_point(self, p1, p2, max_distance):

        new_point = list(p1)
        d = self.calculate_distance(new_point, p2)
        theta = self.calculate_angle(new_point, p2)

        # if distance to closest node is less than the maximum branch lenght
        if max_distance > d:
            max_distance = d

        new_point[0] += int(max_distance * cos(theta))
        new_point[1] += int(max_distance * sin(theta))

        return new_point

    def test_goal(self, p1, node_list, tolerance):

        for i in range(len(node_list)):
            distance = self.calculate_distance(p1, node_list[i].coordinates)
            
            if ((tolerance > distance) and not self.collision_detected(p1, node_list[i].coordinates, self.map, self.width)):
                return [True, i]
            
        return [False, 0]
