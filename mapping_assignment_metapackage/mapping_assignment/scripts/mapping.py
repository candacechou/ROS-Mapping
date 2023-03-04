#!/usr/bin/env python3
"""
    # {Candace CHou}
    # {chchou@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs
from numpy.linalg import inv
# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):



        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        
    
        #step_scan = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        pose_x_laser = 0
        pose_y_laser = 0
        pose_z_laser = 0
        temp_angle = 0
        indice_x = 0
        indice_y = 0
        max_x = 0
        max_y = 0
        min_x = 0
        min_y = 0
        rectangle = {}
        robot_indice_x = int((pose.pose.position.x - origin.position.x)/resolution)
        robot_indice_y = int((pose.pose.position.y - origin.position.y)/resolution)
        for i in range(len(scan.ranges)):
            temp_angle = scan.angle_min + i*scan.angle_increment
            if scan.ranges[i] <= scan.range_min or scan.ranges[i] >= scan.range_max:
                continue 
            else:
            ### 1. Convert the laser scan ranges and bearings to coordinates in the laser frame
                pose_x_laser = pose.pose.position.x + scan.ranges[i] * cos(temp_angle + robot_yaw)
                pose_y_laser = pose.pose.position.y + scan.ranges[i] * sin(temp_angle + robot_yaw)
                indice_x = int((pose_x_laser-origin.position.x)/resolution)
                indice_y = int((pose_y_laser-origin.position.y)/resolution)
                clear_grid = self.raytrace((robot_indice_x,robot_indice_y),(indice_x,indice_y))
                for nodes in clear_grid:
                    if self.is_in_bounds(grid_map,nodes[0],nodes[1]):
                        self.add_to_map(grid_map,nodes[0],nodes[1],self.free_space)
                        rectangle[(nodes[0],nodes[1])] = self.free_space
                        if nodes[0] > max_x :
                            max_x = nodes[0]
                        elif nodes[0] < min_x:
                            min_x = nodes[0]
                        if nodes[1] > max_y:
                            max_y = nodes[1]
                        elif nodes[1] < min_y:
                            min_y = nodes[1]
                        else:
                            pass
        pose_x_laser = 0
        pose_y_laser = 0
        pose_z_laser = 0
        temp_angle = 0
        indice_x = 0
        indice_y = 0   
        for i in range(len(scan.ranges)):
            temp_angle = scan.angle_min + i * scan.angle_increment
            if scan.ranges[i] <= scan.range_min or scan.ranges[i] >= scan.range_max:
                continue    
            else:
            ### 1. Convert the laser scan ranges and bearings to coordinates in the laser frame
                pose_x_laser = pose.pose.position.x + scan.ranges[i] * cos(temp_angle + robot_yaw)
                pose_y_laser = pose.pose.position.y + scan.ranges[i] * sin(temp_angle + robot_yaw)
                #pose_z_laser = pose.pose.position.z 
                
            ### 2. Convert the coordinates to the map frame
                #Real = np.dot(M,np.transpose([pose_x_laser,pose_y_laser,pose_z_laser]))
                
            ### 3. Convert the coordinates to map indices (Hint: Use int(X) to convert from float to int, do not use round(X) 
                indice_x = int((pose_x_laser-origin.position.x)/resolution)
                indice_y = int((pose_y_laser-origin.position.y)/resolution)
                self.add_to_map(grid_map,indice_x,indice_y,self.occupied_space)
                rectangle[(indice_x,indice_y)] = self.occupied_space
                if indice_x > max_x :
                    max_x = indice_x
                elif indice_x < min_x:
                    min_x = indice_x
                if indice_y > max_y:
                    max_y = indice_y
                elif indice_y < min_y:
                    min_y = indice_y
                else:
                    pass
            ###C part: 1. Clear free space between the scan endpoints and the robot's position,
            #  using raytrace(self, start, end) function and self.free_space variable.
                


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y
        # Maximum x index - minimum x index + 1
        update.width = (max_x - min_x) + 1
        # Maximum y index - minimum y index + 1
        update.height = (max_y - min_y) + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        for y in range(min_y,min_y + update.height, 1):
            for x in range(min_x,min_x + update.width, 1):
                if (x,y) in rectangle:
                    update.data.append(rectangle[(x,y)])
                else:
                    update.data.append(self.unknown_space)

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):    
        h = grid_map.get_height()
        w = grid_map.get_width()
        
        for x_0 in range(w):
            for y_0 in range(h):
                if grid_map[x_0,y_0] == self.occupied_space:
                    temp_x = x_0 - self.radius
                    temp_y = y_0 - self.radius
                    for x in range(2 * self.radius+1):
                        for y in range(2 * self.radius+1):
                            distance = (temp_x + x - x_0)**2 +(temp_y + y - y_0)**2
                            if distance <= self.radius **2 and grid_map[temp_x + x,temp_y + y] != self.occupied_space and self.is_in_bounds(grid_map,temp_x + x,temp_y + y):
                                self.add_to_map(grid_map,temp_x + x,temp_y + y,self.c_space)
                            else:
                                pass

        # Return the inflated map
        return grid_map
