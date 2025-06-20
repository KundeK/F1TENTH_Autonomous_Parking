#!/usr/bin/env python3
import numpy as np
import math
from std_msgs.msg import String, Bool, Float32, Float64, Float32MultiArray
import rospy

class ArucoParkingPlanner:
    def __init__(self, distance_threshold=1.5, parking_distance=0.1, lookahead_distance=0.4, 
                 cruise_speed=0.3, parking_speed=0.1, stopping_distance=0.2):
        """
        distance_threshold: Distance threshold for switching between lookahead and parking points (meters)
        parking_distance: Distance from marker for parking point (meters)
        lookahead_distance: Distance from marker for lookahead point (meters)
        cruise_speed: Constant speed for normal driving (m/s)
        parking_speed: Reduced speed for parking maneuvers (m/s)
        stopping_distance: Distance threshold to stop the car (meters)
        """
        self.distance_threshold = distance_threshold
        self.parking_distance = parking_distance
        self.lookahead_distance = lookahead_distance
        self.cruise_speed = cruise_speed
        self.parking_speed = parking_speed
        self.stopping_distance = stopping_distance

        self.last_target = None
        self.status = "no_marker"
        self.steering_angle = 0.
        self.steering_gain = 0.4
        
        self.aruco_sub = rospy.Subscriber('/aruco/pose', Float32MultiArray, self.aruco_callback)
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

    
    def aruco_callback(self, msg):
        self.x, self.y, self.z = msg.data[0], msg.data[1], msg.data[2]
        self.roll, self.pitch, self.yaw = msg.data[3], msg.data[4], msg.data[5]
        # print(self.x)
        # print(self.y)
        # print(self.z)
        # print(self.roll)
        # print(self.pitch)
        # print(self.yaw)

        
    def calculate_2d_projected_points(self):
        # Returns: Tuple of (parking_point, lookahead_point) as 2D coordinates [x, y]
        # Convert yaw to radians
        yaw_rad = np.radians(self.yaw)
        
        # Create a unit vector and determine marker position
        perp_unit_vector = np.array([np.sin(yaw_rad), -np.cos(yaw_rad)])
        marker_position_2d = np.array([self.x, self.y])
        
        # Calculate parking and lookahead points
        parking_point = marker_position_2d + perp_unit_vector * self.parking_distance
        lookahead_point = marker_position_2d + perp_unit_vector * self.lookahead_distance

        # print(parking_point)
        # print(lookahead_point)
        return parking_point, lookahead_point
    

    def get_target_point(self):
        if self.x is None or self.y is None or self.yaw is None or self.y < 0:
            self.status = "no_marker"
            return {
                'status': self.status,
                'target_point': self.last_target,
                'distance_to_marker': None,
                'approach_angle_deg': None
            }
        
        # Calculate direct distance to marker and parking
        self.distance_to_marker = np.linalg.norm(np.array([self.x, self.y])) 
        
        # Calculate parking and lookahead point
        parking_point, lookahead_point = self.calculate_2d_projected_points()
        distance_to_parking_point = np.linalg.norm(parking_point - np.array([self.x, self.y]))
        
        # Determine target point and status based on distance
        # print(parking_point - np.array([self.x, self.y]))
        if self.distance_to_marker > self.distance_threshold:
            target_point = lookahead_point
            self.status = "approaching"
        elif self.distance_to_marker < self.stopping_distance:
            target_point = parking_point
            self.status = "stopped"
        else:
            target_point = parking_point
            self.status = "parking"
      
        self.last_target = target_point
        self.steering_angle = -np.arctan2(target_point[0], target_point[1]) * self.steering_gain
        
        return {
            'status': self.status,
            'target_point': target_point,
            'marker_position': [self.x, self.y],
            'parking_point': parking_point,
            'lookahead_point': lookahead_point,
            'distance_to_marker': self.distance_to_marker,
            'distance_to_parking_point': distance_to_parking_point,
            'yaw_deg': self.yaw,
            'steering_angle': self.steering_angle
        }
  