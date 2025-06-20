#!/usr/bin/env python3

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import time
import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Image, Joy

from arucoDetection import detect_and_estimate_pose
from aruco_parking_planner import ArucoParkingPlanner

from controllers import SpeedController

class ParallelParkSupport(object):
    
    def __init__(self):
        self.angle_min = -2.356194496154785
        self.angle_max =  2.356194496154785
        self.angle_increment = 0.004363323096185923
        self.angle_width = self.angle_max - self.angle_min
        self.num_lidar_points = round((self.angle_width / self.angle_increment) + 1)

        self.angles = np.linspace(self.angle_min, self.angle_max, self.num_lidar_points)

        self.epsilon = 0.3

        self.aruco_tvec = [None, None, None]
        self.aruco_rvec = None

        #ros items
        self.rate = rospy.Rate(80)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = 'f1tenth_control'
        #publishers
        self.aruco_pub = rospy.Publisher('/aruco/pose', Float32MultiArray, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.last_ctrl_pub = time.time()
        #subscribers
        self.lidar_raw_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.img_sub = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback)
        self.joy_sub = rospy.Subscriber('/vesc/joy', Joy, self.joy_callback)

        self.is_safe = True
        self.steering=0
        self.speed_thresh=1.0

        # Initializing the ArucoParkingPlanner
        self.parking_planner = ArucoParkingPlanner(
            distance_threshold=1.1,
            parking_distance=0.1,
            lookahead_distance=0.7,
            cruise_speed=1.2,
            parking_speed=0.8,
            stopping_distance=0.2
        )
        self.parking_enabled = True
        self.parking_completed = False
        self.has_stopped = False

        # self.speed_controller:SpeedController = SpeedController(kp=0.5, ki=0.3, kd=0.0, ke=0.3, target=0.0)
        self.speed_controller:SpeedController = SpeedController(kp=0.5, ki=0.35, kd=0.0, ke=0.3, target=0.0)


    def joy_callback(self, joy: Joy):
        self.steering = -joy.axes[2]


    def img_callback(self, img: Image):
        # img.data
        bridge = CvBridge()
        try:
            frame = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        camera_matrix = np.array([
            [606.0244751,   0,              325.15777588    ],
            [0,             605.73144531,   248.24261475    ],
            [0,             0,              1               ]
        ], dtype=np.float64)

        #check 
        dist_coeffs = np.zeros((5,1)) 

        result = detect_and_estimate_pose(frame, camera_matrix, dist_coeffs, marker_size = 0.127)
        if result is not None:
            self.aruco_rvec, self.aruco_tvec = result
            # print(f"Detected marker - Rvec: {self.aruco_rvec}° | Translation: {self.aruco_tvec}")

            msg = Float32MultiArray()
            msg.data = [0., 0., 0., 0., 0., 0.]
            msg.data[0] =  self.aruco_tvec[0] # lateral distance (right of car)
            msg.data[1] =  self.aruco_tvec[2] # longitudinal distance (front of car)
            msg.data[2] = -self.aruco_tvec[1] # vertical distance (up down of car)
            msg.data[3] =  self.aruco_rvec[2] # roll
            msg.data[4] =  self.aruco_rvec[0] # pitch
            msg.data[5] = -self.aruco_rvec[1] # yaw
            self.aruco_pub.publish(msg)
            # print(f"Detected marker - x: {msg.data[0]}, y: {msg.data[1]}, z: {msg.data[2]} \n        roll: {msg.data[3]}, pitch: {msg.data[4]}, yaw: {msg.data[5]}")
        else:
            msg = Float32MultiArray()
            msg.data = [-1., -1., -1., -1., -1., -1.]
            self.aruco_pub.publish(msg)
            # print("No marker detected")
    
    def ramer_douglas_peucker(self, points):
        if len(points) <= 2:
            return points
        
        points[:, 1] = points[:, 1]

        m = (points[-1][1] - points[0][1]) / (points[-1][0] - points[0][0])
        b = points[0][1] - m * points[0][0]

        point_line_distances = np.abs(points[1:-1][:, 1] - (m * points[1:-1][:, 0] + b))
        max_idx = np.argmax(point_line_distances) + 1
        max_dist = point_line_distances[max_idx - 1]

        if max_dist > self.epsilon:
            first_segment = self.ramer_douglas_peucker(points[:max_idx+1])
            second_segment = self.ramer_douglas_peucker(points[max_idx:])

            return np.vstack((first_segment[:-1], second_segment))
        else:
            return np.vstack((points[0], points[-1]))

    def lidar_callback(self, lidar_msg: LaserScan):
        ranges = np.array(lidar_msg.ranges[::-1])
        simplified = self.ramer_douglas_peucker(np.column_stack((np.arange(len(ranges)), ranges)))

        self.speed_thresh = self.speed_controller.feedback ** 2 / 15. + 0.3
        # print(self.speed_thresh)

        # prev_point = simplified[0]
        for point in simplified:
            if np.abs(self.angles[int(point[0])] - self.steering) < 1.5:
                self.is_safe = np.sin(self.angles[int(point[0])] - self.steering) * point[1] > 0.2 or point[1] > self.speed_thresh
                if not self.is_safe:
                    return

            # prev_point = point

    def start(self):
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            if self.aruco_rvec is not None and self.aruco_tvec is not None and self.parking_enabled:
                target_info = self.parking_planner.get_target_point()

                # FOR DEBUGGING
                # print(f"Status: {target_info['status']}")
                # print(f"Dist to marker: {target_info['distance_to_marker']}")
                # print(f"Lookahead Pt: {target_info['lookahead_point']}")
                # print(f"Parking Pt: {target_info['parking_point']}")
                # print(f"Target Pt: {target_info['target_point']}")
                # print(f"Yaw: {target_info['yaw_deg']}")
                # print(f"Steering: {target_info['steering_angle']}")

                if target_info['status'] == 'approaching':
                    self.speed_controller.set_target(self.parking_planner.cruise_speed)
                elif target_info['status'] == 'parking':
                    self.speed_controller.set_target(self.parking_planner.parking_speed)
                elif target_info['status'] == 'stopped' or target_info['status'] == 'no_marker' :
                    self.speed_controller.set_target(0.0)

                # self.speed_controller.update()
                speed = np.clip(self.speed_controller.output_power, -0.2, self.speed_controller.target * 0.8)
                steering = self.parking_planner.steering_angle

                if self.speed_controller.target == 0:
                    speed = 0.

                self.drive_msg.header.stamp = current_time
                self.drive_msg.drive.steering_angle = steering

                if time.time() - self.last_ctrl_pub > 0.5 or not self.is_safe:# or self.speed_controller.target == 0.0:
                    self.last_ctrl_pub = time.time()
                    self.drive_msg.drive.speed = speed if self.is_safe else 0.0
                self.ctrl_pub.publish(self.drive_msg)

                # if self.speed_controller.target > 0.4:
                #     rospy.loginfo_throttle(0.01, f"Status: {target_info['status']}, Power: {speed:.2f}, Target Spd: {self.speed_controller.target:.2f}, Spd: {self.speed_controller.feedback:.2f}, Int: {self.speed_controller.integral:.2f}")
                # rospy.loginfo_throttle(1, f"Safety: {self.is_safe}")

                # if target_info['status'] == 'stopped':
                #     speed = 0.0
                #     self.parking_completed = True
                #     rospy.loginfo("Parking completed")

            else:
                self.drive_msg.header.stamp = current_time
                self.drive_msg.drive.speed = 0.0
                self.drive_msg.drive.steering_angle = 0.0
                self.ctrl_pub.publish(self.drive_msg)
            
            self.rate.sleep()
        print("ROS node is shutdown!")


def state_pub():
    rospy.init_node('parallel_park_controller', anonymous=True)

    pparker = ParallelParkSupport()
    try:
        pparker.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    state_pub()

