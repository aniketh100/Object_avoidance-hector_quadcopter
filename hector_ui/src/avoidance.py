#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class HectorQuadObstacleAvoidance:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/front_cam/camera/image', Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.obstacle_detected = False
        self.min_distance = 1.0  # Minimum distance to obstacle (in meters)
        self.linear_speed = 0.5  # Forward speed
        self.angular_speed = 0.5  # Turning speed
        self.vertical_speed = 0.5  # Upward speed
        self.ascend_duration = 5.0  # Time to ascend in seconds
        self.state = "ASCENDING"  # Initial state
        self.lidar_data = None

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_obstacle_camera(cv_image)
        except CvBridgeError as e:
            print(e)

    def lidar_callback(self, data):
        self.lidar_data = data
        self.detect_obstacle_lidar()

    def detect_obstacle_camera(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detect edges using Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Count white pixels (edges)
        white_pixel_count = np.sum(edges == 255)
        
        # If the number of edge pixels is above a threshold, consider it an obstacle
        self.obstacle_detected = white_pixel_count > 10000  # Adjust this threshold as needed

    def detect_obstacle_lidar(self):
        if self.lidar_data is None:
            return

        # Check if there's an obstacle within min_distance in any direction
        for distance in self.lidar_data.ranges:
            if 0 < distance < self.min_distance:
                self.obstacle_detected = True
                return
        self.obstacle_detected = False

    def move_drone(self):
        cmd_vel = Twist()
        
        if self.state == "ASCENDING":
            cmd_vel.linear.z = self.vertical_speed
        elif self.state == "EXPLORING":
            if self.obstacle_detected:
                # Turn to avoid obstacle
                cmd_vel.angular.z = self.angular_speed
                cmd_vel.linear.x = 0  # Stop moving forward when turning
            else:
                # Move forward
                cmd_vel.linear.x = self.linear_speed

        self.cmd_vel_pub.publish(cmd_vel)

    def ascend(self):
        rospy.loginfo("Ascending...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz

        while (rospy.Time.now() - start_time).to_sec() < self.ascend_duration:
            cmd_vel = Twist()
            cmd_vel.linear.z = self.vertical_speed
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

        self.state = "EXPLORING"
        rospy.loginfo("Finished ascending. Now exploring.")

    def run(self):
        self.ascend()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.move_drone()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hector_quad_obstacle_avoidance', anonymous=True)
    drone = HectorQuadObstacleAvoidance()
    try:
        drone.run()
    except rospy.ROSInterruptException:
        pass
