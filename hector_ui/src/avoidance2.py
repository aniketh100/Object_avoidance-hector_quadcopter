#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class HectorQuadObstacleAvoidance:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.max_speed = 0.5
        self.turn_speed = 0.5
        self.ascend_speed = 0.5
        self.ascend_time = 5.0

        self.state = "ASCENDING"
        self.obstacle_detected = False
        self.last_image_time = rospy.Time.now()
        self.image_timeout = rospy.Duration(1.0)

    def image_callback(self, data):
        self.last_image_time = rospy.Time.now()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.detect_obstacle(cv_image)
        cv2.imshow("Drone View", cv_image)
        cv2.waitKey(1)

    def detect_obstacle(self, image):
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # Combine the masks
        mask = mask1 + mask2

        # Calculate the percentage of red pixels
        red_percentage = (cv2.countNonZero(mask) / (image.shape[0] * image.shape[1])) * 100

        # If more than 10% of the image is red, consider it an obstacle
        self.obstacle_detected = red_percentage > 10
        rospy.loginfo(f"Red percentage: {red_percentage:.2f}%, Obstacle detected: {self.obstacle_detected}")

    def is_image_data_stale(self):
        return (rospy.Time.now() - self.last_image_time) > self.image_timeout

    def move_drone(self):
        cmd_vel = Twist()

        if self.state == "ASCENDING":
            cmd_vel.linear.z = self.ascend_speed
            rospy.loginfo("Ascending...")
        elif self.is_image_data_stale():
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            rospy.logwarn("Image data is stale. Stopping for safety.")
        elif self.obstacle_detected:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = self.turn_speed
            rospy.logwarn("Obstacle detected! Stopping and turning.")
        else:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = 0
            rospy.loginfo("No obstacle. Moving forward.")

        self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo(f"Published cmd_vel: linear.x={cmd_vel.linear.x}, linear.z={cmd_vel.linear.z}, angular.z={cmd_vel.angular.z}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.state == "ASCENDING" and (current_time - start_time).to_sec() > self.ascend_time:
                self.state = "EXPLORING"
                rospy.loginfo("Finished ascending. Now exploring.")
            
            self.move_drone()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hector_quad_obstacle_avoidance', anonymous=True)
    drone = HectorQuadObstacleAvoidance()
    try:
        drone.run()
    except rospy.ROSInterruptException:
        pass
