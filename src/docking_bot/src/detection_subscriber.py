#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback,
            10)
        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.target_detected = False
        self.target_bbox = None
        self.min_distance = float('inf')  

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        if self.target_detected and self.target_bbox:
            top_left = (int(self.target_bbox[0].x), int(self.target_bbox[0].y))
            bottom_right = (int(self.target_bbox[2].x), int(self.target_bbox[2].y))
            cv2.rectangle(current_frame, top_left, bottom_right, (0, 255, 0), 1)
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_bbox = msg.detections[0].corners
        else:
            self.target_detected = False
        self.move_towards_tag()

    def lidar_callback(self, msg):
        
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')
       

    def move_towards_tag(self):
        twist = Twist()
        image_width = 800  
        
        # PID controller parameters
        angular_gain = 0.002  
        angular_dead_zone = 5  
        linear_speed_base = 0.11  
        linear_speed_gain = 0.003  
        min_linear_speed = 0.010  

        # Allowable tolerances
        distance_tolerance = 0.03  
        angular_tolerance = 15 

        if self.target_detected:
           
            centre_x = (self.target_bbox[0].x + self.target_bbox[2].x) / 2
            
            
            error_x = centre_x - (image_width / 2)
            
            
            if self.min_distance <= 0.25:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Stopping: Obstacle within 25 cm.")
                
               
                if abs(error_x) > angular_dead_zone:
                  
                    twist.angular.z = -angular_gain * error_x
                    self.get_logger().info(f"Aligning with tag after stop: error_x={error_x}, angular.z={twist.angular.z}")
                else:
                   
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0 
                    self.get_logger().info("Aligned with tag, waiting to move.")
            
            else:
               
                if abs(error_x) <= angular_tolerance and abs(self.min_distance - 0.25) <= distance_tolerance:
                   
                    twist.linear.x = max(min_linear_speed, linear_speed_base + linear_speed_gain * (self.min_distance - 0.25))
                    twist.angular.z = 0.0 
                    self.get_logger().info(f"Moving towards tag: linear.x={twist.linear.x}")
                else:
                   
                    if abs(error_x) > angular_dead_zone:
                        twist.angular.z = -angular_gain * error_x
                        twist.linear.x = 0.0  
                        self.get_logger().info(f"Aligning with tag: error_x={error_x}, angular.z={twist.angular.z}")
                    else:
                        
                        twist.linear.x = max(min_linear_speed, linear_speed_base + linear_speed_gain * (self.min_distance - 0.25))
                        twist.angular.z = 0.0  
                        self.get_logger().info(f"Moving towards tag: linear.x={twist.linear.x}")
        else:
            
            twist.linear.x = 0.0
            twist.angular.z = 0.3 
            self.get_logger().info("No tag detected. Rotating to search for tag.")
    
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

main()
