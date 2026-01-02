#!/usr/bin/env python3
"""
Camera Test Script
Tests camera feed and saves sample images to verify camera is working.
Shows what the camera is actually seeing.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os


class CameraTest:
    def __init__(self):
        rospy.init_node('camera_test', anonymous=True)
        
        self.bridge = CvBridge()
        self.camera_info_received = False
        self.image_count = 0
        self.images_saved = 0
        
        # Subscribe to camera topics
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("CAMERA TEST NODE STARTED")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Waiting for camera data...")
        
    def camera_info_callback(self, msg):
        """Receive and display camera info once."""
        if not self.camera_info_received:
            self.camera_info_received = True
            rospy.loginfo("=" * 60)
            rospy.loginfo("CAMERA INFO RECEIVED:")
            rospy.loginfo(f"  Resolution: {msg.width} x {msg.height}")
            rospy.loginfo(f"  Frame ID: {msg.header.frame_id}")
            rospy.loginfo(f"  Distortion Model: {msg.distortion_model}")
            
            # Camera matrix
            K = np.array(msg.K).reshape(3, 3)
            rospy.loginfo(f"  Camera Matrix K:")
            rospy.loginfo(f"    fx={K[0,0]:.2f}, fy={K[1,1]:.2f}")
            rospy.loginfo(f"    cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
            rospy.loginfo("=" * 60)
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        self.image_count += 1
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        # Display image statistics
        if self.image_count % 30 == 1:  # Every ~1 second at 30Hz
            rospy.loginfo(f"Image {self.image_count}:")
            rospy.loginfo(f"  Shape: {cv_image.shape}")
            rospy.loginfo(f"  Min pixel value: {cv_image.min()}")
            rospy.loginfo(f"  Max pixel value: {cv_image.max()}")
            rospy.loginfo(f"  Mean pixel value: {cv_image.mean():.2f}")
            
            # Check if image is mostly gray/empty
            if cv_image.max() < 10:
                rospy.logwarn("  WARNING: Image appears to be BLACK/EMPTY!")
            elif cv_image.std() < 5:
                rospy.logwarn("  WARNING: Image has very low variation (may be solid color)!")
            else:
                rospy.loginfo("  Image looks OK (has variation)")
        
        # Save sample images
        if self.images_saved < 5 and self.image_count % 15 == 0:
            filename = f"/tmp/camera_test_{self.images_saved}.jpg"
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved sample image: {filename}")
            self.images_saved += 1
            
            # Analyze the saved image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            edge_count = np.count_nonzero(edges)
            rospy.loginfo(f"  Edge pixels detected: {edge_count}")
            
            # Color analysis
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            rospy.loginfo(f"  HSV ranges - H:[{hsv[:,:,0].min()}, {hsv[:,:,0].max()}]")
            rospy.loginfo(f"               S:[{hsv[:,:,1].min()}, {hsv[:,:,1].max()}]")
            rospy.loginfo(f"               V:[{hsv[:,:,2].min()}, {hsv[:,:,2].max()}]")
            
            if self.images_saved == 5:
                rospy.loginfo("=" * 60)
                rospy.loginfo("CAMERA TEST COMPLETE!")
                rospy.loginfo("Saved 5 sample images to /tmp/camera_test_*.jpg")
                rospy.loginfo("Check these images to see what camera is seeing.")
                rospy.loginfo("You can view them with: eog /tmp/camera_test_*.jpg")
                rospy.loginfo("=" * 60)


if __name__ == '__main__':
    try:
        tester = CameraTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
