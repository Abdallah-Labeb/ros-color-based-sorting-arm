#!/usr/bin/env python3
"""
Camera Diagnostic Script
Diagnoses camera issues by checking topics, transforms, and image data
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import subprocess


def check_topics():
    """Check if camera topics exist and are publishing"""
    print("\n" + "="*70)
    print("CHECKING CAMERA TOPICS...")
    print("="*70)
    
    result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
    topics = result.stdout.split('\n')
    
    camera_topics = [t for t in topics if 'camera' in t]
    print(f"Found {len(camera_topics)} camera-related topics:")
    for topic in camera_topics:
        print(f"  - {topic}")
    
    # Check if image_raw is publishing
    print("\nChecking /camera/image_raw rate...")
    result = subprocess.run(['rostopic', 'hz', '/camera/image_raw'], 
                          capture_output=True, text=True, timeout=5)
    print(result.stdout[:200] if result.stdout else "No output")


def analyze_image_data():
    """Capture and analyze one image"""
    print("\n" + "="*70)
    print("ANALYZING IMAGE DATA...")
    print("="*70)
    
    rospy.init_node('camera_diagnostic', anonymous=True)
    bridge = CvBridge()
    
    try:
        # Wait for one image
        print("Waiting for image...")
        msg = rospy.wait_for_message('/camera/image_raw', Image, timeout=5.0)
        
        print(f"Image received!")
        print(f"  Encoding: {msg.encoding}")
        print(f"  Size: {msg.width}x{msg.height}")
        print(f"  Step: {msg.step}")
        print(f"  Data length: {len(msg.data)}")
        
        # Try to convert
        print(f"\nTrying to convert image...")
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f"✓ Conversion successful!")
            print(f"  Shape: {cv_image.shape}")
            print(f"  Dtype: {cv_image.dtype}")
            print(f"  Min: {cv_image.min()}, Max: {cv_image.max()}, Mean: {cv_image.mean():.2f}")
            print(f"  Std Dev: {cv_image.std():.2f}")
            
            # Save it
            filename = "/tmp/diagnostic_image.jpg"
            cv2.imwrite(filename, cv_image)
            print(f"  Saved to: {filename}")
            
            # Check if it's the gray ground plane
            if cv_image.std() < 10:
                print("\n⚠️  WARNING: Image has very low variation!")
                print("   This suggests camera is looking at uniform surface (sky or ground)")
                print("   Pixel values: min={}, max={}".format(cv_image.min(), cv_image.max()))
                
                # Check if it's gray
                if abs(cv_image[:,:,0].mean() - cv_image[:,:,1].mean()) < 5:
                    print("   Image is GRAYSCALE - likely looking at ground plane!")
            else:
                print("\n✓ Image has good variation - camera seeing scene correctly!")
                
        except Exception as e:
            print(f"✗ Conversion failed: {e}")
            
    except rospy.ROSException as e:
        print(f"✗ Failed to receive image: {e}")


def check_gazebo_camera():
    """Check Gazebo camera model"""
    print("\n" + "="*70)
    print("GAZEBO CAMERA CHECK...")
    print("="*70)
    
    try:
        result = subprocess.run(['gz', 'model', '-m', 'overhead_camera', '-i'], 
                              capture_output=True, text=True, timeout=3)
        if result.stdout:
            print("Camera model found in Gazebo")
            print(result.stdout[:500])
        else:
            print("Could not get camera model info")
    except:
        print("gz command not available or failed")


if __name__ == '__main__':
    try:
        print("\n" + "="*70)
        print("CAMERA DIAGNOSTIC TOOL")
        print("="*70)
        
        check_topics()
        analyze_image_data()
        check_gazebo_camera()
        
        print("\n" + "="*70)
        print("DIAGNOSTIC COMPLETE")
        print("="*70)
        print("\nTo view diagnostic image: eog /tmp/diagnostic_image.jpg")
        
    except KeyboardInterrupt:
        print("\nDiagnostic interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
