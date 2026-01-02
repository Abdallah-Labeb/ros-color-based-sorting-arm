#!/usr/bin/env python3
"""
Simple Arm Test Script
Directly tests the arm movement without waiting for camera/detection.
Run this to verify the arm is moving correctly.
"""

import rospy
from std_msgs.msg import Float64
import time

def main():
    rospy.init_node('arm_test', anonymous=True)
    rospy.loginfo("=" * 60)
    rospy.loginfo("ARM TEST SCRIPT - Testing basic arm movements")
    rospy.loginfo("=" * 60)
    
    # Create publishers for each joint
    pubs = {}
    for i in range(1, 6):
        topic = f'/sorting_arm/joint{i}_position_controller/command'
        pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
    
    gripper_left = rospy.Publisher('/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
    gripper_right = rospy.Publisher('/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
    
    rospy.loginfo("Waiting for publishers to connect...")
    rospy.sleep(2.0)
    
    def move(j1, j2, j3, j4, j5, duration=2.0, description=""):
        if description:
            rospy.loginfo(f">>> {description}")
        rospy.loginfo(f"    Joints: [{j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}, {j5:.2f}]")
        pubs['joint1'].publish(Float64(j1))
        pubs['joint2'].publish(Float64(j2))
        pubs['joint3'].publish(Float64(j3))
        pubs['joint4'].publish(Float64(j4))
        pubs['joint5'].publish(Float64(j5))
        rospy.sleep(duration)
    
    def gripper(state):
        if state == "open":
            rospy.loginfo(">>> Opening gripper")
            gripper_left.publish(Float64(0.015))
            gripper_right.publish(Float64(0.015))
        else:
            rospy.loginfo(">>> Closing gripper")
            gripper_left.publish(Float64(0.003))
            gripper_right.publish(Float64(0.003))
        rospy.sleep(0.5)
    
    # ========== TEST SEQUENCE ==========
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 1: NEUTRAL POSITION")
    rospy.loginfo("=" * 60)
    move(0, 0, 0, 0, 0, 2.0, "All joints to zero")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 2: BASE ROTATION (Joint 1)")
    rospy.loginfo("=" * 60)
    move(1.0, 0, 0, 0, 0, 1.5, "Rotate base LEFT")
    move(-1.0, 0, 0, 0, 0, 1.5, "Rotate base RIGHT")
    move(0, 0, 0, 0, 0, 1.5, "Return to center")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 3: SHOULDER (Joint 2)")
    rospy.loginfo("=" * 60)
    move(0, 1.2, 0, 0, 0, 1.5, "Shoulder FORWARD")
    move(0, -0.5, 0, 0, 0, 1.5, "Shoulder BACK")
    move(0, 0, 0, 0, 0, 1.5, "Return to neutral")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 4: ELBOW (Joint 3)")
    rospy.loginfo("=" * 60)
    move(0, 0.8, -1.5, 0, 0, 1.5, "Bend elbow DOWN")
    move(0, 0.8, 1.0, 0, 0, 1.5, "Bend elbow UP")
    move(0, 0, 0, 0, 0, 1.5, "Return to neutral")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 5: WRIST (Joint 4)")
    rospy.loginfo("=" * 60)
    move(0, 0.8, -1.0, -0.8, 0, 1.5, "Wrist DOWN")
    move(0, 0.8, -1.0, 0.8, 0, 1.5, "Wrist UP")
    move(0, 0, 0, 0, 0, 1.5, "Return to neutral")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 6: WRIST ROLL (Joint 5)")
    rospy.loginfo("=" * 60)
    move(0, 0.8, -1.0, 0, 1.5, 1.5, "Roll LEFT")
    move(0, 0.8, -1.0, 0, -1.5, 1.5, "Roll RIGHT")
    move(0, 0, 0, 0, 0, 1.5, "Return to neutral")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 7: GRIPPER")
    rospy.loginfo("=" * 60)
    gripper("open")
    gripper("close")
    gripper("open")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 8: REACHING POSE")
    rospy.loginfo("=" * 60)
    move(0, 1.0, -1.5, -0.5, 0, 2.0, "Reaching forward pose")
    gripper("close")
    move(0, 0.5, -0.8, -0.3, 0, 2.0, "Lift up")
    gripper("open")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 9: WAVE MOTION")
    rospy.loginfo("=" * 60)
    for i in range(3):
        rospy.loginfo(f"Wave {i+1}")
        move(0.6, 0.6, -1.0, 0, 0, 0.6, "Wave left")
        move(-0.6, 0.6, -1.0, 0, 0, 0.6, "Wave right")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("TEST 10: HOME POSITION")
    rospy.loginfo("=" * 60)
    move(0, 0.6, -1.0, -0.3, 0, 2.0, "Home position")
    gripper("open")
    
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("ALL TESTS COMPLETE!")
    rospy.loginfo("The arm should have moved through various positions.")
    rospy.loginfo("If you saw the arm move, the system is working correctly.")
    rospy.loginfo("=" * 60)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
