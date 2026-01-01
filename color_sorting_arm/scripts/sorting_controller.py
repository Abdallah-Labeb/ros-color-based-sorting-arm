#!/usr/bin/env python3
"""
Sorting Controller Node
Main controller that orchestrates the complete sorting pipeline:
1. Detects colored cubes
2. Estimates their 3D positions
3. Plans pick-and-place motions
4. Controls gripper
5. Sorts cubes into designated zones
"""

import rospy
import sys
import os
import numpy as np
from std_msgs.msg import Bool, Float64
from color_sorting_arm.msg import Object3DArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import time


class SortingController:
    def __init__(self):
        rospy.init_node('sorting_controller', anonymous=True)
        
        # Publishers for joint controllers
        self.joint_pubs = {
            'joint1': rospy.Publisher('/sorting_arm/joint1_position_controller/command', 
                                     Float64, queue_size=10),
            'joint2': rospy.Publisher('/sorting_arm/joint2_position_controller/command', 
                                     Float64, queue_size=10),
            'joint3': rospy.Publisher('/sorting_arm/joint3_position_controller/command', 
                                     Float64, queue_size=10),
            'joint4': rospy.Publisher('/sorting_arm/joint4_position_controller/command', 
                                     Float64, queue_size=10),
            'joint5': rospy.Publisher('/sorting_arm/joint5_position_controller/command', 
                                     Float64, queue_size=10),
        }
        
        # Current joint states
        self.current_joints = None
        
        # Subscriber for joint states
        self.joint_sub = rospy.Subscriber('/sorting_arm/joint_states', JointState, 
                                         self.joint_state_callback)
        
        # Link lengths (from URDF)
        self.L1 = 0.2   # Link 1 height
        self.L2 = 0.3   # Link 2 length
        self.L3 = 0.3   # Link 3 length
        self.L4 = 0.2   # Link 4 length
        self.L5 = 0.1   # Link 5 to gripper
        
        # Gripper command publisher
        self.gripper_pub = rospy.Publisher('/gripper_command', Bool, queue_size=10)
        
        # Wait for publishers to be ready
        rospy.sleep(1.0)
        
        # Detected objects
        self.detected_objects = []
        self.processing = False
        
        # Sorting zone positions (x, y, z)
        self.sorting_zones = {
            'red': (0.2, 0.3, 0.85),
            'blue': (0.2, 0.0, 0.85),
            'green': (0.2, -0.3, 0.85)
        }
        
        # Subscribe to object positions
        self.object_sub = rospy.Subscriber('/object_positions', Object3DArray, 
                                          self.object_callback)
        
        rospy.loginfo("Sorting Controller Node initialized")
        rospy.loginfo("Waiting for objects to detect...")
    
    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joints = {}
        for i, name in enumerate(msg.name):
            if 'joint' in name and 'gripper' not in name:
                self.current_joints[name] = msg.position[i]
    
    def move_joints(self, joint_positions, duration=2.0):
        """Move joints to specified positions"""
        for joint_name, position in joint_positions.items():
            if joint_name in self.joint_pubs:
                self.joint_pubs[joint_name].publish(position)
        rospy.sleep(duration)
    
    def move_to_home(self):
        """Move arm to home position"""
        rospy.loginfo("Moving to home position")
        home_positions = {
            'joint1': 1.57,   # face the table
            'joint2': 0.5,    # Tilt forward to see table
            'joint3': -0.5,   # Elbow compensation
            'joint4': -0.3,   # Wrist down to point camera forward
            'joint5': 0.0,
        }
        self.move_joints(home_positions)
    
    def move_to_observation_pose(self):
        """Move arm to observation position to see cubes"""
        rospy.loginfo("Moving to observation position")
        observation_positions = {
            'joint1': 1.57,   # rotate toward table/cubes
            'joint2': 0.9,    # Tilt more forward
            'joint3': -1.0,   # Elbow up
            'joint4': -0.6,   # Wrist angled to see table
            'joint5': 0.0,
        }
        self.move_joints(observation_positions, duration=3.0)
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        """Simple inverse kinematics for 5-DOF arm"""
        # Joint 1 (base rotation)
        j1 = np.arctan2(target_y, target_x)
        
        # Horizontal reach distance
        r = np.sqrt(target_x**2 + target_y**2)
        
        # Vertical height from joint 2
        z = target_z - self.L1
        
        # Account for end effector length
        gripper_offset = self.L4 + self.L5
        r_wrist = r - gripper_offset * 0.3
        z_wrist = z + gripper_offset * 0.3
        
        # Distance to wrist target
        d = np.sqrt(r_wrist**2 + z_wrist**2)
        
        # Check if target is reachable
        if d > (self.L2 + self.L3):
            rospy.logwarn(f"Target may be out of reach: d={d:.2f}, max={self.L2 + self.L3:.2f}")
            d = self.L2 + self.L3 - 0.01
        
        # Joint 3 (elbow)
        cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = np.clip(cos_j3, -1, 1)
        j3 = np.arccos(cos_j3)
        
        # Joint 2 (shoulder)
        alpha = np.arctan2(z_wrist, r_wrist)
        beta = np.arctan2(self.L3 * np.sin(j3), self.L2 + self.L3 * np.cos(j3))
        j2 = alpha - beta
        
        # Joint 4 (wrist pitch)
        j4 = -(j2 + j3) - np.pi/4
        
        # Joint 5 (wrist roll)
        j5 = -j1
        
        # Apply joint limits
        j1 = np.clip(j1, -np.pi, np.pi)
        j2 = np.clip(j2, -np.pi/2, np.pi/2)
        j3 = np.clip(j3, -np.pi/2, np.pi/2)
        j4 = np.clip(j4, -np.pi/2, np.pi/2)
        j5 = np.clip(j5, -np.pi, np.pi)
        
        return [j1, j2, j3, j4, j5]
    
    def move_to_position(self, x, y, z):
        """Move end effector to target position"""
        rospy.loginfo(f"Moving to position: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        try:
            joint_angles = self.inverse_kinematics(x, y, z)
            joint_positions = {
                'joint1': joint_angles[0],
                'joint2': joint_angles[1],
                'joint3': joint_angles[2],
                'joint4': joint_angles[3],
                'joint5': joint_angles[4],
            }
            self.move_joints(joint_positions, duration=3.0)
            rospy.loginfo("Reached target position")
            return True
        except Exception as e:
            rospy.logerr(f"IK Error: {e}")
            return False
    
    def pick_object(self, x, y, z):
        """Execute pick sequence"""
        rospy.loginfo(f"Picking object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Move above object
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        # Move down to object
        self.move_to_position(x, y, z)
        rospy.sleep(0.5)
        
        # Close gripper
        rospy.sleep(1.0)
        
        # Lift object
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def place_object(self, x, y, z):
        """Execute place sequence"""
        rospy.loginfo(f"Placing object at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Move above placement location
        above_z = z + 0.15
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
        
        # Move down to placement height
        self.move_to_position(x, y, z + 0.05)
        rospy.sleep(0.5)
        
        # Open gripper
        rospy.sleep(1.0)
        
        # Move up
        self.move_to_position(x, y, above_z)
        rospy.sleep(0.5)
    
    def object_callback(self, msg):
        """Store detected objects"""
        if not self.processing and len(msg.objects) > 0:
            self.detected_objects = msg.objects
            rospy.loginfo(f"Detected {len(self.detected_objects)} objects")
    
    def open_gripper(self):
        """Send command to open gripper"""
        self.gripper_pub.publish(True)
        rospy.sleep(1.5)
    
    def close_gripper(self):
        """Send command to close gripper"""
        self.gripper_pub.publish(False)
        rospy.sleep(1.5)
    
    def sort_single_object(self, obj):
        """Sort a single object"""
        color = obj.color
        pos = obj.position
        
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"Sorting {color} cube")
        rospy.loginfo(f"Current position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        
        # Open gripper
        self.open_gripper()
        
        # Pick object
        rospy.loginfo(f"Step 1: Moving to pick {color} cube...")
        self.pick_object(pos.x, pos.y, pos.z)
        
        # Close gripper to grab
        rospy.loginfo(f"Step 2: Grabbing {color} cube...")
        self.close_gripper()
        
        # Get sorting zone for this color
        if color in self.sorting_zones:
            zone_x, zone_y, zone_z = self.sorting_zones[color]
            rospy.loginfo(f"Step 3: Moving to {color} zone ({zone_x}, {zone_y}, {zone_z})...")
            
            # Place object
            self.place_object(zone_x, zone_y, zone_z)
            
            # Open gripper to release
            rospy.loginfo(f"Step 4: Releasing {color} cube...")
            self.open_gripper()
            
            rospy.loginfo(f"Successfully sorted {color} cube!")
        else:
            rospy.logwarn(f"Unknown color: {color}")
            self.open_gripper()
        
        # Return to home
        rospy.loginfo("Returning to home position...")
        self.move_to_home()
        
        rospy.loginfo(f"{'='*50}\n")
    
    def sort_all_objects(self):
        """Sort all detected objects"""
        if len(self.detected_objects) == 0:
            rospy.loginfo("No objects to sort")
            return
        
        self.processing = True
        
        rospy.loginfo(f"\n{'#'*50}")
        rospy.loginfo(f"# Starting sorting sequence")
        rospy.loginfo(f"# Total objects: {len(self.detected_objects)}")
        rospy.loginfo(f"{'#'*50}\n")
        
        # Sort objects by distance (closest first)
        sorted_objects = sorted(self.detected_objects, 
                              key=lambda obj: obj.position.x**2 + obj.position.y**2)
        
        # Process each object
        for i, obj in enumerate(sorted_objects):
            rospy.loginfo(f"\nProcessing object {i+1}/{len(sorted_objects)}")
            self.sort_single_object(obj)
            rospy.sleep(1.0)
        
        rospy.loginfo(f"\n{'#'*50}")
        rospy.loginfo(f"# Sorting sequence completed!")
        rospy.loginfo(f"# All {len(sorted_objects)} objects sorted successfully")
        rospy.loginfo(f"{'#'*50}\n")
        
        self.detected_objects = []
        self.processing = False
    
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(0.5)  # 0.5 Hz (check every 2 seconds)
        
        rospy.loginfo("Sorting controller ready. Waiting for objects...")
        
        # Wait a bit for everything to initialize
        rospy.sleep(3.0)
        
        # Move to observation position to see the cubes
        rospy.loginfo("Moving to observation position to detect cubes...")
        self.move_to_observation_pose()
        rospy.sleep(2.0)
        
        while not rospy.is_shutdown():
            if not self.processing and len(self.detected_objects) > 0:
                # Start sorting
                rospy.loginfo("\nObjects detected! Starting sorting sequence in 2 seconds...")
                rospy.sleep(2.0)
                self.sort_all_objects()
            
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
