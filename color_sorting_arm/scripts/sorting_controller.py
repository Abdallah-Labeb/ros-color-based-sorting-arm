#!/usr/bin/env python3
"""
Sorting Controller Node
Main control node for the color sorting robotic arm.
Handles pick and place operations based on detected object colors.
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from color_sorting_arm.msg import Object3DArray

class SortingController:
    """Main controller for sorting colored cubes by color."""
    
    # Sorting bin locations (x, y, z) in base_link frame
    BIN_LOCATIONS = {
        'red': (0.2, 0.35, 0.03),    # Left bin
        'blue': (0.2, -0.35, 0.03),  # Right bin
        'green': (0.45, 0.0, 0.03)   # Far bin
    }
    
    # Robot arm parameters (matches URDF link lengths)
    L1 = 0.09   # base_link(0.04) + link1(0.05) = 0.09
    L2 = 0.08   # link2 length
    L3 = 0.07   # link3 length
    L4 = 0.10   # link4(0.05) + link5(0.03) + gripper(0.02)
    
    def __init__(self):
        rospy.init_node('sorting_controller')
        rospy.loginfo("Starting Sorting Controller...")
        
        # State variables
        self.current_joints = [0.0] * 7  # 5 arm joints + 2 gripper
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 
                           'gripper_left_joint', 'gripper_right_joint']
        self.detected_objects = []
        self.is_busy = False
        self.gripper_open = True
        
        # Publishers for joint position commands
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/sorting_arm/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/object_positions', Object3DArray, self.objects_callback)
        
        # Wait for connections
        rospy.sleep(2.0)
        
        # Perform startup test movements
        self.startup_test()
        
        # Move to home position
        self.move_to_home()
        rospy.sleep(1.0)
        
        # Main control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_loop)
        
        rospy.loginfo("Sorting Controller initialized successfully!")
    
    def startup_test(self):
        """
        Perform test movements on startup to verify arm is working.
        This helps confirm the arm is operational.
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("STARTUP TEST: Performing test movements...")
        rospy.loginfo("=" * 50)
        
        # Test 1: Move to neutral position
        rospy.loginfo("Test 1: Moving to neutral position...")
        neutral = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_joint_positions(neutral, 2.0)
        
        # Test 2: Rotate base left
        rospy.loginfo("Test 2: Rotating base left...")
        self.set_joint_positions([0.8, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        # Test 3: Rotate base right
        rospy.loginfo("Test 3: Rotating base right...")
        self.set_joint_positions([-0.8, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        # Test 4: Return to center
        rospy.loginfo("Test 4: Returning to center...")
        self.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0], 1.5)
        
        # Test 5: Bend forward (shoulder)
        rospy.loginfo("Test 5: Bending shoulder forward...")
        self.set_joint_positions([0.0, 1.0, 0.0, 0.0, 0.0], 1.5)
        
        # Test 6: Bend elbow
        rospy.loginfo("Test 6: Bending elbow...")
        self.set_joint_positions([0.0, 1.0, -1.5, 0.0, 0.0], 1.5)
        
        # Test 7: Bend wrist
        rospy.loginfo("Test 7: Bending wrist...")
        self.set_joint_positions([0.0, 1.0, -1.5, -0.5, 0.0], 1.5)
        
        # Test 8: Test gripper
        rospy.loginfo("Test 8: Testing gripper open...")
        self.open_gripper()
        rospy.sleep(0.5)
        
        rospy.loginfo("Test 9: Testing gripper close...")
        self.close_gripper()
        rospy.sleep(0.5)
        
        self.open_gripper()
        
        # Test 10: Wave motion
        rospy.loginfo("Test 10: Performing wave motion...")
        for _ in range(2):
            self.set_joint_positions([0.5, 0.5, -1.0, 0.0, 0.0], 0.8)
            self.set_joint_positions([-0.5, 0.5, -1.0, 0.0, 0.0], 0.8)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("STARTUP TEST COMPLETE! Arm is working correctly.")
        rospy.loginfo("=" * 50)
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_joints[idx] = msg.position[i]
    
    def objects_callback(self, msg):
        """Receive detected 3D object positions."""
        self.detected_objects = msg.objects
    
    def publish_joint_command(self, joint_name, position):
        """Publish command to a single joint."""
        if joint_name in self.joint_pubs:
            cmd = Float64()
            cmd.data = float(position)
            self.joint_pubs[joint_name].publish(cmd)
    
    def set_joint_positions(self, positions, duration=2.0):
        """
        Set all joint positions and wait for completion.
        positions: [j1, j2, j3, j4, j5]
        """
        # Publish all joint commands
        for i, pos in enumerate(positions):
            self.publish_joint_command(f'joint{i+1}', pos)
        
        # Wait for motion to complete
        rospy.sleep(duration)
    
    def open_gripper(self):
        """Open the gripper."""
        rospy.loginfo("Opening gripper...")
        self.gripper_left_pub.publish(Float64(0.015))
        self.gripper_right_pub.publish(Float64(0.015))
        self.gripper_open = True
        rospy.sleep(0.5)
    
    def close_gripper(self):
        """Close the gripper to grasp object."""
        rospy.loginfo("Closing gripper...")
        self.gripper_left_pub.publish(Float64(0.003))
        self.gripper_right_pub.publish(Float64(0.003))
        self.gripper_open = False
        rospy.sleep(0.5)
    
    def move_to_home(self):
        """Move arm to home/observation position."""
        rospy.loginfo("Moving to home position...")
        # Home position: arm bent forward ready to pick
        # joint2 tilts forward, joint3 bends down
        home = [0.0, 0.6, -1.0, -0.3, 0.0]
        self.set_joint_positions(home, 2.0)
        self.open_gripper()
    
    def inverse_kinematics(self, x, y, z, pitch=math.pi/2):
        """
        Calculate joint angles to reach target position.
        Uses geometric approach for 5-DOF arm.
        
        Args:
            x, y, z: Target position in base_link frame
            pitch: Desired end-effector pitch angle (default: pointing down)
        
        Returns:
            List of 5 joint angles or None if unreachable
        """
        try:
            # Joint 1: Base rotation
            j1 = math.atan2(y, x)
            
            # Distance in XY plane
            r = math.sqrt(x**2 + y**2)
            
            # Wrist position (accounting for gripper/wrist length)
            wrist_offset = self.L4 * math.cos(pitch)
            z_offset = self.L4 * math.sin(pitch)
            
            r_wrist = r - wrist_offset if r > wrist_offset else 0.01
            z_wrist = z - self.L1 + z_offset
            
            # Distance from shoulder to wrist
            d = math.sqrt(r_wrist**2 + z_wrist**2)
            
            # Check reachability
            max_reach = self.L2 + self.L3
            if d > max_reach * 0.95:
                rospy.logwarn(f"Target may be at edge of reach: d={d:.3f}, max={max_reach:.3f}")
                d = max_reach * 0.95
            
            # Elbow angle using law of cosines
            cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            cos_j3 = np.clip(cos_j3, -1.0, 1.0)
            j3 = math.acos(cos_j3)  # Elbow angle
            
            # Shoulder angle
            alpha = math.atan2(z_wrist, r_wrist)
            beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
            j2 = alpha + beta  # Shoulder lifts up
            
            # Wrist pitch to maintain end-effector orientation
            j4 = pitch - j2 - j3 + math.pi/2
            
            # Wrist roll (keep at 0 for now)
            j5 = 0.0
            
            # Adjust j3 to match URDF convention
            j3 = -(math.pi - j3)
            
            return [j1, j2, j3, j4, j5]
            
        except Exception as e:
            rospy.logerr(f"IK calculation failed: {e}")
            return None
    
    def move_to_position(self, x, y, z, pitch=math.pi/2):
        """Move end-effector to specified position."""
        rospy.loginfo(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        joints = self.inverse_kinematics(x, y, z, pitch)
        if joints is None:
            rospy.logerr("Failed to calculate IK solution")
            return False
        
        rospy.loginfo(f"Joint angles: {[f'{j:.2f}' for j in joints]}")
        self.set_joint_positions(joints, 2.0)
        return True
    
    def pick_object(self, x, y, z):
        """
        Execute pick operation at given position.
        """
        rospy.loginfo(f"Picking object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Approach position (above the object)
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        # Move down to grasp
        if not self.move_to_position(x, y, z + 0.02):
            return False
        rospy.sleep(0.5)
        
        # Close gripper
        self.close_gripper()
        rospy.sleep(0.5)
        
        # Lift object
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def place_object(self, x, y, z):
        """
        Execute place operation at given position.
        """
        rospy.loginfo(f"Placing object at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Approach position (above the target)
        approach_z = z + 0.1
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        # Move down to place
        if not self.move_to_position(x, y, z + 0.03):
            return False
        rospy.sleep(0.5)
        
        # Open gripper to release
        self.open_gripper()
        rospy.sleep(0.5)
        
        # Lift away
        if not self.move_to_position(x, y, approach_z):
            return False
        rospy.sleep(0.5)
        
        return True
    
    def sort_object(self, obj):
        """
        Sort a single object: pick it up and place in appropriate bin.
        """
        color = obj.color.lower()
        
        if color not in self.BIN_LOCATIONS:
            rospy.logwarn(f"Unknown color: {color}, skipping object")
            return False
        
        rospy.loginfo(f"Sorting {color} object...")
        
        # Pick up the object
        pick_z = obj.position.z - 0.7  # Adjust for table height relative to robot base
        if not self.pick_object(obj.position.x, obj.position.y, pick_z):
            rospy.logerr("Pick operation failed")
            return False
        
        # Get bin location
        bin_x, bin_y, bin_z = self.BIN_LOCATIONS[color]
        
        # Place in bin
        if not self.place_object(bin_x, bin_y, bin_z):
            rospy.logerr("Place operation failed")
            return False
        
        rospy.loginfo(f"Successfully sorted {color} object!")
        return True
    
    def control_loop(self, event):
        """Main control loop - called periodically."""
        if self.is_busy:
            return
        
        if not self.detected_objects:
            return
        
        # Get the first detected object
        obj = self.detected_objects[0]
        
        # Mark as busy
        self.is_busy = True
        
        try:
            # Sort the object
            success = self.sort_object(obj)
            
            if success:
                # Remove sorted object from list
                self.detected_objects = self.detected_objects[1:]
            
            # Return to home position
            self.move_to_home()
            
        except Exception as e:
            rospy.logerr(f"Error in sorting: {e}")
            self.move_to_home()
        
        finally:
            self.is_busy = False
    
    def run(self):
        """Run the controller."""
        rospy.spin()


def main():
    try:
        controller = SortingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
