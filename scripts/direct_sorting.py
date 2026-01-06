#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class DirectSorting:
    
    CUBE_COLORS = {
        'red_cube_1': 'red',
        'red_cube_2': 'red',
        'blue_cube_1': 'blue',
        'blue_cube_2': 'blue',
        'green_cube_1': 'green',
        'green_cube_2': 'green'
    }
    
    BIN_LOCATIONS = {
        'red': (0.45, 0.20, 0.75),
        'blue': (0.45, -0.20, 0.75),
        'green': (0.45, 0.0, 0.75)
    }
    
    L1 = 0.11
    L2 = 0.25
    L3 = 0.20
    L4 = 0.12
    
    def __init__(self):
        rospy.init_node('direct_sorting')
        
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        self.cube_poses = {}
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        rospy.sleep(3.0)
        self.open_gripper()
        self.move_to_home()
        
        rospy.loginfo("Direct Sorting Ready! Starting in 3 seconds...")
        rospy.sleep(3.0)
        
        self.sort_all_cubes()
        
    def model_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.CUBE_COLORS:
                self.cube_poses[name] = msg.pose[i]
    
    def inverse_kinematics(self, x, y, z):
        j1 = math.atan2(y, x)
        
        r = math.sqrt(x**2 + y**2)
        wrist_x = r - self.L4 * math.cos(0)
        wrist_z = z - self.L1 - self.L4 * math.sin(0)
        
        d = math.sqrt(wrist_x**2 + wrist_z**2)
        max_reach = self.L2 + self.L3
        
        if d > max_reach:
            rospy.logwarn(f"Target ({x:.2f}, {y:.2f}, {z:.2f}) out of reach! d={d:.2f}, max={max_reach:.2f}")
            return None
        
        cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = max(-1, min(1, cos_j3))
        j3 = -math.acos(cos_j3)
        
        alpha = math.atan2(wrist_z, wrist_x)
        beta = math.atan2(self.L3 * math.sin(j3), self.L2 + self.L3 * math.cos(j3))
        j2 = alpha - beta
        
        j4 = -(j2 + j3)
        j5 = 0.0
        
        return [j1, j2, j3, j4, j5]
    
    def move_joints(self, joints, duration=2.0):
        if joints is None:
            return False
        for i, j in enumerate(joints, 1):
            self.joint_pubs[f'joint{i}'].publish(Float64(j))
        rospy.sleep(duration)
        return True
    
    def open_gripper(self):
        self.gripper_left_pub.publish(Float64(0.02))
        self.gripper_right_pub.publish(Float64(0.02))
        rospy.sleep(0.5)
    
    def close_gripper(self):
        self.gripper_left_pub.publish(Float64(0.005))
        self.gripper_right_pub.publish(Float64(0.005))
        rospy.sleep(0.5)
    
    def move_to_home(self):
        home = [0.0, -0.5, -1.0, 1.5, 0.0]
        self.move_joints(home, 2.0)
    
    def pick_cube(self, cube_name):
        if cube_name not in self.cube_poses:
            rospy.logwarn(f"Cube {cube_name} not found in model states!")
            return False
        
        pose = self.cube_poses[cube_name]
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        rospy.loginfo(f"Picking {cube_name} at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        approach_z = z + 0.05
        joints = self.inverse_kinematics(x, y, approach_z)
        if not self.move_joints(joints, 2.0):
            return False
        
        self.open_gripper()
        
        joints = self.inverse_kinematics(x, y, z + 0.025)
        if not self.move_joints(joints, 1.5):
            return False
        
        self.close_gripper()
        
        joints = self.inverse_kinematics(x, y, z + 0.05)
        self.move_joints(joints, 1.0)
        
        return True
    
    def place_cube(self, color):
        x, y, z = self.BIN_LOCATIONS[color]
        
        rospy.loginfo(f"Placing in {color} bin at ({x:.2f}, {y:.2f}, {z:.2f})")
        
        approach_z = z + 0.05
        joints = self.inverse_kinematics(x, y, approach_z)
        if not self.move_joints(joints, 2.0):
            return False
        
        joints = self.inverse_kinematics(x, y, z + 0.03)
        if not self.move_joints(joints, 1.5):
            return False
        
        self.open_gripper()
        
        joints = self.inverse_kinematics(x, y, z + 0.05)
        self.move_joints(joints, 1.0)
        
        return True
    
    def sort_all_cubes(self):
        rospy.loginfo("="*60)
        rospy.loginfo("STARTING AUTOMATIC CUBE SORTING")
        rospy.loginfo("="*60)
        
        for cube_name, color in self.CUBE_COLORS.items():
            rospy.loginfo(f"\nProcessing: {cube_name} ({color})")
            
            if self.pick_cube(cube_name):
                if self.place_cube(color):
                    rospy.loginfo(f"✓ Successfully sorted {cube_name}!")
                else:
                    rospy.logwarn(f"✗ Failed to place {cube_name}")
            else:
                rospy.logwarn(f"✗ Failed to pick {cube_name}")
            
            self.move_to_home()
            rospy.sleep(1.0)
        
        rospy.loginfo("="*60)
        rospy.loginfo("SORTING COMPLETE!")
        rospy.loginfo("="*60)

if __name__ == '__main__':
    try:
        DirectSorting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
