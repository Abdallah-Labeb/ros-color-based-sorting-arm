#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64

class DirectSorting:
    
    CUBE_POSITIONS = {
        'red_1': (0.25, 0.15, 0.725),
        'red_2': (0.4, 0.2, 0.725),
        'blue_1': (0.3, -0.1, 0.725),
        'blue_2': (0.5, -0.15, 0.725),
        'green_1': (0.35, 0.05, 0.725),
        'green_2': (0.45, 0.1, 0.725)
    }
    
    BIN_LOCATIONS = {
        'red': (0.6, 0.25, 0.05),
        'blue': (0.6, -0.25, 0.05),
        'green': (0.6, 0.0, 0.05)
    }
    
    L1 = 0.11
    L2 = 0.15
    L3 = 0.13
    L4 = 0.16
    
    def __init__(self):
        rospy.init_node('direct_sorting')
        rospy.loginfo("Direct Sorting Controller Started")
        
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        rospy.sleep(2.0)
        
        self.open_gripper()
        self.move_to_home()
        rospy.sleep(2.0)
        
        self.sort_all_cubes()
    
    def open_gripper(self):
        self.gripper_left_pub.publish(Float64(0.02))
        self.gripper_right_pub.publish(Float64(0.02))
        rospy.sleep(0.5)
    
    def close_gripper(self):
        self.gripper_left_pub.publish(Float64(0.005))
        self.gripper_right_pub.publish(Float64(0.005))
        rospy.sleep(0.5)
    
    def set_joint_positions(self, positions, duration=2.0):
        for i, pos in enumerate(positions, 1):
            self.joint_pubs[f'joint{i}'].publish(Float64(pos))
        rospy.sleep(duration)
    
    def move_to_home(self):
        home = [0.0, 0.3, -0.5, 0.2, 0.0]
        self.set_joint_positions(home, 2.0)
    
    def inverse_kinematics(self, x, y, z):
        j1 = math.atan2(y, x)
        
        r = math.sqrt(x*x + y*y)
        
        wrist_x = r - self.L4 * math.cos(0)
        wrist_z = z - self.L1 - self.L4 * math.sin(0)
        
        d = math.sqrt(wrist_x**2 + wrist_z**2)
        
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        if d > max_reach or d < min_reach:
            rospy.logwarn(f"Target unreachable: d={d:.3f}, max={max_reach:.3f}")
            return None
        
        cos_j3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3 = -math.acos(cos_j3)
        
        alpha = math.atan2(wrist_z, wrist_x)
        beta = math.atan2(self.L3 * math.sin(j3), self.L2 + self.L3 * math.cos(j3))
        j2 = alpha - beta
        
        j4 = -(j2 + j3)
        
        j5 = 0.0
        
        j3 = -j3
        
        return [j1, j2, j3, j4, j5]
    
    def move_to_position(self, x, y, z):
        joints = self.inverse_kinematics(x, y, z)
        if joints:
            self.set_joint_positions(joints, 2.0)
            return True
        return False
    
    def pick_cube(self, x, y, z):
        rospy.loginfo(f"Picking cube at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        approach_z = z + 0.15
        if not self.move_to_position(x, y, approach_z):
            return False
        
        if not self.move_to_position(x, y, z + 0.05):
            return False
        
        self.close_gripper()
        
        self.move_to_position(x, y, approach_z)
        
        return True
    
    def place_cube(self, x, y, z):
        rospy.loginfo(f"Placing cube at ({x:.3f}, {y:.3f}, {z:.3f})")
        
        approach_z = z + 0.15
        if not self.move_to_position(x, y, approach_z):
            return False
        
        if not self.move_to_position(x, y, z + 0.05):
            return False
        
        self.open_gripper()
        
        self.move_to_position(x, y, approach_z)
        
        return True
    
    def sort_cube(self, cube_name, cube_pos):
        color = cube_name.split('_')[0]
        bin_pos = self.BIN_LOCATIONS[color]
        
        rospy.loginfo("="*50)
        rospy.loginfo(f"Sorting {cube_name} ({color})")
        
        if self.pick_cube(*cube_pos):
            if self.place_cube(*bin_pos):
                rospy.loginfo(f"Successfully sorted {cube_name}")
            else:
                rospy.logwarn(f"Failed to place {cube_name}")
        else:
            rospy.logwarn(f"Failed to pick {cube_name}")
        
        self.move_to_home()
        rospy.sleep(1.0)
    
    def sort_all_cubes(self):
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("STARTING SORTING SEQUENCE")
        rospy.loginfo("="*50 + "\n")
        
        for cube_name, position in self.CUBE_POSITIONS.items():
            self.sort_cube(cube_name, position)
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("SORTING COMPLETE!")
        rospy.loginfo("="*50 + "\n")
        
        self.move_to_home()

if __name__ == '__main__':
    try:
        controller = DirectSorting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
