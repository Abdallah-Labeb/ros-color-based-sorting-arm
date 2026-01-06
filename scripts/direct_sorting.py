#!/usr/bin/env python3
"""
Simple Direct Sorting - Uses pre-calculated joint angles
No complex IK - just simple trigonometry
"""

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

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
        'red': (0.35, 0.18),
        'blue': (0.35, -0.18),
        'green': (0.35, 0.0)
    }
    
    def __init__(self):
        rospy.init_node('direct_sorting')
        
        # Joint publishers
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[i] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        self.cube_poses = {}
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb)
        
        rospy.sleep(2.0)
        rospy.loginfo("="*50)
        rospy.loginfo("SIMPLE SORTING - NO COMPLEX IK")
        rospy.loginfo("="*50)
        
        self.gripper_open()
        self.go_home()
        
        rospy.sleep(2.0)
        self.sort_cubes()
        
    def model_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.CUBE_COLORS:
                self.cube_poses[name] = msg.pose[i]
    
    def send_joint(self, joint_num, angle):
        self.joint_pubs[joint_num].publish(Float64(angle))
    
    def send_all_joints(self, j1, j2, j3, j4, j5, wait=0.8):
        """Send all joint angles at once"""
        self.send_joint(1, j1)
        self.send_joint(2, j2)
        self.send_joint(3, j3)
        self.send_joint(4, j4)
        self.send_joint(5, j5)
        rospy.sleep(wait)
    
    def gripper_open(self):
        self.gripper_left.publish(Float64(0.03))
        self.gripper_right.publish(Float64(0.03))
        rospy.sleep(0.3)
    
    def gripper_close(self):
        self.gripper_left.publish(Float64(0.0))
        self.gripper_right.publish(Float64(0.0))
        rospy.sleep(0.5)
    
    def go_home(self):
        """Safe home position - arm pointing forward and down"""
        # j1=0: forward, j2=0.5: tilted forward, j3=-1.0: elbow bent
        # j4=-1.0: wrist down, j5=0: no rotation
        self.send_all_joints(0.0, 0.5, -1.0, -1.0, 0.0, wait=1.0)
    
    def calc_angles_for_xy(self, x, y, reach_down=False):
        """
        Calculate joint angles to reach position (x,y) on table.
        Robot is at origin, facing -X direction (yaw=pi).
        
        Since robot faces -X, we need to account for that:
        - Positive X in world = behind the robot
        - We rotate j1 to face the target
        """
        # Robot faces -X direction, so flip x
        # The target x,y in robot frame
        rx = -x  # flip because robot faces -X
        ry = -y  # flip because robot faces -X (yaw=pi)
        
        # j1: base rotation to face target
        j1 = math.atan2(ry, rx)
        
        # Distance to target in XY plane
        dist = math.sqrt(rx**2 + ry**2)
        
        # Arm geometry (approximate)
        # Link2: 0.25m, Link3: 0.20m, Link4+gripper: ~0.14m
        # Total reach ~0.59m, but we need to reach DOWN
        
        # For picking (reach_down=True), we need arm more horizontal
        # For approach (reach_down=False), arm slightly raised
        
        if dist < 0.15:
            dist = 0.15
        if dist > 0.45:
            dist = 0.45
        
        # Scale factor based on distance
        # Farther = more stretched out (j2 smaller, j3 less negative)
        # Closer = more folded (j2 bigger, j3 more negative)
        
        t = (dist - 0.15) / 0.30  # 0 to 1 as distance goes from 0.15 to 0.45
        
        if reach_down:
            # Reaching down to grab - arm more horizontal, wrist down
            j2 = 0.3 + t * 0.4      # 0.3 to 0.7 (more forward for far targets)
            j3 = -1.5 + t * 0.5     # -1.5 to -1.0 (less bent for far)
            j4 = -1.2 + t * 0.2     # wrist pointing down
        else:
            # Approach position - slightly higher
            j2 = 0.4 + t * 0.3      # 0.4 to 0.7
            j3 = -1.3 + t * 0.4     # -1.3 to -0.9
            j4 = -1.0 + t * 0.2     # wrist down
        
        j5 = 0.0
        
        return j1, j2, j3, j4, j5
    
    def pick_cube(self, name):
        if name not in self.cube_poses:
            rospy.logwarn(f"Cube {name} not found!")
            return False
        
        pose = self.cube_poses[name]
        x, y = pose.position.x, pose.position.y
        
        rospy.loginfo(f"Picking {name} at ({x:.3f}, {y:.3f})")
        
        # Step 1: Approach above cube
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=False)
        rospy.loginfo(f"  Approach: j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, j4={j4:.2f}")
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.6)
        
        # Step 2: Open gripper
        self.gripper_open()
        
        # Step 3: Go down to grab
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=True)
        rospy.loginfo(f"  Grab: j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, j4={j4:.2f}")
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.6)
        
        # Step 4: Close gripper
        self.gripper_close()
        
        # Step 5: Lift up
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=False)
        j2 -= 0.2  # lift more
        j3 += 0.2  # bend elbow up
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.6)
        
        return True
    
    def place_cube(self, color):
        x, y = self.BIN_LOCATIONS[color]
        
        rospy.loginfo(f"Placing in {color} bin at ({x:.3f}, {y:.3f})")
        
        # Step 1: Move above bin
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=False)
        rospy.loginfo(f"  Above bin: j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, j4={j4:.2f}")
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.6)
        
        # Step 2: Lower into bin
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=True)
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.6)
        
        # Step 3: Release
        self.gripper_open()
        
        # Step 4: Lift away
        j1, j2, j3, j4, j5 = self.calc_angles_for_xy(x, y, reach_down=False)
        j2 -= 0.2
        j3 += 0.2
        self.send_all_joints(j1, j2, j3, j4, j5, wait=0.5)
        
        return True
    
    def sort_cubes(self):
        rospy.loginfo("="*50)
        rospy.loginfo("STARTING SORTING")
        rospy.loginfo("="*50)
        
        count = 0
        for cube, color in self.CUBE_COLORS.items():
            rospy.loginfo(f"\n>>> {cube} -> {color}")
            
            if self.pick_cube(cube):
                if self.place_cube(color):
                    count += 1
                    rospy.loginfo(f"OK!")
            
            self.go_home()
            rospy.sleep(0.5)
        
        rospy.loginfo("="*50)
        rospy.loginfo(f"DONE: {count}/{len(self.CUBE_COLORS)}")
        rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        DirectSorting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
