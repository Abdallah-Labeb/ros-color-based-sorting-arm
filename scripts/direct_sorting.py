#!/usr/bin/env python3
"""
Direct Sorting - Pick and place cubes using ground truth positions.
Uses geometric inverse kinematics for a 5-DOF arm.
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
        'red': (0.35, 0.18, 0.73),
        'blue': (0.35, -0.18, 0.73),
        'green': (0.35, 0.0, 0.73)
    }

    # ========== ARM DIMENSIONS FROM URDF ==========
    # Robot spawns at z=0.70 (from launch file)
    # base_link: cylinder height 0.05m, joint1 at z=0.05
    # link1: cylinder height 0.04m, joint2 at z=0.04 from link1
    # link2: cylinder length 0.25m, joint3 at z=0.25 from link2
    # link3: cylinder length 0.20m, joint4 at z=0.20 from link3
    # link4: box height 0.08m, joint5 at z=0.08 from link4
    # gripper: additional ~0.04m
    
    SPAWN_Z = 0.70
    BASE_HEIGHT = 0.05
    LINK1_HEIGHT = 0.04
    L2 = 0.25  # link2 length
    L3 = 0.20  # link3 length
    L4 = 0.08  # link4 length
    GRIPPER_OFFSET = 0.06  # gripper + link5 extension
    
    # Shoulder pivot height (joint2 in world frame)
    SHOULDER_Z = SPAWN_Z + BASE_HEIGHT + LINK1_HEIGHT  # 0.79m
    
    def __init__(self):
        rospy.init_node('direct_sorting')
        
        # Joint publishers
        self.joint_pubs = {}
        for i in range(1, 6):
            topic = f'/sorting_arm/joint{i}_position_controller/command'
            self.joint_pubs[f'joint{i}'] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_left_pub = rospy.Publisher(
            '/sorting_arm/gripper_left_position_controller/command', Float64, queue_size=1)
        self.gripper_right_pub = rospy.Publisher(
            '/sorting_arm/gripper_right_position_controller/command', Float64, queue_size=1)
        
        # Get cube positions from Gazebo
        self.cube_poses = {}
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)
        
        rospy.sleep(2.0)
        rospy.loginfo("="*50)
        rospy.loginfo("DIRECT SORTING NODE INITIALIZED")
        rospy.loginfo(f"Shoulder height: {self.SHOULDER_Z:.3f}m")
        rospy.loginfo(f"Arm reach: L2={self.L2}, L3={self.L3}, L4={self.L4}")
        rospy.loginfo("="*50)
        
        self.open_gripper()
        self.go_home()
        
        rospy.loginfo("Starting sorting in 2 seconds...")
        rospy.sleep(2.0)
        
        self.sort_all_cubes()
        
    def model_states_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.CUBE_COLORS:
                self.cube_poses[name] = msg.pose[i]
    
    def solve_ik(self, x, y, z):
        """
        Geometric IK for 5-DOF arm.
        Returns [j1, j2, j3, j4, j5] or None if unreachable.
        
        The arm has:
        - j1: base rotation (around Z)
        - j2: shoulder pitch (around Y)
        - j3: elbow pitch (around Y)
        - j4: wrist pitch (around Y)
        - j5: wrist roll (around Z)
        """
        # Joint 1: rotation around vertical axis
        j1 = math.atan2(y, x)
        
        # Distance in XY plane from base to target
        r_xy = math.sqrt(x**2 + y**2)
        
        # We want gripper pointing down, so subtract gripper offset from r
        # Gripper extends horizontally when pointing down
        r_wrist = r_xy - self.GRIPPER_OFFSET
        if r_wrist < 0.05:
            r_wrist = 0.05  # minimum reach
        
        # Height of target relative to shoulder joint
        z_wrist = z - self.SHOULDER_Z
        
        rospy.loginfo(f"  IK: target=({x:.3f},{y:.3f},{z:.3f}), r_xy={r_xy:.3f}, r_wrist={r_wrist:.3f}, z_wrist={z_wrist:.3f}")
        
        # We'll use a 2-link planar IK for joints 2 and 3
        # with link4 (j4) compensating for end-effector orientation
        
        # For now, treat L2 and L3 as the two main segments
        # Distance from shoulder to wrist position
        d_sq = r_wrist**2 + z_wrist**2
        d = math.sqrt(d_sq)
        
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        rospy.loginfo(f"  IK: d={d:.3f}, max_reach={max_reach:.3f}, min_reach={min_reach:.3f}")
        
        if d > max_reach:
            rospy.logwarn(f"Target out of reach: d={d:.3f} > max={max_reach:.3f}")
            # Clamp to max reach
            scale = max_reach * 0.98 / d
            r_wrist *= scale
            z_wrist *= scale
            d = max_reach * 0.98
            d_sq = d * d
        
        if d < min_reach:
            rospy.logwarn(f"Target too close: d={d:.3f} < min={min_reach:.3f}")
            d = min_reach + 0.01
            d_sq = d * d
        
        # Elbow angle using cosine rule
        cos_j3 = (d_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        
        # Elbow down configuration (negative angle)
        j3 = -math.acos(cos_j3)
        
        # Shoulder angle
        # alpha: angle from horizontal to line connecting shoulder to wrist
        alpha = math.atan2(z_wrist, r_wrist)
        
        # beta: angle at shoulder in the triangle
        sin_j3 = math.sin(-j3)
        cos_j3_pos = math.cos(-j3)
        beta = math.atan2(self.L3 * sin_j3, self.L2 + self.L3 * cos_j3_pos)
        
        j2 = alpha + beta
        
        # Wrist pitch to keep gripper pointing down
        # Total pitch = j2 + j3 + j4
        # We want total pitch = -pi/2 (pointing straight down)
        desired_pitch = -math.pi / 2
        j4 = desired_pitch - j2 - j3
        
        # Clamp j4 to limits
        j4 = max(-2.0, min(2.0, j4))
        
        # Wrist roll
        j5 = 0.0
        
        rospy.loginfo(f"  IK result: j1={math.degrees(j1):.1f}, j2={math.degrees(j2):.1f}, j3={math.degrees(j3):.1f}, j4={math.degrees(j4):.1f}")
        
        return [j1, j2, j3, j4, j5]
    
    def send_joints(self, joints, wait=0.5):
        """Publish joint commands."""
        if joints is None:
            return False
        
        for i, angle in enumerate(joints, 1):
            self.joint_pubs[f'joint{i}'].publish(Float64(angle))
        
        rospy.sleep(wait)
        return True
    
    def open_gripper(self):
        self.gripper_left_pub.publish(Float64(0.04))
        self.gripper_right_pub.publish(Float64(0.04))
        rospy.sleep(0.3)
    
    def close_gripper(self):
        self.gripper_left_pub.publish(Float64(0.0))
        self.gripper_right_pub.publish(Float64(0.0))
        rospy.sleep(0.4)
    
    def go_home(self):
        """Safe home position with arm raised."""
        # Manually set safe home angles
        home = [0.0, 0.5, -1.5, -0.5, 0.0]
        self.send_joints(home, 0.8)
    
    def pick_cube(self, cube_name):
        """Pick up a specific cube."""
        if cube_name not in self.cube_poses:
            rospy.logwarn(f"Cube {cube_name} not found!")
            return False
        
        pose = self.cube_poses[cube_name]
        cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
        
        rospy.loginfo(f"--- Picking {cube_name} at ({cx:.3f}, {cy:.3f}, {cz:.3f}) ---")
        
        # Cube is 0.025m tall, center at cz
        # Grasp at center height
        grasp_z = cz
        
        # 1) Approach from above
        approach_z = grasp_z + 0.06
        rospy.loginfo(f"  1. Approach z={approach_z:.3f}")
        joints = self.solve_ik(cx, cy, approach_z)
        if not self.send_joints(joints, 0.5):
            return False
        
        self.open_gripper()
        
        # 2) Descend to grasp
        rospy.loginfo(f"  2. Descend z={grasp_z:.3f}")
        joints = self.solve_ik(cx, cy, grasp_z)
        if not self.send_joints(joints, 0.5):
            return False
        
        # 3) Close gripper
        rospy.loginfo("  3. Close gripper")
        self.close_gripper()
        
        # 4) Lift
        lift_z = grasp_z + 0.08
        rospy.loginfo(f"  4. Lift z={lift_z:.3f}")
        joints = self.solve_ik(cx, cy, lift_z)
        self.send_joints(joints, 0.5)
        
        return True
    
    def place_cube(self, color):
        """Place cube in the bin for the given color."""
        bx, by, bz = self.BIN_LOCATIONS[color]
        
        rospy.loginfo(f"--- Placing in {color} bin at ({bx:.3f}, {by:.3f}, {bz:.3f}) ---")
        
        # 1) Approach from above
        approach_z = bz + 0.06
        rospy.loginfo(f"  1. Approach z={approach_z:.3f}")
        joints = self.solve_ik(bx, by, approach_z)
        if not self.send_joints(joints, 0.5):
            return False
        
        # 2) Lower
        rospy.loginfo(f"  2. Lower z={bz:.3f}")
        joints = self.solve_ik(bx, by, bz)
        if not self.send_joints(joints, 0.5):
            return False
        
        # 3) Release
        rospy.loginfo("  3. Open gripper")
        self.open_gripper()
        
        # 4) Lift away
        lift_z = bz + 0.08
        rospy.loginfo(f"  4. Lift z={lift_z:.3f}")
        joints = self.solve_ik(bx, by, lift_z)
        self.send_joints(joints, 0.5)
        
        return True
    
    def sort_all_cubes(self):
        rospy.loginfo("="*60)
        rospy.loginfo("    STARTING CUBE SORTING")
        rospy.loginfo("="*60)
        
        success = 0
        total = len(self.CUBE_COLORS)
        
        for cube_name, color in self.CUBE_COLORS.items():
            rospy.loginfo(f"\n>>> {cube_name} -> {color} bin")
            
            if self.pick_cube(cube_name):
                if self.place_cube(color):
                    rospy.loginfo(f"OK: {cube_name} sorted!")
                    success += 1
                else:
                    rospy.logwarn(f"FAIL: couldn't place {cube_name}")
            else:
                rospy.logwarn(f"FAIL: couldn't pick {cube_name}")
            
            self.go_home()
            rospy.sleep(0.3)
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"    DONE: {success}/{total} cubes sorted")
        rospy.loginfo("="*60)

if __name__ == '__main__':
    try:
        DirectSorting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
