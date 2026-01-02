#!/usr/bin/env python3
"""
Position Estimator Node
Converts 2D pixel coordinates from color detection to 3D world coordinates.
Uses camera intrinsics and TF transforms.
"""

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo
from color_sorting_arm.msg import DetectedObject, DetectedObjectArray, Object3D, Object3DArray
import tf2_geometry_msgs


class PositionEstimator:
    """Estimates 3D positions of detected objects from 2D camera detections."""
    
    # Table height in world frame (known from simulation)
    TABLE_HEIGHT = 0.725  # z position of objects on table
    
    def __init__(self):
        rospy.init_node('position_estimator')
        rospy.loginfo("Starting Position Estimator...")
        
        # Camera parameters
        self.camera_matrix = None
        self.camera_frame = 'camera_optical_frame'
        self.base_frame = 'base_link'
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for 3D positions
        self.position_pub = rospy.Publisher('/object_positions', Object3DArray, queue_size=10)
        
        # Subscribers - Updated to use separate camera topic
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/detected_objects', DetectedObjectArray, self.detection_callback)
        
        rospy.loginfo("Position Estimator initialized")
    
    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters."""
        if self.camera_matrix is None:
            # Camera matrix K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.image_width = msg.width
            self.image_height = msg.height
            rospy.loginfo(f"Camera info received: {msg.width}x{msg.height}")
            rospy.loginfo(f"Camera matrix:\n{self.camera_matrix}")
    
    def pixel_to_3d_simple(self, pixel_x, pixel_y, depth=0.5):
        """
        Convert pixel coordinates to 3D position using simple projection.
        Assumes a fixed depth for estimation.
        
        Args:
            pixel_x: X coordinate in image
            pixel_y: Y coordinate in image
            depth: Assumed depth from camera
        
        Returns:
            Point in camera frame
        """
        if self.camera_matrix is None:
            rospy.logwarn("Camera matrix not yet available")
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Back-project to 3D (in camera optical frame)
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        
        return point
    
    def estimate_depth_from_area(self, area):
        """
        Estimate depth based on detected object area.
        Larger area = closer object.
        This is a rough estimation.
        """
        # Reference: cube at 0.5m appears with area ~2000 pixels
        reference_area = 2000.0
        reference_depth = 0.5
        
        if area <= 0:
            return reference_depth
        
        # Area is inversely proportional to depth squared
        depth = reference_depth * np.sqrt(reference_area / area)
        
        # Clamp to reasonable range
        depth = np.clip(depth, 0.2, 1.0)
        
        return depth
    
    def transform_point_to_base(self, point, source_frame):
        """
        Transform a point from source frame to base_link frame.
        
        Args:
            point: Point in source frame
            source_frame: Frame ID of the point
        
        Returns:
            Point in base_link frame or None if transform fails
        """
        try:
            # Create stamped point
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = rospy.Time(0)  # Use latest available transform
            point_stamped.point = point
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Transform point
            transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return transformed.point
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return None
    
    def estimate_position_with_known_height(self, pixel_x, pixel_y, target_z):
        """
        Estimate 3D position given pixel coordinates and known Z height.
        Uses ray-casting from camera to horizontal plane at target_z.
        """
        if self.camera_matrix is None:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Get camera to base transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn(f"Cannot get camera transform: {e}")
            return None
        
        # Camera position in base frame
        cam_pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        # Ray direction in camera frame (optical convention: Z forward)
        ray_cam = np.array([
            (pixel_x - cx) / fx,
            (pixel_y - cy) / fy,
            1.0
        ])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        # Convert quaternion to rotation matrix
        q = transform.transform.rotation
        R = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        
        # Transform ray to base frame
        ray_base = R @ ray_cam
        
        # Find intersection with horizontal plane at target_z
        # P = cam_pos + t * ray_base
        # target_z = cam_pos[2] + t * ray_base[2]
        if abs(ray_base[2]) < 1e-6:
            rospy.logwarn("Ray is parallel to table plane")
            return None
        
        t = (target_z - cam_pos[2]) / ray_base[2]
        
        if t < 0:
            rospy.logwarn("Target is behind camera")
            return None
        
        # Calculate intersection point
        point = Point()
        point.x = cam_pos[0] + t * ray_base[0]
        point.y = cam_pos[1] + t * ray_base[1]
        point.z = target_z
        
        return point
    
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Convert quaternion to 3x3 rotation matrix."""
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        return R
    
    def detection_callback(self, msg):
        """
        Process detected objects and estimate their 3D positions.
        """
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Waiting for camera info...")
            return
        
        output = Object3DArray()
        output.header = msg.header
        output.header.frame_id = self.base_frame
        
        for det in msg.objects:
            # Try to estimate position using known table height
            pos = self.estimate_position_with_known_height(
                det.pixel_x, det.pixel_y, self.TABLE_HEIGHT
            )
            
            if pos is None:
                # Fallback: use simple projection with estimated depth
                depth = self.estimate_depth_from_area(det.area)
                camera_point = self.pixel_to_3d_simple(det.pixel_x, det.pixel_y, depth)
                
                if camera_point is not None:
                    pos = self.transform_point_to_base(camera_point, self.camera_frame)
            
            if pos is not None:
                obj_3d = Object3D()
                obj_3d.color = det.color
                obj_3d.position = pos
                obj_3d.confidence = 0.8
                output.objects.append(obj_3d)
                
                rospy.loginfo_throttle(2, 
                    f"Detected {det.color} object at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        if output.objects:
            self.position_pub.publish(output)
    
    def run(self):
        """Run the node."""
        rospy.spin()


def main():
    try:
        estimator = PositionEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
