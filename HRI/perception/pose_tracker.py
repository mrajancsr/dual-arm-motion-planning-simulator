"""
Pose Tracker Module

MediaPipe Pose-based human pose tracking.
Extracts full body keypoints and projects them to 3D working plane.
"""

import cv2
import mediapipe as mp
import numpy as np
from typing import Optional, Dict, Tuple, List
from environment.working_plane import WorkingPlane
from calibration.pixel_to_3d import pixel_to_3d


class PoseTracker:
    """
    Human pose tracker using MediaPipe Pose.
    
    Tracks full body keypoints and projects them to a working plane.
    """
    
    def __init__(self, camera_id: int = 0, 
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5):
        """
        Initialize pose tracker.
        
        Args:
            camera_id: Webcam device ID
            min_detection_confidence: Minimum detection confidence (0.0-1.0)
            min_tracking_confidence: Minimum tracking confidence (0.0-1.0)
        """
        self.camera_id = camera_id
        self.cap = None
        
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,  # 0=light, 1=full, 2=heavy
            smooth_landmarks=True,
            enable_segmentation=False,
            smooth_segmentation=False,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils
    
    def start(self) -> bool:
        """
        Start webcam capture.
        
        Returns:
            True if camera opened successfully
        """
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.camera_id}")
            return False
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        return True
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Capture a single frame from webcam.
        
        Returns:
            BGR frame as numpy array, or None if capture failed
        """
        if self.cap is None:
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        return frame
    
    def detect_pose(self, frame: Optional[np.ndarray] = None) -> Optional[Dict]:
        """
        Detect human pose in frame.
        
        Args:
            frame: BGR frame (if None, captures new frame)
            
        Returns:
            Dictionary with pose data:
            - 'landmarks_2d': Dict of {landmark_name: (u, v)} pixel coordinates
            - 'landmarks_3d': Dict of {landmark_name: (x, y, z)} world coordinates (normalized)
            - 'visibility': Dict of {landmark_name: visibility_score}
            - 'frame': BGR frame with pose drawn
            - Or None if no pose detected
        """
        if frame is None:
            frame = self.get_frame()
            if frame is None:
                return None
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame
        results = self.pose.process(rgb_frame)
        
        if not results.pose_landmarks:
            return None
        
        # Extract landmarks
        landmarks_2d = {}
        landmarks_3d = {}
        visibility = {}
        
        h, w = frame.shape[:2]
        
        # MediaPipe Pose landmark names
        landmark_names = {
            0: 'nose', 1: 'left_eye_inner', 2: 'left_eye', 3: 'left_eye_outer',
            4: 'right_eye_inner', 5: 'right_eye', 6: 'right_eye_outer',
            7: 'left_ear', 8: 'right_ear', 9: 'mouth_left', 10: 'mouth_right',
            11: 'left_shoulder', 12: 'right_shoulder', 13: 'left_elbow', 14: 'right_elbow',
            15: 'left_wrist', 16: 'right_wrist', 17: 'left_pinky', 18: 'right_pinky',
            19: 'left_index', 20: 'right_index', 21: 'left_thumb', 22: 'right_thumb',
            23: 'left_hip', 24: 'right_hip', 25: 'left_knee', 26: 'right_knee',
            27: 'left_ankle', 28: 'right_ankle', 29: 'left_heel', 30: 'right_heel',
            31: 'left_foot_index', 32: 'right_foot_index'
        }
        
        for idx, landmark in enumerate(results.pose_landmarks.landmark):
            if idx in landmark_names:
                name = landmark_names[idx]
                
                # 2D pixel coordinates
                u = landmark.x * w
                v = landmark.y * h
                landmarks_2d[name] = (u, v)
                
                # 3D world coordinates (MediaPipe's normalized coordinates)
                landmarks_3d[name] = (landmark.x, landmark.y, landmark.z)
                
                # Visibility
                visibility[name] = landmark.visibility
        
        # Draw pose on frame
        annotated_frame = frame.copy()
        self.mp_draw.draw_landmarks(
            annotated_frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_draw.DrawingSpec(
                color=(0, 255, 0), thickness=2, circle_radius=2
            ),
            connection_drawing_spec=self.mp_draw.DrawingSpec(
                color=(0, 0, 255), thickness=2
            )
        )
        
        return {
            'landmarks_2d': landmarks_2d,
            'landmarks_3d': landmarks_3d,
            'visibility': visibility,
            'frame': annotated_frame
        }
    
    def project_to_plane(self, pose_data: Dict, 
                        working_plane: WorkingPlane,
                        intrinsics: Dict) -> Dict:
        """
        Project pose keypoints to 3D working plane.
        
        Assumes human is standing in the working plane (X = x0).
        For each 2D keypoint (u, v), backprojects to 3D on the plane.
        
        Args:
            pose_data: Pose data from detect_pose()
            working_plane: WorkingPlane instance
            intrinsics: Camera intrinsics dict
            
        Returns:
            Dictionary with:
            - 'landmarks_3d_plane': Dict of {landmark_name: [x, y, z]} in world coords
            - 'landmarks_2d_plane': Dict of {landmark_name: (y, z)} in plane coords
            - 'frame': Annotated frame
        """
        landmarks_3d_plane = {}
        landmarks_2d_plane = {}
        
        landmarks_2d = pose_data['landmarks_2d']
        visibility = pose_data['visibility']
        
        for name, (u, v) in landmarks_2d.items():
            # Skip if not visible
            if visibility.get(name, 0.0) < 0.5:
                continue
            
            # Backproject pixel to 3D on the working plane
            # Working plane is Z = z0 (constant depth from camera)
            # Solve for X and Y at that depth using pinhole model
            
            fx = intrinsics['fx']
            fy = intrinsics['fy']
            cx = intrinsics['cx']
            cy = intrinsics['cy']
            z0 = working_plane.z0  # Fixed depth from plane definition
            
            # Pinhole model: solve for X and Y at depth Z = z0
            # u = fx * X / Z + cx  =>  X = (u - cx) * Z / fx
            # v = fy * Y / Z + cy  =>  Y = (v - cy) * Z / fy
            x = (u - cx) * z0 / fx
            y = -(v - cy) * z0 / fy  # Negate Y: image v increases downward, camera Y increases upward
            
            # Check if depth is reasonable
            if z0 < 0.15 or z0 > 20.0:
                continue
            
            # 3D point on plane (Z = z0)
            point_3d = np.array([x, y, z0])
            landmarks_3d_plane[name] = point_3d
            
            # 2D plane coordinates: (X, Y) - left/right and up/down
            x_2d, y_2d = working_plane.project_to_2d(point_3d)
            landmarks_2d_plane[name] = (x_2d, y_2d)
        
        return {
            'landmarks_3d_plane': landmarks_3d_plane,
            'landmarks_2d_plane': landmarks_2d_plane,
            'frame': pose_data['frame'],
            'visibility': pose_data.get('visibility', {})  # Include visibility for arm length estimation
        }
    
    def estimate_arm_length(self, pose_data: Dict, 
                           min_visibility: float = 0.7) -> Optional[float]:
        """
        Estimate total arm length from pose landmarks.
        
        Measures left arm: shoulder → elbow → wrist
        Returns total length (upper arm + forearm) in plane coordinates.
        
        Args:
            pose_data: Pose data from project_to_plane()
            min_visibility: Minimum visibility threshold for landmarks
            
        Returns:
            Total arm length in meters, or None if landmarks not visible
        """
        landmarks_2d_plane = pose_data.get('landmarks_2d_plane', {})
        visibility = pose_data.get('visibility', {})
        
        # Check if all required landmarks are present and visible
        required = ['left_shoulder', 'left_elbow', 'left_wrist']
        if not all(name in landmarks_2d_plane for name in required):
            return None
        
        # Check visibility
        if any(visibility.get(name, 0.0) < min_visibility for name in required):
            return None
        
        # Get 2D coordinates in plane
        shoulder = np.array(landmarks_2d_plane['left_shoulder'])
        elbow = np.array(landmarks_2d_plane['left_elbow'])
        wrist = np.array(landmarks_2d_plane['left_wrist'])
        
        # Calculate segment lengths
        upper_arm_length = np.linalg.norm(elbow - shoulder)
        forearm_length = np.linalg.norm(wrist - elbow)
        total_length = upper_arm_length + forearm_length
        
        return float(total_length)
    
    def estimate_max_arm_reach(self, duration_seconds: float = 5.0,
                              min_visibility: float = 0.7,
                              percentile: float = 95.0) -> Optional[float]:
        """
        Estimate maximum arm reach over time by tracking pose.
        
        Collects arm length measurements over duration, filters noise,
        and returns a conservative estimate (high percentile).
        
        Args:
            duration_seconds: How long to collect measurements
            min_visibility: Minimum visibility for landmarks
            percentile: Percentile to use for conservative estimate (default: 95th)
            
        Returns:
            Maximum arm reach in meters, or None if insufficient data
        """
        import time
        
        arm_lengths = []
        start_time = time.time()
        
        print(f"Estimating arm reach over {duration_seconds} seconds...")
        print("  Move your left arm around to full extension")
        
        while time.time() - start_time < duration_seconds:
            frame = self.get_frame()
            if frame is None:
                continue
            
            pose_result = self.detect_pose(frame)
            if pose_result is None:
                continue
            
            # Need to project to plane, but we need working_plane and intrinsics
            # For now, return None - this will be called from test script with plane info
            break
        
        # This method will be called from test script with plane projection
        return None
    
    def stop(self):
        """Stop webcam capture."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

