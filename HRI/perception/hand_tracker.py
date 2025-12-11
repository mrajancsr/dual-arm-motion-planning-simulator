"""
Hand Tracker Module

MediaPipe-based hand tracking for front camera.
Extracts 2D hand keypoints (u, v) from webcam feed.
"""

import cv2
import mediapipe as mp
import numpy as np
import time
from typing import Optional, Tuple


class HandTracker:
    """
    Hand tracker using MediaPipe for real-time hand detection.
    
    Provides interface: get_hand_2d() -> (u, v, confidence)
    """
    
    def __init__(self, camera_id: int = 0, min_detection_confidence: float = 0.5):
        """
        Initialize hand tracker.
        
        Args:
            camera_id: Webcam device ID (default: 0)
            min_detection_confidence: Minimum confidence for hand detection (0.0-1.0)
        """
        self.camera_id = camera_id
        self.cap = None
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # Track one hand
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # FPS tracking
        self.prev_time = time.time()
        self.fps = 0.0
        
        # Last detected hand position
        self.last_hand_pos = None
        self.last_confidence = 0.0
    
    def start(self) -> bool:
        """
        Start webcam capture.
        
        Returns:
            True if camera opened successfully, False otherwise
        """
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.camera_id}")
            return False
        
        # Set camera properties for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"Hand tracker started on camera {self.camera_id}")
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
    
    def get_hand_2d(self, frame: Optional[np.ndarray] = None) -> Optional[Tuple[float, float, float]]:
        """
        Extract hand wrist position from frame.
        
        Args:
            frame: BGR frame (if None, captures new frame)
            
        Returns:
            Tuple of (u, v, confidence) in pixel coordinates, or None if no hand detected
            u, v are pixel coordinates (0-based, top-left origin)
            confidence is detection confidence (0.0-1.0)
        """
        if frame is None:
            frame = self.get_frame()
            if frame is None:
                return None
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame
        results = self.hands.process(rgb_frame)
        
        # Extract wrist position (landmark 0 is wrist)
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]  # Get first hand
            wrist = hand_landmarks.landmark[0]  # Wrist landmark
            
            # Get frame dimensions
            h, w = frame.shape[:2]
            
            # Convert normalized coordinates to pixel coordinates
            u = wrist.x * w
            v = wrist.y * h
            
            # Get detection confidence (use hand presence confidence)
            confidence = 0.8  # MediaPipe doesn't provide per-landmark confidence
            
            self.last_hand_pos = (u, v)
            self.last_confidence = confidence
            
            return (u, v, confidence)
        
        # No hand detected
        self.last_hand_pos = None
        self.last_confidence = 0.0
        return None
    
    def draw_hand(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw hand landmarks and wrist position on frame.
        
        Args:
            frame: BGR frame to draw on
            
        Returns:
            Frame with hand visualization
        """
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        annotated_frame = frame.copy()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    annotated_frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_draw.DrawingSpec(color=(0, 0, 255), thickness=2)
                )
                
                # Highlight wrist (landmark 0)
                h, w = frame.shape[:2]
                wrist = hand_landmarks.landmark[0]
                wrist_x = int(wrist.x * w)
                wrist_y = int(wrist.y * h)
                cv2.circle(annotated_frame, (wrist_x, wrist_y), 10, (255, 0, 0), -1)
        
        return annotated_frame
    
    def update_fps(self):
        """Update FPS counter."""
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time
    
    def get_fps(self) -> float:
        """Get current FPS."""
        return self.fps
    
    def stop(self):
        """Stop webcam capture and cleanup."""
        if self.cap is not None:
            self.cap.release()
        self.hands.close()
        cv2.destroyAllWindows()
        print("Hand tracker stopped")
    
    def run_live(self):
        """
        Run live hand tracking with visualization.
        Press 'q' to quit.
        """
        if not self.start():
            print("ERROR: Failed to start camera")
            return
        
        print("Hand Tracker - Live Mode")
        print("Press 'q' to quit")
        
        # Test camera with a few frames first
        print("Testing camera...")
        test_frames = 0
        for _ in range(5):
            test_frame = self.get_frame()
            if test_frame is not None:
                test_frames += 1
                print(f"  Frame {test_frames}: OK ({test_frame.shape})")
            else:
                print("  Frame capture failed")
            time.sleep(0.1)
        
        if test_frames == 0:
            print("ERROR: Camera not capturing frames. Check camera permissions.")
            self.stop()
            return
        
        print(f"Camera OK ({test_frames}/5 test frames successful)")
        print("Starting live tracking...")
        
        frame_count = 0
        
        while True:
            frame = self.get_frame()
            if frame is None:
                print(f"WARNING: Failed to capture frame (attempt {frame_count + 1})")
                frame_count += 1
                if frame_count > 10:
                    print("ERROR: Too many failed frame captures. Exiting.")
                    break
                time.sleep(0.1)
                continue
            
            frame_count = 0  # Reset counter on successful capture
            
            # Get hand position
            hand_pos = self.get_hand_2d(frame)
            
            # Draw hand landmarks
            annotated_frame = self.draw_hand(frame)
            
            # Update FPS
            self.update_fps()
            
            # Display hand position and FPS
            if hand_pos is not None:
                u, v, conf = hand_pos
                cv2.putText(annotated_frame, 
                           f"Hand: ({u:.0f}, {v:.0f}) conf={conf:.2f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(annotated_frame, "No hand detected",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(annotated_frame, f"FPS: {self.fps:.1f}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show frame
            try:
                cv2.imshow('Hand Tracker', annotated_frame)
            except Exception as e:
                print(f"ERROR displaying frame: {e}")
                break
            
            # Check for quit (wait 30ms for key press)
            # On macOS, cv2.waitKey() may return -1 if no key pressed
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                print("Quit key pressed")
                break
        
        self.stop()


if __name__ == "__main__":
    # Test hand tracker
    tracker = HandTracker()
    tracker.run_live()
