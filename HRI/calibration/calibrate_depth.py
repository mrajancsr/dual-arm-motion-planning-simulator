"""
Depth Calibration Script

Calibrates depth scale by measuring a known distance (e.g., desk at 0.6m).
This scales Depth Anything V2's relative depth output to real meters.
"""

import cv2
import numpy as np
import json
import os
import sys

# Add parent directory to path so we can import from perception module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.depth_estimator import DepthEstimator

def calibrate_depth(camera_id=0, 
                   known_distance=0.6,
                   sample_region_size=50,
                   intrinsics_file="calibration/camera_intrinsics.json"):
    """
    Calibrate depth scale using a known distance.
    
    Args:
        camera_id: Webcam device ID
        known_distance: Known distance in meters (e.g., 0.6m for desk)
        sample_region_size: Size of region to sample for depth (pixels)
        intrinsics_file: Path to calibrated camera intrinsics
        
    Returns:
        Depth scale factor to multiply raw depth values
    """
    print("=" * 60)
    print("Depth Calibration")
    print("=" * 60)
    print(f"Known distance: {known_distance}m")
    print(f"Instructions:")
    print(f"  1. Place an object at a known distance from camera")
    print(f"  2. Measure the distance (e.g., desk at {known_distance}m)")
    print(f"  3. Position object in center of frame")
    print(f"  4. Press SPACE to capture and calibrate")
    print(f"  5. Press 'q' to quit")
    print("=" * 60)
    
    # Load intrinsics if available
    intrinsics = None
    if os.path.exists(intrinsics_file):
        with open(intrinsics_file, 'r') as f:
            calib_data = json.load(f)
            intrinsics = {
                'fx': calib_data['fx'],
                'fy': calib_data['fy'],
                'cx': calib_data['cx'],
                'cy': calib_data['cy']
            }
            print(f"\n✓ Loaded calibrated intrinsics from {intrinsics_file}")
    else:
        print(f"\n⚠ No calibrated intrinsics found at {intrinsics_file}")
        print("  Using estimated intrinsics (calibration may be less accurate)")
    
    # Initialize depth estimator
    print("\nLoading Depth Anything V2...")
    estimator = DepthEstimator()
    
    # Open camera
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    
    # Warm up camera
    print("Warming up camera...")
    for i in range(5):
        ret, _ = cap.read()
        if ret:
            break
        cv2.waitKey(100)
    
    calibrated = False
    depth_scale = None
    
    while not calibrated:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break
        
        # Estimate depth for preview
        depth_map = estimator.estimate_depth(frame)
        
        # Display frame
        h, w = frame.shape[:2]
        center_u, center_v = w // 2, h // 2
        
        display_frame = frame.copy()
        
        # Draw sample region
        half_size = sample_region_size // 2
        cv2.rectangle(display_frame,
                     (center_u - half_size, center_v - half_size),
                     (center_u + half_size, center_v + half_size),
                     (0, 255, 0), 2)
        cv2.circle(display_frame, (center_u, center_v), 5, (0, 255, 0), -1)
        
        # Show depth info if available
        if depth_map is not None:
            # Sample depth in center region
            u_start = max(0, center_u - half_size)
            u_end = min(w, center_u + half_size)
            v_start = max(0, center_v - half_size)
            v_end = min(h, center_v + half_size)
            
            depth_region = depth_map[v_start:v_end, u_start:u_end]
            valid_depths = depth_region[np.isfinite(depth_region) & (depth_region > 0)]
            
            if len(valid_depths) > 0:
                median_depth = np.median(valid_depths)
                mean_depth = np.mean(valid_depths)
                
                cv2.putText(display_frame, f"Raw depth (median): {median_depth:.2f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_frame, f"Known distance: {known_distance:.2f}m", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if depth_scale is not None:
                    scaled_depth = median_depth * depth_scale
                    cv2.putText(display_frame, f"Scaled depth: {scaled_depth:.2f}m", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.putText(display_frame, "SPACE: Calibrate | 'q': Quit", 
                   (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Depth Calibration", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # SPACE to calibrate
            if depth_map is None:
                print("  ⚠ No depth map available. Try again.")
                continue
            
            # Sample depth in center region
            u_start = max(0, center_u - half_size)
            u_end = min(w, center_u + half_size)
            v_start = max(0, center_v - half_size)
            v_end = min(h, center_v + half_size)
            
            depth_region = depth_map[v_start:v_end, u_start:u_end]
            valid_depths = depth_region[np.isfinite(depth_region) & (depth_region > 0)]
            
            if len(valid_depths) == 0:
                print("  ⚠ No valid depth values in center region. Try again.")
                continue
            
            # Use median depth for robustness
            raw_depth = np.median(valid_depths)
            
            # Calculate scale factor
            depth_scale = known_distance / raw_depth
            
            print(f"\n  ✓ Calibration complete!")
            print(f"    Raw depth value: {raw_depth:.2f}")
            print(f"    Known distance: {known_distance:.2f}m")
            print(f"    Depth scale factor: {depth_scale:.6f}")
            print(f"    (Multiply raw depth by {depth_scale:.6f} to get meters)")
            
            # Save calibration
            calibration_data = {
                'depth_scale': float(depth_scale),
                'known_distance': float(known_distance),
                'raw_depth_sample': float(raw_depth),
                'calibration_method': 'known_distance'
            }
            
            calibration_file = "calibration/depth_calibration.json"
            os.makedirs("calibration", exist_ok=True)
            with open(calibration_file, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            print(f"\n  ✓ Depth calibration saved to: {calibration_file}")
            print(f"\n  Next: Run test_pipeline.py - it will use this scale automatically!")
            
            calibrated = True
            
            # Flash green to confirm
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 255, 0), -1)
            cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, display_frame)
            cv2.imshow("Depth Calibration", display_frame)
            cv2.waitKey(1000)
        
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    return depth_scale


if __name__ == "__main__":
    # Adjust known_distance to match your setup
    # Example: desk at 0.6m, wall at 1.5m, etc.
    scale = calibrate_depth(
        camera_id=0,
        known_distance=0.6,  # Adjust this to your known distance in meters
        intrinsics_file="calibration/camera_intrinsics.json"
    )
    
    if scale:
        print(f"\n✓ Depth calibration complete! Scale factor: {scale:.6f}")
    else:
        print("\n✗ Depth calibration cancelled or failed.")

