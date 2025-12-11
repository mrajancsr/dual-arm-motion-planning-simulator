"""
Camera Calibration Script

Calibrates camera using checkerboard images to get real intrinsics.
"""

import cv2
import numpy as np
import glob
import os
import json

def calibrate_camera(images_dir="calibration_images", 
                    pattern_size=(9, 6),
                    square_size=0.025,  # Size of one square in meters (adjust if needed)
                    output_file="camera_intrinsics.json"):
    """
    Calibrate camera using checkerboard images.
    
    Args:
        images_dir: Directory containing calibration images (calib_*.jpg)
        pattern_size: Tuple of (inner_corners_width, inner_corners_height)
                     For a 10x7 checkerboard, inner corners = (9, 6)
        square_size: Size of one checkerboard square in meters (default: 2.5cm = 0.025m)
        output_file: Output JSON file to save intrinsics
        
    Returns:
        Dictionary with 'fx', 'fy', 'cx', 'cy', 'distortion', 'K' (camera matrix)
    """
    print("=" * 60)
    print("Camera Calibration")
    print("=" * 60)
    print(f"Images directory: {images_dir}")
    print(f"Checkerboard pattern: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print(f"Square size: {square_size}m")
    print("=" * 60)
    
    # Prepare object points (3D points in checkerboard coordinate system)
    # For a flat checkerboard, Z=0 for all points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size  # Scale to real-world units (meters)
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    # Find all calibration images
    images = glob.glob(os.path.join(images_dir, "calib_*.jpg"))
    images.sort()
    
    if len(images) == 0:
        print(f"ERROR: No calibration images found in {images_dir}/")
        print("  Expected files: calib_01.jpg, calib_02.jpg, ...")
        return None
    
    print(f"\nFound {len(images)} calibration images")
    print("Processing images...")
    
    successful = 0
    failed = 0
    
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"  ⚠ Could not read: {fname}")
            failed += 1
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + 
            cv2.CALIB_CB_FAST_CHECK + 
            cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if ret:
            # Refine corner positions for sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners2)
            successful += 1
            print(f"  ✓ {os.path.basename(fname)} - Found {pattern_size[0]*pattern_size[1]} corners")
        else:
            failed += 1
            print(f"  ✗ {os.path.basename(fname)} - Could not find checkerboard")
    
    print(f"\nSuccessful: {successful}/{len(images)}")
    print(f"Failed: {failed}/{len(images)}")
    
    if successful < 10:
        print("\n⚠ WARNING: Less than 10 successful images. Calibration may be inaccurate.")
        print("  Try capturing more images with the checkerboard clearly visible.")
    
    if successful == 0:
        print("\nERROR: No images with detected checkerboards!")
        print("  Check:")
        print("    - Checkerboard pattern size is correct")
        print("    - Images show the full checkerboard clearly")
        print("    - Checkerboard is flat and in focus")
        return None
    
    # Get image dimensions
    img_shape = gray.shape[::-1]  # (width, height)
    
    print(f"\nRunning calibration with {successful} images...")
    print(f"Image size: {img_shape[0]}x{img_shape[1]}")
    
    # Calibrate camera
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    
    if not ret:
        print("ERROR: Calibration failed!")
        return None
    
    # Extract intrinsics
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    
    # Calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    mean_error /= len(objpoints)
    
    # Prepare results
    intrinsics = {
        'fx': float(fx),
        'fy': float(fy),
        'cx': float(cx),
        'cy': float(cy),
        'width': int(img_shape[0]),
        'height': int(img_shape[1]),
        'distortion': dist.tolist(),
        'camera_matrix': K.tolist(),
        'reprojection_error': float(mean_error)
    }
    
    # Print results
    print("\n" + "=" * 60)
    print("Calibration Results")
    print("=" * 60)
    print(f"Focal length (fx): {fx:.2f} pixels")
    print(f"Focal length (fy): {fy:.2f} pixels")
    print(f"Principal point (cx): {cx:.2f} pixels")
    print(f"Principal point (cy): {cy:.2f} pixels")
    print(f"Image size: {img_shape[0]}x{img_shape[1]}")
    print(f"Reprojection error: {mean_error:.4f} pixels (lower is better)")
    print("=" * 60)
    
    if mean_error > 1.0:
        print("\n⚠ WARNING: High reprojection error. Calibration may be inaccurate.")
        print("  Try:")
        print("    - More calibration images (15-20 recommended)")
        print("    - Better checkerboard images (sharper, better lighting)")
        print("    - More varied angles and distances")
    
    # Save to file (save in calibration/ directory)
    output_path = os.path.join("calibration", output_file)
    os.makedirs("calibration", exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(intrinsics, f, indent=2)
    
    print(f"\n✓ Intrinsics saved to: {output_path}")
    print("\nNext steps:")
    print("  1. Update depth_estimator.py to load these intrinsics")
    print("  2. Run depth calibration (calibrate_depth.py) to scale depth values")
    
    return intrinsics


if __name__ == "__main__":
    # Adjust these parameters if needed:
    # - pattern_size: (inner_corners_width, inner_corners_height)
    #   For a 10x7 checkerboard (10 squares x 7 squares), inner corners = (9, 6)
    # - square_size: Size of one square in meters (e.g., 0.025 = 2.5cm)
    
    intrinsics = calibrate_camera(
        images_dir="calibration_images",
        pattern_size=(9, 6),  # Adjust to your checkerboard
        square_size=0.025,     # Adjust to your square size in meters
        output_file="camera_intrinsics.json"
    )
    
    if intrinsics:
        print("\n✓ Calibration complete!")
    else:
        print("\n✗ Calibration failed. Check the errors above.")

