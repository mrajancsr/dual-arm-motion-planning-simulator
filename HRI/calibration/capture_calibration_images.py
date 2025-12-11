"""
Capture Calibration Images

Simple script to capture checkerboard images for camera calibration.
Press SPACE to capture an image, 'q' to quit.
"""

import cv2
import os

def capture_calibration_images(camera_id=0, output_dir="calibration_images", num_images=20):
    """
    Capture checkerboard images for camera calibration.
    
    Args:
        camera_id: Webcam device ID (default: 0)
        output_dir: Directory to save images
        num_images: Number of images to capture
    """
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Open camera
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return
    
    # Set camera resolution (optional, adjust if needed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    print("=" * 60)
    print("Checkerboard Calibration Image Capture")
    print("=" * 60)
    print(f"Camera: {camera_id}")
    print(f"Output directory: {output_dir}")
    print(f"Target images: {num_images}")
    print("\nInstructions:")
    print("  - Show checkerboard to camera")
    print("  - Press SPACE to capture an image")
    print("  - Press 'q' to quit")
    print("  - Try different angles and distances")
    print("  - Cover all corners of the frame across images")
    print("=" * 60)
    
    image_count = 0
    
    # Checkerboard pattern size (adjust to match your checkerboard)
    # For a 10x7 checkerboard (10 squares x 7 squares), inner corners = (9, 6)
    pattern_size = (9, 6)  # (inner_corners_width, inner_corners_height)
    
    print(f"\nCheckerboard pattern: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print("(Adjust pattern_size in the script if your checkerboard is different)")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break
        
        # Try to detect checkerboard in real-time (for visual feedback)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_corners, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        )
        
        # Display frame with instructions
        display_frame = frame.copy()
        
        # Draw checkerboard corners if detected
        if ret_corners:
            cv2.drawChessboardCorners(display_frame, pattern_size, corners, ret_corners)
            status_text = "✓ Checkerboard detected - Ready to capture!"
            status_color = (0, 255, 0)  # Green
        else:
            status_text = "✗ Checkerboard not detected - Move checkerboard into view"
            status_color = (0, 0, 255)  # Red
        
        cv2.putText(display_frame, f"Images captured: {image_count}/{num_images}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display_frame, status_text, 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(display_frame, "SPACE: Capture | 'q': Quit", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Calibration Capture", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # SPACE to capture
            # Only capture if checkerboard is detected
            if ret_corners:
                filename = os.path.join(output_dir, f"calib_{image_count+1:02d}.jpg")
                cv2.imwrite(filename, frame)
                image_count += 1
                print(f"  ✓ Captured: {filename} ({image_count}/{num_images})")
                
                # Flash green overlay to confirm capture
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), (0, 255, 0), -1)
                cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, display_frame)
                cv2.imshow("Calibration Capture", display_frame)
                cv2.waitKey(200)  # Brief flash
                
                if image_count >= num_images:
                    print(f"\n✓ Captured {num_images} images. You can quit now or capture more.")
            else:
                print("  ⚠ Checkerboard not detected! Move checkerboard into view and try again.")
                # Flash red overlay to indicate failure
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 255), -1)
                cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, display_frame)
                cv2.imshow("Calibration Capture", display_frame)
                cv2.waitKey(300)  # Brief flash
        
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print(f"\n✓ Calibration capture complete!")
    print(f"  Total images: {image_count}")
    print(f"  Saved to: {output_dir}/")
    print(f"\nNext step: Run calibrate_camera.py")


if __name__ == "__main__":
    capture_calibration_images()

