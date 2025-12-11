"""
Depth Estimator Module

Monocular depth estimation using Depth Anything V2.
Converts RGB frames to depth maps.
"""

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
from PIL import Image
from typing import Optional, Tuple
import time


class DepthEstimator:
    """
    Depth estimation using Depth Anything V2 from HuggingFace.
    
    Converts RGB images to depth maps in meters.
    """
    
    def __init__(self, model_name: str = "depth-anything/Depth-Anything-V2-Large-hf", device: Optional[str] = None):
        """
        Initialize depth estimator.
        
        Args:
            model_name: HuggingFace model name for Depth Anything V2
            device: Device to run on ('cpu', 'mps' for M1, or None for auto)
        """
        self.model_name = model_name
        
        # Auto-detect device for M1 Mac
        if device is None:
            if torch.backends.mps.is_available():
                self.device = torch.device("mps")  # Apple Silicon GPU
                print("Using MPS (Apple Silicon GPU)")
            elif torch.cuda.is_available():
                self.device = torch.device("cuda")
                print("Using CUDA")
            else:
                self.device = torch.device("cpu")
                print("Using CPU")
        else:
            self.device = torch.device(device)
        
        self.model = None
        self.processor = None
        self._load_model()
    
    def _load_model(self, max_retries: int = 3):
        """Load Depth Anything V2 model from HuggingFace with retry logic."""
        for attempt in range(max_retries):
            try:
                print(f"Loading Depth Anything V2 from HuggingFace: {self.model_name}... (attempt {attempt + 1}/{max_retries})")
                
                # Load processor and model from HuggingFace
                self.processor = AutoImageProcessor.from_pretrained(self.model_name)
                self.model = AutoModelForDepthEstimation.from_pretrained(self.model_name)
                self.model.to(self.device)
                self.model.eval()
                
                print(f"Depth Anything V2 loaded successfully on {self.device}")
                return
                
            except Exception as e:
                if attempt < max_retries - 1:
                    wait_time = (attempt + 1) * 5  # 5s, 10s, 15s
                    print(f"Error loading model (attempt {attempt + 1}): {e}")
                    print(f"Retrying in {wait_time} seconds...")
                    time.sleep(wait_time)
                else:
                    print(f"Error loading Depth Anything V2 after {max_retries} attempts: {e}")
                    print("\nTroubleshooting:")
                    print("1. Check your internet connection")
                    print("2. HuggingFace may be slow - try again in a few minutes")
                    print("3. First-time download requires ~500MB - ensure stable connection")
                    raise
    
    def estimate_depth(self, rgb_image: np.ndarray) -> Optional[np.ndarray]:
        """
        Estimate depth map from RGB image.
        
        Args:
            rgb_image: RGB image as numpy array (H, W, 3) in BGR format (from OpenCV)
            
        Returns:
            Depth map as numpy array (H, W) in meters, or None if failed
        """
        if self.model is None or self.processor is None:
            return None
        
        try:
            # Convert BGR to RGB
            rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            pil_image = Image.fromarray(rgb)
            
            # Process image
            inputs = self.processor(images=pil_image, return_tensors="pt")
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            
            # Run inference
            with torch.no_grad():
                outputs = self.model(**inputs)
                predicted_depth = outputs.predicted_depth
            
            # Interpolate to original image size
            prediction = torch.nn.functional.interpolate(
                predicted_depth.unsqueeze(1),
                size=rgb.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
            
            # Convert to numpy - Depth Anything V2 outputs depth in meters
            depth_map = prediction.cpu().numpy()
            
            # Depth Anything V2 outputs metric depth directly, no normalization needed
            return depth_map
            
        except Exception as e:
            print(f"Error estimating depth: {e}")
            return None
    
    def capture_depth_snapshot(self, camera_id: int = 0, 
                              save_path: str = "depth_snapshot",
                              fx: Optional[float] = None,
                              fy: Optional[float] = None,
                              cx: Optional[float] = None,
                              cy: Optional[float] = None,
                              intrinsics_file: Optional[str] = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], dict]:
        """
        Capture a depth snapshot and save to disk for reuse.
        
        Args:
            camera_id: Webcam device ID
            save_path: Base path for saving files (will save {save_path}_depth.npy and {save_path}_intrinsics.npy)
            fx, fy: Focal lengths (if None, estimated from image size)
            cx, cy: Principal point (if None, image center)
            
        Returns:
            Tuple of (rgb_frame, depth_map, intrinsics_dict) or (None, None, None) if failed
            intrinsics_dict contains: {'fx': float, 'fy': float, 'cx': float, 'cy': float}
        """
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"Error: Could not open camera {camera_id}")
            return None, None, None
        
        # Warm up camera - first few frames are often black/garbage on macOS
        print("Warming up camera...")
        for i in range(5):
            ret, _ = cap.read()
            if not ret:
                print(f"Warning: failed to grab frame during warmup ({i+1}/5)")
                continue
            time.sleep(0.1)  # Small delay between frames
        
        # Now capture the actual frame to use
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print("Error: Failed to capture frame after warmup")
            return None, None, None
        
        # Estimate depth
        depth = self.estimate_depth(frame)
        if depth is None:
            return None, None, None
        
        # Get image dimensions
        h, w = frame.shape[:2]
        
        # Try to load calibrated intrinsics first
        if intrinsics_file is not None:
            try:
                import json
                with open(intrinsics_file, 'r') as f:
                    calib_intrinsics = json.load(f)
                    fx = calib_intrinsics['fx']
                    fy = calib_intrinsics['fy']
                    cx = calib_intrinsics['cx']
                    cy = calib_intrinsics['cy']
                    print(f"  Loaded calibrated intrinsics from {intrinsics_file}")
            except Exception as e:
                print(f"  Warning: Could not load intrinsics from {intrinsics_file}: {e}")
                print(f"  Falling back to estimated intrinsics")
        
        # Estimate camera intrinsics if not provided
        if fx is None:
            fx = w * 0.7  # Rough estimate: focal length ~70% of image width
        if fy is None:
            fy = h * 0.7
        if cx is None:
            cx = w / 2.0
        if cy is None:
            cy = h / 2.0
        
        intrinsics = {
            'fx': float(fx),
            'fy': float(fy),
            'cx': float(cx),
            'cy': float(cy),
            'width': int(w),
            'height': int(h)
        }
        
        # Save to disk
        depth_path = f"{save_path}_depth.npy"
        intrinsics_path = f"{save_path}_intrinsics.npy"
        
        np.save(depth_path, depth)
        np.save(intrinsics_path, intrinsics)
        
        print(f"Depth snapshot saved:")
        print(f"  Depth: {depth_path}")
        print(f"  Intrinsics: {intrinsics_path}")
        print(f"  Depth shape: {depth.shape}, range: {depth.min():.2f}m - {depth.max():.2f}m")
        print(f"  Intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        
        return frame, depth, intrinsics
    
    def visualize_depth(self, depth_map: np.ndarray, colormap: str = "plasma") -> np.ndarray:
        """
        Visualize depth map as colored image.
        
        Note: This normalizes depth ONLY for visualization. The input depth_map
        should be in metric units (from estimate_depth), and normalization here
        does not affect the actual depth values used for obstacle extraction.
        
        Args:
            depth_map: Depth map (H, W) in meters
            colormap: Colormap name ('plasma', 'viridis', 'inferno', 'jet')
            
        Returns:
            Colored depth visualization (H, W, 3)
        """
        # Normalize depth for visualization ONLY (0-255)
        # This normalization is just for display - doesn't affect metric depth
        depth_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min() + 1e-6)
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)
        
        # Apply colormap
        if colormap == "plasma":
            colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_PLASMA)
        elif colormap == "viridis":
            colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_VIRIDIS)
        elif colormap == "inferno":
            colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_INFERNO)
        else:  # jet
            colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
        
        return colored


if __name__ == "__main__":
    # Test depth estimator - show raw Depth Anything V2 output
    print("Testing Depth Estimator (Depth Anything V2)")
    print("=" * 50)
    
    estimator = DepthEstimator()
    
    print("\nCapturing frame from webcam...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        exit(1)
    
    # Warm up camera
    print("Warming up camera...")
    for i in range(5):
        ret, _ = cap.read()
        if not ret:
            print(f"Warning: failed to grab frame during warmup ({i+1}/5)")
            continue
        time.sleep(0.1)
    
    ret, rgb_frame = cap.read()
    cap.release()
    
    if not ret:
        print("Error: Failed to capture frame")
        exit(1)
    
    print(f"RGB frame shape: {rgb_frame.shape}")
    
    # Get raw depth map from model
    print("\nRunning Depth Anything V2 inference...")
    depth_map = estimator.estimate_depth(rgb_frame)
    
    if depth_map is not None:
        print(f"Depth map shape: {depth_map.shape}")
        print(f"Depth data type: {depth_map.dtype}")
        print(f"Depth range (raw): {depth_map.min():.6f} to {depth_map.max():.6f}")
        print(f"Depth mean: {depth_map.mean():.6f}")
        print(f"Depth std: {depth_map.std():.6f}")
        print(f"Valid pixels (>0): {np.sum(depth_map > 0)} / {depth_map.size}")
        print(f"Negative values: {np.sum(depth_map < 0)}")
        print(f"Zero values: {np.sum(depth_map == 0)}")
        
        # Show raw depth statistics
        valid_depths = depth_map[depth_map > 0]
        if len(valid_depths) > 0:
            print(f"\nValid depth statistics:")
            print(f"  Min: {valid_depths.min():.6f}")
            print(f"  Max: {valid_depths.max():.6f}")
            print(f"  Mean: {valid_depths.mean():.6f}")
            print(f"  Median: {np.median(valid_depths):.6f}")
            print(f"  25th percentile: {np.percentile(valid_depths, 25):.6f}")
            print(f"  75th percentile: {np.percentile(valid_depths, 75):.6f}")
        
        # Visualize raw depth (normalized for display)
        depth_viz = estimator.visualize_depth(depth_map)
        
        # Show side-by-side
        combined = np.hstack([rgb_frame, depth_viz])
        cv2.namedWindow("RGB | Depth (Raw Output)", cv2.WINDOW_NORMAL)
        cv2.imshow("RGB | Depth (Raw Output)", combined)
        
        # Also save the raw depth map
        np.save("raw_depth_output.npy", depth_map)
        print(f"\nRaw depth map saved to: raw_depth_output.npy")
        print("You can load it with: depth = np.load('raw_depth_output.npy')")
        
        print("\nPress 'q' or close window to exit...")
        
        # Wait for key press or window close
        while True:
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            # Check if window was closed
            if cv2.getWindowProperty("RGB | Depth (Raw Output)", cv2.WND_PROP_VISIBLE) < 1:
                break
        
        # Clean shutdown
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Flush events
        print("Exiting cleanly...")
    else:
        print("Failed to estimate depth")
