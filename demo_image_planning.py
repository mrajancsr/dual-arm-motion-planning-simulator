#!/usr/bin/env python3
"""
Image-Based RRT* Path Planning

Upload an image of a workspace with obstacles, start/goal markers, and plan a path.
- Auto-detects obstacles (dark regions)
- Auto-detects start (green marker) and goal (red marker)
- Runs RRT* planning overlaid on the image
- Animates the arm moving over the image
"""

import numpy as np
import sys
from src import DualArm, TwoLinkArm, RRTStar, ThreeLinkArm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from PIL import Image
import scipy.ndimage as ndimage


# Workspace dimensions (hardcoded - assumes all images represent this size)
WORKSPACE_WIDTH = 5.0  # meters
WORKSPACE_HEIGHT = 5.0  # meters


def load_image():
    """Load an image file from path or prompt user."""
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        file_path = input("Enter path to workspace image: ").strip()
        if not file_path:
            return None
    
    try:
        img = Image.open(file_path)
        return np.array(img)
    except Exception as e:
        print(f"Error loading image: {e}")
        return None


def pixel_to_workspace(pixel_x, pixel_y, img_shape, workspace_width, workspace_height):
    """Convert pixel coordinates to workspace coordinates (meters)."""
    img_height, img_width = img_shape[:2]
    
    # Map pixel (0,0) at top-left to workspace (0,0) at bottom-left
    # Assuming image origin is at workspace center
    x = (pixel_x / img_width) * workspace_width - workspace_width / 2
    y = workspace_height / 2 - (pixel_y / img_height) * workspace_height
    
    return np.array([x, y])


def detect_obstacles(img, threshold=0.3, min_size=50):
    """
    Detect obstacles in image using thresholding.
    
    Args:
        img: Image array (H, W, 3) or (H, W)
        threshold: Threshold value (0-1) for dark regions
        min_size: Minimum obstacle size in pixels
        
    Returns:
        List of obstacles: [{'type': 'circle', 'center': [x, y], 'radius': r}, ...]
    """
    # Convert to grayscale if needed
    if len(img.shape) == 3:
        gray = np.mean(img, axis=2) / 255.0
    else:
        gray = img / 255.0
    
    # Threshold to find dark regions (obstacles)
    binary = gray < threshold
    
    # Remove small noise
    binary = ndimage.binary_opening(binary, structure=np.ones((3, 3)))
    binary = ndimage.binary_closing(binary, structure=np.ones((5, 5)))
    
    # Label connected components
    labeled, num_features = ndimage.label(binary)
    
    obstacles = []
    img_height, img_width = img.shape[:2]
    
    for i in range(1, num_features + 1):
        region = (labeled == i)
        region_size = np.sum(region)
        
        if region_size < min_size:
            continue
        
        # Find centroid
        y_coords, x_coords = np.where(region)
        center_pixel_x = np.mean(x_coords)
        center_pixel_y = np.mean(y_coords)
        
        # Convert to workspace coordinates
        center_ws = pixel_to_workspace(center_pixel_x, center_pixel_y, img.shape, 
                                       WORKSPACE_WIDTH, WORKSPACE_HEIGHT)
        
        # Estimate radius (use max distance from center to edge)
        distances = np.sqrt((x_coords - center_pixel_x)**2 + (y_coords - center_pixel_y)**2)
        radius_pixels = np.max(distances)
        
        # Convert radius to meters
        radius_meters = (radius_pixels / img_width) * WORKSPACE_WIDTH
        
        obstacles.append({
            'type': 'circle',
            'center': center_ws.tolist(),
            'radius': max(radius_meters, 0.1)  # Minimum 10cm radius
        })
    
    return obstacles


def detect_color_marker(img, color='green', threshold=0.5):
    """
    Detect colored marker (green for start, red for goal).
    
    Args:
        img: Image array (H, W, 3)
        color: 'green' or 'red'
        threshold: Minimum color intensity (0-1), lowered for robustness
        
    Returns:
        [x, y] in workspace coordinates, or None if not found
    """
    if len(img.shape) != 3:
        return None
    
    # Normalize to 0-1
    img_norm = img.astype(float) / 255.0
    
    if color == 'green':
        # Green channel should be high, red and blue low
        color_mask = (img_norm[:, :, 1] > threshold) & \
                     (img_norm[:, :, 0] < 0.5) & \
                     (img_norm[:, :, 2] < 0.5)
    elif color == 'red':
        # Red channel should be high, green and blue low
        color_mask = (img_norm[:, :, 0] > threshold) & \
                     (img_norm[:, :, 1] < 0.5) & \
                     (img_norm[:, :, 2] < 0.5)
    else:
        return None
    
    if not np.any(color_mask):
        return None
    
    # Find centroid of colored region
    y_coords, x_coords = np.where(color_mask)
    center_pixel_x = np.mean(x_coords)
    center_pixel_y = np.mean(y_coords)
    
    # Convert to workspace coordinates
    center_ws = pixel_to_workspace(center_pixel_x, center_pixel_y, img.shape,
                                   WORKSPACE_WIDTH, WORKSPACE_HEIGHT)
    
    return center_ws


def compute_all_joint_positions(arm, base, joint_angles):
    """Compute all joint positions for an arm."""
    positions = [base.copy()]
    current_pos = base.copy()
    cumulative_angle = 0.0
    
    if hasattr(arm, 'link_lengths'):
        for length, angle in zip(arm.link_lengths, joint_angles):
            cumulative_angle += float(angle)
            current_pos = current_pos + np.array([
                length * np.cos(cumulative_angle),
                length * np.sin(cumulative_angle)
            ])
            positions.append(current_pos.copy())
    elif hasattr(arm, 'L1') and hasattr(arm, 'L2') and len(joint_angles) == 2:
        theta1, theta2 = joint_angles[0], joint_angles[1]
        joint = current_pos + np.array([arm.L1 * np.cos(theta1), arm.L1 * np.sin(theta1)])
        positions.append(joint)
        end_eff = joint + np.array([
            arm.L2 * np.cos(theta1 + theta2),
            arm.L2 * np.sin(theta1 + theta2)
        ])
        positions.append(end_eff)
    else:
        end_eff_pos = arm.forward_kinematics(*joint_angles)
        positions.append(base + end_eff_pos)
    
    return positions


def interpolate_path_detailed(path, points_per_segment=15):
    """Interpolate path for smooth animation."""
    if len(path) < 2:
        return path
    
    detailed_path = [path[0]]
    
    for i in range(len(path) - 1):
        config_start = path[i]
        config_end = path[i + 1]
        
        for t in np.linspace(0, 1, points_per_segment + 1)[1:]:
            interpolated = config_start + t * (config_end - config_start)
            detailed_path.append(interpolated)
    
    return detailed_path


def try_fix_collision(left_config, dual_arm, right_config, arm, base, target_ee, attempts=200):
    """Try to fix colliding config while keeping end-effector close to target."""
    # Strategy 1: Try small perturbations to existing config
    best_config = None
    best_error = float('inf')
    
    for scale in [0.05, 0.1, 0.15, 0.2, 0.3, 0.4]:
        for _ in range(attempts // 6):
            perturbed = left_config + np.random.uniform(-scale, scale, size=left_config.shape)
            test_config = np.concatenate([perturbed, right_config])
            
            if dual_arm.is_valid_configuration(test_config):
                # Check how far end-effector moved from target
                ee_pos = base + arm.forward_kinematics(*perturbed)
                error = np.linalg.norm(ee_pos - target_ee)
                
                if error < best_error:
                    best_error = error
                    best_config = test_config
    
    # If we found a valid config, return the one closest to target
    if best_config is not None:
        return best_config
    
    # Strategy 2: Try solving IK with slightly offset goal positions
    # Try offsets in a circle around the target
    for offset_dist in [0.05, 0.1, 0.15, 0.2, 0.25]:
        # Try 8 directions around the circle
        for angle in np.linspace(0, 2*np.pi, 16, endpoint=False):
            offset_dir = np.array([np.cos(angle), np.sin(angle)]) * offset_dist
            new_target = target_ee + offset_dir
            
            # Try solving IK for this offset position
            new_config = solve_left_arm_ik(arm, new_target, base, dual_arm, right_config, max_attempts=20)
            if new_config is not None:
                test_config = np.concatenate([new_config, right_config])
                if dual_arm.is_valid_configuration(test_config):
                    # Check how close we got to original target
                    ee_pos = base + arm.forward_kinematics(*new_config)
                    error = np.linalg.norm(ee_pos - target_ee)
                    if error < best_error or best_config is None:
                        best_error = error
                        best_config = test_config
    
    # Strategy 3: Try solving IK with goal slightly above obstacles (if goal is low)
    if target_ee[1] < 1.0:  # If goal is in lower half
        for height_offset in [0.1, 0.2, 0.3]:
            new_target = target_ee.copy()
            new_target[1] += height_offset  # Move up
            new_config = solve_left_arm_ik(arm, new_target, base, dual_arm, right_config, max_attempts=20)
            if new_config is not None:
                test_config = np.concatenate([new_config, right_config])
                if dual_arm.is_valid_configuration(test_config):
                    ee_pos = base + arm.forward_kinematics(*new_config)
                    error = np.linalg.norm(ee_pos - target_ee)
                    if error < best_error or best_config is None:
                        best_error = error
                        best_config = test_config
    
    return best_config


def solve_left_arm_ik(arm, target_ee, base, dual_arm=None, right_config=None, max_attempts=30):
    """
    Solve IK for 2-link arm with multiple initial guesses.
    If dual_arm and right_config are provided, also checks for obstacle collisions.
    """
    target_local = target_ee - base
    dist_to_target = np.linalg.norm(target_local)
    
    theta_to_target = np.arctan2(target_local[1], target_local[0])
    
    alphas = [0.8, 0.6, 1.0, 0.5]
    
    # 2-DOF guesses for 2-link arm - try both elbow-up and elbow-down solutions
    guesses = [
        (theta_to_target, 0.0),
        (theta_to_target, 0.3),
        (theta_to_target, -0.3),
        (theta_to_target, 0.5),
        (theta_to_target, -0.5),
        (theta_to_target, np.pi/3),
        (theta_to_target, -np.pi/3),
        (0.0, 0.0),
        (np.pi/2, 0.0),
        (-np.pi/2, 0.0),
        (np.pi, 0.0),
        (0.0, 0.5),
        (0.0, -0.5),
        (np.pi/4, 0.0),
        (-np.pi/4, 0.0),
        (np.pi/4, np.pi/4),
        (np.pi/4, -np.pi/4),
        (-np.pi/4, np.pi/4),
        (-np.pi/4, -np.pi/4),
    ]
    
    tolerances = [1e-2, 5e-2, 1e-1, 2e-1]
    
    # Collect all valid IK solutions
    valid_solutions = []
    
    for alpha in alphas:
        for tol in tolerances:
            for guess in guesses[:max_attempts]:
                try:
                    config = arm.ik_iterative(
                        target_local, theta_init=guess, max_iters=500, tol=tol, alpha=alpha
                    )
                    
                    if config is not None:
                        actual_pos = arm.forward_kinematics(*config)
                        error = np.linalg.norm(actual_pos - target_local)
                        if error < tol * 2 and arm.is_valid_configuration(*config):
                            # If dual_arm provided, check obstacle collisions
                            if dual_arm is not None and right_config is not None:
                                full_config = np.concatenate([np.array(config), right_config])
                                if dual_arm.is_valid_configuration(full_config):
                                    # This solution avoids obstacles!
                                    return np.array(config)
                                else:
                                    # Store for fallback (collides but reaches goal)
                                    valid_solutions.append((np.array(config), error))
                            else:
                                # No obstacle checking, return first valid solution
                                return np.array(config)
                except:
                    continue
    
    # If we have valid solutions but they all collide, return the best one
    # (the collision fixing code will try to adjust it)
    if valid_solutions:
        # Return the one with smallest IK error
        valid_solutions.sort(key=lambda x: x[1])
        return valid_solutions[0][0]
    
    return None


def animate_path(dual_arm, path, obstacles, img):
    """Animate the planned path over the image."""
    detailed_path = interpolate_path_detailed(path, points_per_segment=15)
    
    fig, ax = plt.subplots(figsize=(16, 12))
    
    # Display image as background
    ax.imshow(img, extent=[-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2, 
                          -WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2],
              origin='upper', alpha=0.7, zorder=0)
    
    ax.set_aspect('equal')
    ax.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
    ax.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    
    # Plot obstacles
    for obstacle in obstacles:
        if obstacle['type'] == 'circle':
            circle = Circle(obstacle['center'], obstacle['radius'], facecolor='gray', 
                          alpha=0.6, edgecolor='black', linewidth=2, zorder=5)
            ax.add_patch(circle)
    
    n_left_joints = dual_arm.left_arm.get_num_joints()
    
    left_lines = []
    left_colors = plt.cm.Blues(np.linspace(0.4, 0.9, n_left_joints))
    
    for i in range(n_left_joints):
        line, = ax.plot([], [], '-o', color=left_colors[i], linewidth=2.5, 
                       markersize=7, markeredgecolor='black', markeredgewidth=1.2,
                       label=f'Left Arm ({n_left_joints} links)' if i == 0 else '', zorder=6)
        left_lines.append(line)
    
    ax.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', 
            color='darkblue', markersize=18, label='Left Base', 
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    # Compute trajectory
    left_trajectory = []
    for config in detailed_path:
        left_config, _ = dual_arm._split_configuration(config)
        left_local = dual_arm.left_arm.forward_kinematics(*left_config)
        left_trajectory.append(dual_arm.left_base + left_local)
    
    left_traj = np.array(left_trajectory)
    
    ax.plot(left_traj[:, 0], left_traj[:, 1], 'b-', linewidth=1.5, 
            alpha=0.3, label='Left Trajectory', zorder=1)
    
    # Mark start and goal
    ax.plot(left_traj[0, 0], left_traj[0, 1], 'go', markersize=15, 
            markeredgecolor='black', markeredgewidth=2, label='Start', zorder=9)
    ax.plot(left_traj[-1, 0], left_traj[-1, 1], 'g*', markersize=20,
            markeredgecolor='black', markeredgewidth=1, label='Goal', zorder=9)
    
    title_text = ax.text(0.5, 0.98, '', transform=ax.transAxes,
                        fontsize=15, fontweight='bold', ha='center', va='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9), zorder=11)
    
    info_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                       fontsize=10, verticalalignment='bottom',
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8), zorder=11)
    
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    def animate(frame):
        config = detailed_path[frame]
        left_config, _ = dual_arm._split_configuration(config)
        
        left_positions = compute_all_joint_positions(
            dual_arm.left_arm, dual_arm.left_base, left_config
        )
        
        for i in range(len(left_positions) - 1):
            link_x = [left_positions[i][0], left_positions[i+1][0]]
            link_y = [left_positions[i][1], left_positions[i+1][1]]
            left_lines[i].set_data(link_x, link_y)
        
        title_text.set_text(f'RRT* Animation - 2-Link Left Arm\n'
                           f'Frame {frame+1}/{len(detailed_path)}')
        
        left_ee = left_positions[-1]
        
        info_text.set_text(
            f'Left EE: ({left_ee[0]:.2f}, {left_ee[1]:.2f})\n'
            f'Path: {len(path)} → {len(detailed_path)} configs'
        )
        
        return left_lines + [title_text, info_text]
    
    anim = FuncAnimation(fig, animate, frames=len(detailed_path), interval=80, 
                        blit=True, repeat=True)
    
    plt.tight_layout()
    plt.show()


def preview_detection(img, obstacles, start_pos, goal_pos, left_base, right_base, left_arm):
    """Show preview of detected features before planning."""
    fig, ax = plt.subplots(figsize=(14, 10))
    
    # Display image as background
    ax.imshow(img, extent=[-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2, 
                          -WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2],
              origin='upper', alpha=0.7, zorder=0)
    
    ax.set_aspect('equal')
    ax.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
    ax.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax.set_title('Detection Preview - Review before planning', fontsize=14, fontweight='bold')
    
    # Plot obstacles
    for i, obstacle in enumerate(obstacles):
        if obstacle['type'] == 'circle':
            circle = Circle(obstacle['center'], obstacle['radius'], facecolor='gray', 
                          alpha=0.6, edgecolor='black', linewidth=2, zorder=5)
            ax.add_patch(circle)
            # Label obstacle
            ax.text(obstacle['center'][0], obstacle['center'][1], f'O{i+1}',
                   ha='center', va='center', fontsize=10, fontweight='bold',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), zorder=6)
    
    # Plot start and goal markers
    ax.plot(start_pos[0], start_pos[1], 'go', markersize=20, 
            markeredgecolor='black', markeredgewidth=2, label='Start (Green)', zorder=9)
    ax.plot(goal_pos[0], goal_pos[1], 'r*', markersize=25,
            markeredgecolor='black', markeredgewidth=1, label='Goal (Red)', zorder=9)
    
    # Plot arm bases
    ax.plot(left_base[0], left_base[1], 's', 
            color='darkblue', markersize=18, label='Left Base', 
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    ax.plot(right_base[0], right_base[1], 's',
            color='darkred', markersize=18, label='Right Base',
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    # Draw reachable workspace for left arm
    if hasattr(left_arm, 'L1') and hasattr(left_arm, 'L2'):
        max_reach = left_arm.L1 + left_arm.L2
        reach_circle = Circle(left_base, max_reach, fill=False, 
                            linestyle='--', color='blue', alpha=0.3, 
                            linewidth=2, label=f'Left Reach ({max_reach:.2f}m)', zorder=2)
        ax.add_patch(reach_circle)
    
    # Info text
    info_text = (
        f'Detected Features:\n'
        f'• Obstacles: {len(obstacles)}\n'
        f'• Start: ({start_pos[0]:.2f}, {start_pos[1]:.2f})\n'
        f'• Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})\n'
        f'• Workspace: {WORKSPACE_WIDTH}m × {WORKSPACE_HEIGHT}m\n'
        f'\nClose this window to proceed with planning.'
    )
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
           fontsize=10, verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9), zorder=11)
    
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    plt.tight_layout()
    plt.show()


def main():
    """Main function: load image, detect features, plan, animate."""
    print("Image-Based RRT* Path Planning")
    print("Usage: python demo_image_planning.py [image_path]")
    print("Instructions: Image should have GREEN start marker, RED goal marker, and dark obstacles")
    
    # Load image
    img = load_image()
    if img is None:
        print("No image provided. Exiting.")
        return
    
    # Detect obstacles
    obstacles = detect_obstacles(img, threshold=0.3, min_size=50)
    print(f"Found {len(obstacles)} obstacles")
    
    # Detect start and goal markers
    start_pos = detect_color_marker(img, color='green', threshold=0.5)
    if start_pos is None:
        print("Could not detect green start marker. Using default position.")
        start_pos = np.array([-2.0, 0.0])
    
    goal_pos = detect_color_marker(img, color='red', threshold=0.5)
    if goal_pos is None:
        print("Could not detect red goal marker. Using default position.")
        goal_pos = np.array([2.0, 0.0])
    
    # Create dual-arm system with 2-link arms (longer reach for better obstacle navigation)
    # Longer arms with more equal proportions give better dexterity around obstacles
    left_arm = ThreeLinkArm(L1=2.0, L2=2.0, L3=2.0, name="Left3Link")  # Total reach: 4.5m
    right_arm = TwoLinkArm(L1=2.5, L2=2.0, name="Right2Link")
    
    # Position left arm base at center-left (better reach across workspace)
    left_base = np.array([-1.5, 0.0])
    right_base = np.array([1.5, 0.0])
    separation = np.linalg.norm(right_base - left_base)
    
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, 
                       separation=separation, obstacles=obstacles)
    dual_arm.left_base = left_base
    dual_arm.right_base = right_base
    
    max_reach = left_arm.L1 + left_arm.L2
    
    # Check if start/goal are reachable
    dist_to_start = np.linalg.norm(start_pos - left_base)
    dist_to_goal = np.linalg.norm(goal_pos - left_base)
    
    if dist_to_start > max_reach:
        print(f"Warning: Start position ({dist_to_start:.2f}m) exceeds arm reach ({max_reach:.2f}m)")
    if dist_to_goal > max_reach:
        print(f"Warning: Goal position ({dist_to_goal:.2f}m) exceeds arm reach ({max_reach:.2f}m)")
    
    # Show preview
    preview_detection(img, obstacles, start_pos, goal_pos, left_base, right_base, left_arm)
    
    # Solve IK for start and goal (with obstacle checking)
    right_config = np.zeros(2)  # Create right config early for IK checking
    left_start_config = solve_left_arm_ik(left_arm, start_pos, left_base, dual_arm, right_config)
    left_goal_config = solve_left_arm_ik(left_arm, goal_pos, left_base, dual_arm, right_config)
    
    if left_start_config is None:
        print("Could not solve IK for start position")
        return
    
    if left_goal_config is None:
        print("Could not solve IK for goal position")
        return
    
    # Fix right arm at zeros (2-DOF for 2-link arm)
    right_config = np.zeros(2)
    start_config = np.concatenate([left_start_config, right_config])
    goal_config = np.concatenate([left_goal_config, right_config])
    
    # Verify configurations are valid with obstacles
    if not dual_arm.is_valid_configuration(start_config):
        print("Start config collides, adjusting...")
        fixed = try_fix_collision(left_start_config, dual_arm, right_config, left_arm, left_base, start_pos)
        if fixed is not None:
            start_config = fixed
            # Check how far end-effector moved
            fixed_left, _ = dual_arm._split_configuration(fixed)
            fixed_ee = left_base + left_arm.forward_kinematics(*fixed_left)
            error = np.linalg.norm(fixed_ee - start_pos)
            print(f"Start config fixed (EE moved {error:.3f}m from target)")
        else:
            print("Warning: Could not fix start config collision. Planning may fail.")
    
    if not dual_arm.is_valid_configuration(goal_config):
        print("Goal config collides, adjusting...")
        fixed = try_fix_collision(left_goal_config, dual_arm, right_config, left_arm, left_base, goal_pos)
        if fixed is not None:
            goal_config = fixed
            # Check how far end-effector moved
            fixed_left, _ = dual_arm._split_configuration(fixed)
            fixed_ee = left_base + left_arm.forward_kinematics(*fixed_left)
            error = np.linalg.norm(fixed_ee - goal_pos)
            print(f"Goal config fixed (EE moved {error:.3f}m from target)")
        else:
            print("Error: Could not fix goal config collision.")
            print("The goal position may be too close to an obstacle.")
            print("Try adjusting the goal marker position in your image.")
            return
    
    # Final validation before planning
    if not dual_arm.is_valid_configuration(start_config):
        print("Error: Start configuration is still invalid after collision fixing.")
        return
    
    if not dual_arm.is_valid_configuration(goal_config):
        print("Error: Goal configuration is still invalid after collision fixing.")
        return
    
    # Plan path (more iterations for better path quality)
    planner = RRTStar(
        dual_arm,
        max_iterations=3000,  # More iterations for better exploration and path quality
        step_size=0.8,  # Smaller step for smoother paths (0.9 was too large)
        goal_threshold=0.7,  # Tighter threshold for better goal precision
        rewire_radius=0.6,
        verbose=True
        #use_workspace_cost=False
    )
    
    path = planner.plan(start_config, goal_config)
    
    if path is None:
        print("Planning failed - no path found")
        return
    
    print(f"Path found with {len(path)} configurations")
    
    # Animate
    animate_path(dual_arm, path, obstacles, img)


if __name__ == "__main__":
    main()

