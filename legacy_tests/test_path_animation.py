#!/usr/bin/env python3
"""
Animated visualization of dual-arm motion planning paths.

This script animates the arms moving through a path configuration by configuration,
showing a stop-motion style animation of the motion.
"""

import numpy as np
from src import DualArm, SixLinkArm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def generate_fake_path_6link():
    """Generate a complex path for 6-link dual arm (12D configs) with multiple waypoints"""
    # Define multiple waypoints for a complex trajectory
    waypoints = [
        np.array([0.1]*6 + [0.1]*6),      # Start: all joints at 0.1 rad
        np.array([0.3, 0.2, 0.1, 0.0, -0.1, -0.2] + [0.2, 0.1, 0.0, -0.1, -0.2, -0.3]),
        np.array([0.6, 0.4, 0.2, -0.2, -0.4, -0.6] + [0.4, 0.2, 0.0, -0.2, -0.4, -0.6]),
        np.array([0.8, 0.5, 0.2, -0.2, -0.5, -0.8] + [0.5, 0.3, 0.0, -0.3, -0.5, -0.7]),
        np.array([0.5, 0.3, 0.1, -0.1, -0.3, -0.5] + [0.3, 0.1, -0.1, -0.3, -0.5, -0.7]),
        np.array([0.2, 0.1, 0.0, -0.1, -0.2, -0.3] + [0.1, 0.0, -0.1, -0.2, -0.3, -0.4]),
        np.array([-0.2, -0.1, 0.0, 0.1, 0.2, 0.3] + [-0.1, 0.0, 0.1, 0.2, 0.3, 0.4]),  # Reverse direction
        np.array([-0.5, -0.3, -0.1, 0.1, 0.3, 0.5] + [-0.3, -0.1, 0.1, 0.3, 0.5, 0.7]),
        np.array([0.1]*6 + [-0.3]*6),    # End: left at 0.1, right at -0.3
    ]
    
    # Interpolate between waypoints with smooth transitions
    path = []
    num_points_per_segment = 40  # More points for smoother animation
    
    for i in range(len(waypoints) - 1):
        segment = [waypoints[i] + t * (waypoints[i+1] - waypoints[i]) 
                  for t in np.linspace(0, 1, num_points_per_segment)]
        path.extend(segment)
    
    return path


def generate_fake_path_2link():
    """Generate a complex path for 2-link dual arm (4D configs) with curved trajectory"""
    # Define waypoints for a figure-8 like motion
    waypoints = [
        np.array([0.5, 0.3, -0.2, 0.4]),   # Start
        np.array([0.8, 0.5, 0.0, 0.2]),    # Move up and right
        np.array([1.2, 0.3, 0.3, 0.0]),    # Continue right
        np.array([1.0, -0.2, 0.5, -0.2]),  # Move down
        np.array([0.6, -0.5, 0.8, -0.4]),  # Move left and down
        np.array([0.2, -0.3, 0.5, -0.2]),  # Move left
        np.array([0.0, 0.0, 0.2, 0.0]),    # Center
        np.array([-0.3, 0.2, -0.2, 0.3]), # Move left and up
        np.array([0.2, 0.4, 0.0, 0.5]),   # Move right and up
        np.array([1.0, -0.5, 0.8, -0.3]), # Final position
    ]
    
    # Interpolate between waypoints
    path = []
    num_points_per_segment = 35
    
    for i in range(len(waypoints) - 1):
        # Use smooth interpolation (ease in/out)
        t_vals = np.linspace(0, 1, num_points_per_segment)
        # Apply smoothstep for more natural motion
        t_smooth = t_vals * t_vals * (3 - 2 * t_vals)
        segment = [waypoints[i] + t * (waypoints[i+1] - waypoints[i]) 
                  for t in t_smooth]
        path.extend(segment)
    
    return path


def compute_all_joint_positions(arm, base, joint_angles):
    """
    Compute all intermediate joint positions for any arm type.
    
    Returns list of positions: [base, joint1, joint2, ..., end_effector]
    """
    positions = [base.copy()]
    current_pos = base.copy()
    cumulative_angle = 0.0
    
    # Check if arm has link_lengths attribute (6-link arm)
    if hasattr(arm, 'link_lengths'):
        # 6-link arm
        for i, (length, angle) in enumerate(zip(arm.link_lengths, joint_angles)):
            cumulative_angle += float(angle)
            current_pos = current_pos + np.array([
                length * np.cos(cumulative_angle),
                length * np.sin(cumulative_angle)
            ])
            positions.append(current_pos.copy())
    elif hasattr(arm, 'L1') and hasattr(arm, 'L2') and len(joint_angles) == 2:
        # 2-link arm
        theta1, theta2 = joint_angles[0], joint_angles[1]
        joint = current_pos + np.array([arm.L1 * np.cos(theta1), arm.L1 * np.sin(theta1)])
        positions.append(joint)
        end_eff = joint + np.array([
            arm.L2 * np.cos(theta1 + theta2),
            arm.L2 * np.sin(theta1 + theta2)
        ])
        positions.append(end_eff)
    else:
        # Generic fallback: just base and end-effector
        end_eff_pos = arm.forward_kinematics(*joint_angles)
        positions.append(base + end_eff_pos)
    
    return positions


def animate_path(dual_arm, path, arm_type="2-link", save_gif=False):
    """
    Animate the arms moving through a path.
    
    Args:
        dual_arm: DualArm instance
        path: List of configuration arrays
        arm_type: String describing arm type
        save_gif: Whether to save as GIF (requires pillow)
    """
    print(f"\n=== Animating {arm_type} Path ===")
    print(f"Path length: {len(path)} configurations")
    
    # Validate path
    valid_count = sum(1 for config in path if dual_arm.is_valid_configuration(config))
    print(f"Valid configurations: {valid_count}/{len(path)}")
    
    # Set up the figure
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3.0, 3.0)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    
    # Initialize empty lines for arms (one line per link segment)
    # We'll create lines dynamically based on number of joints
    n_left_joints = dual_arm.left_arm.get_num_joints()
    n_right_joints = dual_arm.right_arm.get_num_joints()
    
    # Create line objects for each link segment
    left_lines = []
    right_lines = []
    
    # Colors for different links (darker to lighter)
    # For n joints, we have n links (base->joint1, joint1->joint2, ..., jointN->end_effector)
    left_colors = plt.cm.Blues(np.linspace(0.4, 0.9, n_left_joints))
    right_colors = plt.cm.Reds(np.linspace(0.4, 0.9, n_right_joints))
    
    for i in range(n_left_joints):
        line, = ax.plot([], [], '-o', color=left_colors[i], linewidth=2.5, 
                       markersize=6, markeredgecolor='black', markeredgewidth=1,
                       label=f'Left Arm ({n_left_joints} links)' if i == 0 else '')
        left_lines.append(line)
    
    for i in range(n_right_joints):
        line, = ax.plot([], [], '-o', color=right_colors[i], linewidth=2.5,
                       markersize=6, markeredgecolor='black', markeredgewidth=1,
                       label=f'Right Arm ({n_right_joints} links)' if i == 0 else '')
        right_lines.append(line)
    
    # Plot bases (static)
    ax.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', 
            color='darkblue', markersize=15, label='Left Base', 
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    ax.plot(dual_arm.right_base[0], dual_arm.right_base[1], 's',
            color='darkred', markersize=15, label='Right Base',
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    # Add trajectory paths (faded)
    left_trajectory = []
    right_trajectory = []
    for config in path:
        left_config, right_config = dual_arm._split_configuration(config)
        left_local = dual_arm.left_arm.forward_kinematics(*left_config)
        right_local = dual_arm.right_arm.forward_kinematics(*right_config)
        left_trajectory.append(dual_arm.left_base + left_local)
        right_trajectory.append(dual_arm.right_base + right_local)
    
    left_traj = np.array(left_trajectory)
    right_traj = np.array(right_trajectory)
    
    ax.plot(left_traj[:, 0], left_traj[:, 1], 'b-', linewidth=1, 
            alpha=0.2, label='Left Trajectory', zorder=0)
    ax.plot(right_traj[:, 0], right_traj[:, 1], 'r-', linewidth=1,
            alpha=0.2, label='Right Trajectory', zorder=0)
    
    # Title and info text
    title_text = ax.text(0.5, 0.98, '', transform=ax.transAxes,
                        fontsize=14, fontweight='bold', ha='center', va='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    info_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                       fontsize=10, verticalalignment='bottom',
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    def animate(frame):
        """Update function for animation"""
        config = path[frame]
        left_config, right_config = dual_arm._split_configuration(config)
        
        # Compute all joint positions for both arms
        left_positions = compute_all_joint_positions(
            dual_arm.left_arm, dual_arm.left_base, left_config
        )
        right_positions = compute_all_joint_positions(
            dual_arm.right_arm, dual_arm.right_base, right_config
        )
        
        # Update each link segment for left arm
        for i in range(len(left_positions) - 1):
            link_x = [left_positions[i][0], left_positions[i+1][0]]
            link_y = [left_positions[i][1], left_positions[i+1][1]]
            left_lines[i].set_data(link_x, link_y)
        
        # Update each link segment for right arm
        for i in range(len(right_positions) - 1):
            link_x = [right_positions[i][0], right_positions[i+1][0]]
            link_y = [right_positions[i][1], right_positions[i+1][1]]
            right_lines[i].set_data(link_x, link_y)
        
        # Update title
        title_text.set_text(f'{arm_type.upper()} Dual-Arm Animation\n'
                           f'Frame {frame+1}/{len(path)} - Showing All {n_left_joints} Links')
        
        # Update info
        left_ee = left_positions[-1]
        right_ee = right_positions[-1]
        
        info_text.set_text(
            f'Config Space: {n_left_joints + n_right_joints}D\n'
            f'Left EE: ({left_ee[0]:.2f}, {left_ee[1]:.2f})\n'
            f'Right EE: ({right_ee[0]:.2f}, {right_ee[1]:.2f})\n'
            f'Left Angles: {np.round(left_config, 2)}\n'
            f'Right Angles: {np.round(right_config, 2)}'
        )
        
        # Return all line objects for blitting
        return left_lines + right_lines + [title_text, info_text]
    
    # Create animation
    anim = FuncAnimation(fig, animate, frames=len(path), interval=100, 
                        blit=True, repeat=True)
    
    plt.tight_layout()
    
    if save_gif:
        try:
            print("Saving animation as GIF...")
            anim.save(f'{arm_type}_animation.gif', writer='pillow', fps=10)
            print(f"✓ Saved as {arm_type}_animation.gif")
        except Exception as e:
            print(f"Could not save GIF: {e}")
            print("Install pillow: pip install pillow")
    
    plt.show()
    print("✓ Animation complete!")


def main():
    print("🎬 Dual-Arm Path Animation")
    print("=" * 60)
    
    # Test 1: 6-link arms (main focus)
    print("\n--- Animation 1: 6-Link Arms ---")
    left_6link = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Left6Link")
    right_6link = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Right6Link")
    dual_6link = DualArm(left_arm=left_6link, right_arm=right_6link, separation=2.0)
    path_6link = generate_fake_path_6link()
    animate_path(dual_6link, path_6link, "6-link", save_gif=False)
    
    # Test 2: 2-link arms (for comparison)
    print("\n--- Animation 2: 2-Link Arms ---")
    dual_2link = DualArm(L1=1.0, L2=0.7, separation=2.0)
    path_2link = generate_fake_path_2link()
    animate_path(dual_2link, path_2link, "2-link", save_gif=False)
    
    print("\n" + "=" * 60)
    print("✅ All animations completed!")
    print("\nTip: Set save_gif=True to save animations as GIF files")


if __name__ == "__main__":
    main()

