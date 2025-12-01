#!/usr/bin/env python3
"""
Animated RRT* Path Planning for 6-Link Dual-Arm System

This test combines RRT* motion planning with detailed path animation.
It plans a collision-free path in 12D configuration space and animates
the 6-link arms following that path with smooth, detailed motion.
"""

import numpy as np
from src import DualArm, SixLinkArm, RRTStar
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


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


def interpolate_path_detailed(path, points_per_segment=10):
    """
    Interpolate between path configurations to create a detailed, smooth path.
    
    Args:
        path: List of configurations from RRT*
        points_per_segment: Number of intermediate points between each pair of configs
        
    Returns:
        Detailed interpolated path
    """
    if len(path) < 2:
        return path
    
    detailed_path = [path[0]]  # Start with first config
    
    for i in range(len(path) - 1):
        config_start = path[i]
        config_end = path[i + 1]
        
        # Linear interpolation in configuration space
        for t in np.linspace(0, 1, points_per_segment + 1)[1:]:  # Skip t=0 (already added)
            interpolated = config_start + t * (config_end - config_start)
            detailed_path.append(interpolated)
    
    return detailed_path


def animate_rrt_path(dual_arm, path, save_gif=False):
    """
    Animate the 6-link arms moving through an RRT* planned path.
    
    Args:
        dual_arm: DualArm instance with 6-link arms
        path: List of configurations from RRT*
        save_gif: Whether to save as GIF
    """
    print(f"\n=== Animating RRT* Planned Path ===")
    print(f"Original path length: {len(path)} configurations")
    
    # Interpolate to make path detailed and smooth
    detailed_path = interpolate_path_detailed(path, points_per_segment=15)
    print(f"Detailed path length: {len(detailed_path)} configurations")
    
    # Validate path
    valid_count = sum(1 for config in detailed_path if dual_arm.is_valid_configuration(config))
    print(f"Valid configurations: {valid_count}/{len(detailed_path)}")
    
    # Set up the figure
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-3.5, 3.5)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    
    # Get number of joints
    n_left_joints = dual_arm.left_arm.get_num_joints()
    n_right_joints = dual_arm.right_arm.get_num_joints()
    
    # Create line objects for each link segment
    left_lines = []
    right_lines = []
    
    # Colors for different links (darker to lighter for 6-link arms)
    left_colors = plt.cm.Blues(np.linspace(0.4, 0.9, n_left_joints))
    right_colors = plt.cm.Reds(np.linspace(0.4, 0.9, n_right_joints))
    
    for i in range(n_left_joints):
        line, = ax.plot([], [], '-o', color=left_colors[i], linewidth=2.5, 
                       markersize=7, markeredgecolor='black', markeredgewidth=1.2,
                       label=f'Left Arm ({n_left_joints} links)' if i == 0 else '')
        left_lines.append(line)
    
    for i in range(n_right_joints):
        line, = ax.plot([], [], '-o', color=right_colors[i], linewidth=2.5,
                       markersize=7, markeredgecolor='black', markeredgewidth=1.2,
                       label=f'Right Arm ({n_right_joints} links)' if i == 0 else '')
        right_lines.append(line)
    
    # Plot bases (static)
    ax.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', 
            color='darkblue', markersize=18, label='Left Base', 
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    ax.plot(dual_arm.right_base[0], dual_arm.right_base[1], 's',
            color='darkred', markersize=18, label='Right Base',
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    # Add trajectory paths (faded)
    left_trajectory = []
    right_trajectory = []
    for config in detailed_path:
        left_config, right_config = dual_arm._split_configuration(config)
        left_local = dual_arm.left_arm.forward_kinematics(*left_config)
        right_local = dual_arm.right_arm.forward_kinematics(*right_config)
        left_trajectory.append(dual_arm.left_base + left_local)
        right_trajectory.append(dual_arm.right_base + right_local)
    
    left_traj = np.array(left_trajectory)
    right_traj = np.array(right_trajectory)
    
    ax.plot(left_traj[:, 0], left_traj[:, 1], 'b-', linewidth=1.5, 
            alpha=0.25, label='Left Trajectory', zorder=0)
    ax.plot(right_traj[:, 0], right_traj[:, 1], 'r-', linewidth=1.5,
            alpha=0.25, label='Right Trajectory', zorder=0)
    
    # Mark start and goal positions
    ax.plot(left_traj[0, 0], left_traj[0, 1], 'go', markersize=15, 
            markeredgecolor='black', markeredgewidth=2, label='Start', zorder=9)
    ax.plot(left_traj[-1, 0], left_traj[-1, 1], 'g*', markersize=20,
            markeredgecolor='black', markeredgewidth=1, label='Goal', zorder=9)
    ax.plot(right_traj[0, 0], right_traj[0, 1], 'go', markersize=15,
            markeredgecolor='black', markeredgewidth=2, zorder=9)
    ax.plot(right_traj[-1, 0], right_traj[-1, 1], 'g*', markersize=20,
            markeredgecolor='black', markeredgewidth=1, zorder=9)
    
    # Title and info text
    title_text = ax.text(0.5, 0.98, '', transform=ax.transAxes,
                        fontsize=15, fontweight='bold', ha='center', va='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    info_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                       fontsize=10, verticalalignment='bottom',
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    def animate(frame):
        """Update function for animation"""
        config = detailed_path[frame]
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
        title_text.set_text(f'RRT* Planned Path - 6-Link Dual-Arm Animation\n'
                           f'Frame {frame+1}/{len(detailed_path)} - Showing All {n_left_joints} Links')
        
        # Update info
        left_ee = left_positions[-1]
        right_ee = right_positions[-1]
        
        info_text.set_text(
            f'Config Space: {n_left_joints + n_right_joints}D (12D)\n'
            f'Left EE: ({left_ee[0]:.2f}, {left_ee[1]:.2f})\n'
            f'Right EE: ({right_ee[0]:.2f}, {right_ee[1]:.2f})\n'
            f'Left Angles: {np.round(left_config, 2)}\n'
            f'Right Angles: {np.round(right_config, 2)}'
        )
        
        # Return all line objects for blitting
        return left_lines + right_lines + [title_text, info_text]
    
    # Create animation with slower interval for smoother viewing
    anim = FuncAnimation(fig, animate, frames=len(detailed_path), interval=80, 
                        blit=True, repeat=True)
    
    plt.tight_layout()
    
    if save_gif:
        try:
            print("Saving animation as GIF...")
            anim.save('rrt_star_6link_animation.gif', writer='pillow', fps=12)
            print(f"✓ Saved as rrt_star_6link_animation.gif")
        except Exception as e:
            print(f"Could not save GIF: {e}")
            print("Install pillow: pip install pillow")
    
    plt.show()
    print("✓ Animation complete!")


def test_rrt_star_6link_animation():
    """Test RRT* planning and animation for 6-link arms."""
    print("=" * 70)
    print("RRT* Path Planning & Animation Test - 6-Link Dual-Arm System")
    print("=" * 70)
    
    # Create 6-link dual-arm system
    left_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Left6Link")
    right_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Right6Link")
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)
    
    print("✓ Created 6-link dual-arm system (12D configuration space)")
    
    # Define start and goal configurations
    # Format: [θ1-6_left, θ1-6_right]
    start_config = np.array([
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1,  # Left arm: all joints at 0.1 rad
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1   # Right arm: all joints at 0.1 rad
    ])
    
    goal_config = np.array([
        0.5, -0.3, 0.2, -0.1, 0.4, 0.1,   # Left arm: varied angles
        -0.3, 0.5, -0.2, 0.1, -0.4, -0.1  # Right arm: varied angles
    ])
    
    print(f"\nStart config: {start_config}")
    print(f"Goal config: {goal_config}")
    
    # Validate configurations
    if not dual_arm.is_valid_configuration(start_config):
        print("❌ Start configuration is invalid!")
        return False
    if not dual_arm.is_valid_configuration(goal_config):
        print("❌ Goal configuration is invalid!")
        return False
    
    print("✓ Both start and goal configurations are valid")
    
    # Create RRT* planner with parameters tuned for 6-link arms
    planner = RRTStar(
        dual_arm, 
        max_iterations=8000,  # More iterations for 12D space
        step_size=0.08,       # Smaller step size for smoother paths
        goal_threshold=0.2,   # Slightly larger threshold for 12D
        rewire_radius=0.4
    )
    
    print("\n🔍 Planning path using RRT*...")
    print("   (This may take a moment for 12D configuration space)")
    
    path = planner.plan(start_config, goal_config)
    
    if path is None:
        print("❌ Planning failed - no path found")
        print("   Try adjusting start/goal configurations or planner parameters")
        return False
    
    print(f"✓ Path found with {len(path)} configurations")
    
    # Validate all configurations in path
    valid_count = sum(1 for config in path if dual_arm.is_valid_configuration(config))
    print(f"✓ Valid configurations: {valid_count}/{len(path)}")
    
    # Animate the path
    print("\n🎬 Starting animation...")
    animate_rrt_path(dual_arm, path, save_gif=False)
    
    return True


def main():
    """Run the RRT* animation test."""
    print("🎬 RRT* Path Planning & Animation for 6-Link Dual-Arm System")
    print("=" * 70)
    print("\nThis test:")
    print("1. Plans a collision-free path using RRT* in 12D configuration space")
    print("2. Interpolates the path for smooth, detailed motion")
    print("3. Animates the 6-link arms following the planned path")
    print()
    
    try:
        success = test_rrt_star_6link_animation()
        
        if success:
            print("\n" + "=" * 70)
            print("✅ Test completed successfully!")
            print("=" * 70)
        else:
            print("\n" + "=" * 70)
            print("❌ Test failed")
            print("=" * 70)
            
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

