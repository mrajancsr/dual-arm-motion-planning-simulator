#!/usr/bin/env python3
"""
Interactive RRT* Path Planning with Obstacles for 6-Link Arm

This test provides an interactive interface where you can:
1. Drag and drop start/goal end-effector positions (left arm only)
2. Drag and drop obstacles
3. Click "Run RRT*" to plan and animate the path
All computation happens only when "Run RRT*" is clicked.
"""

import numpy as np
from src import DualArm, SixLinkArm, RRTStar
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from matplotlib.widgets import Button


class DraggablePoint:
    """A simple draggable point with workspace constraints."""
    
    def __init__(self, point, ax, base, max_reach, color='green', marker='o', size=15, label=''):
        self.point = point
        self.ax = ax
        self.base = base
        self.max_reach = max_reach
        self.color = color
        self.marker = marker
        self.size = size
        self.label = label
        
        self.line = ax.plot(point[0], point[1], marker, color=color, 
                           markersize=size, markeredgecolor='black', 
                           markeredgewidth=2, picker=5, label=label, zorder=10)[0]
        self.press = None
        self.background = None
        
    def connect(self):
        """Connect to all the events we need."""
        self.cidpress = self.ax.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.ax.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.ax.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)
    
    def disconnect(self):
        """Disconnect all callbacks."""
        self.ax.figure.canvas.mpl_disconnect(self.cidpress)
        self.ax.figure.canvas.mpl_disconnect(self.cidrelease)
        self.ax.figure.canvas.mpl_disconnect(self.cidmotion)
    
    def constrain_to_workspace(self, pos):
        """Constrain position to reachable workspace."""
        vec = pos - self.base
        dist = np.linalg.norm(vec)
        
        if dist > self.max_reach:
            direction = vec / dist if dist > 0 else np.array([1.0, 0.0])
            pos = self.base + direction * self.max_reach
        elif dist < 0.1:  # Minimum distance from base
            direction = vec / dist if dist > 0 else np.array([1.0, 0.0])
            pos = self.base + direction * 0.1
        
        return pos
    
    def on_press(self, event):
        """Check whether mouse is over us."""
        if event.inaxes != self.ax:
            return
        
        contains, attrd = self.line.contains(event)
        if not contains:
            return
        
        self.press = (self.line.get_xdata()[0], self.line.get_ydata()[0], event.xdata, event.ydata)
        self.background = self.ax.figure.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.line)
        self.ax.figure.canvas.blit(self.ax.bbox)
    
    def on_motion(self, event):
        """Move the point if the mouse is over us."""
        if self.press is None or event.inaxes != self.ax:
            return
        
        dx = event.xdata - self.press[2]
        dy = event.ydata - self.press[3]
        
        new_x = self.press[0] + dx
        new_y = self.press[1] + dy
        new_pos = np.array([new_x, new_y])
        
        # Constrain to workspace
        new_pos = self.constrain_to_workspace(new_pos)
        
        self.point[0] = new_pos[0]
        self.point[1] = new_pos[1]
        
        self.line.set_data([new_pos[0]], [new_pos[1]])
        
        self.ax.figure.canvas.restore_region(self.background)
        self.ax.draw_artist(self.line)
        self.ax.figure.canvas.blit(self.ax.bbox)
    
    def on_release(self, event):
        """Button release handler."""
        if self.press is None:
            return
        
        self.press = None
        self.background = None
        self.ax.figure.canvas.draw_idle()
    
    def get_position(self):
        """Get current position."""
        return self.point.copy()


class DraggableObstacle:
    """A draggable circular obstacle."""
    
    def __init__(self, obstacle, ax, dual_arm, color='gray', alpha=0.6):
        self.obstacle = obstacle
        self.ax = ax
        self.dual_arm = dual_arm
        self.center = np.array(obstacle['center'])
        self.radius = obstacle['radius']
        
        self.circle = Circle(self.center, self.radius, color=color, alpha=alpha,
                           edgecolor='black', linewidth=2, zorder=5, picker=5)
        self.ax.add_patch(self.circle)
        
        self.press = None
        self.background = None
    
    def connect(self):
        """Connect to all the events we need."""
        self.cidpress = self.ax.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.ax.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.ax.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)
    
    def disconnect(self):
        """Disconnect all callbacks."""
        self.ax.figure.canvas.mpl_disconnect(self.cidpress)
        self.ax.figure.canvas.mpl_disconnect(self.cidrelease)
        self.ax.figure.canvas.mpl_disconnect(self.cidmotion)
    
    def prevent_base_collision(self, pos):
        """Prevent obstacle from overlapping with arm bases."""
        min_dist_from_base = self.radius + 0.2
        
        dist_to_left = np.linalg.norm(pos - self.dual_arm.left_base)
        
        if dist_to_left < min_dist_from_base:
            direction = (pos - self.dual_arm.left_base)
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
            pos = self.dual_arm.left_base + direction * min_dist_from_base
        
        return pos
    
    def on_press(self, event):
        """Check whether mouse is over us."""
        if event.inaxes != self.ax:
            return
        
        dx = event.xdata - self.center[0]
        dy = event.ydata - self.center[1]
        dist = np.sqrt(dx*dx + dy*dy)
        
        if dist > self.radius:
            return
        
        self.press = (self.center[0], self.center[1], event.xdata, event.ydata)
        self.background = self.ax.figure.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.circle)
        self.ax.figure.canvas.blit(self.ax.bbox)
    
    def on_motion(self, event):
        """Move the obstacle if the mouse is over us."""
        if self.press is None or event.inaxes != self.ax:
            return
        
        dx = event.xdata - self.press[2]
        dy = event.ydata - self.press[3]
        
        new_x = self.press[0] + dx
        new_y = self.press[1] + dy
        new_pos = np.array([new_x, new_y])
        
        # Prevent base collision
        new_pos = self.prevent_base_collision(new_pos)
        
        self.center[0] = new_pos[0]
        self.center[1] = new_pos[1]
        self.obstacle['center'] = [new_pos[0], new_pos[1]]
        
        self.circle.center = (new_pos[0], new_pos[1])
        self.ax.figure.canvas.restore_region(self.background)
        self.ax.draw_artist(self.circle)
        self.ax.figure.canvas.blit(self.ax.bbox)
    
    def on_release(self, event):
        """Button release handler."""
        if self.press is None:
            return
        
        self.press = None
        self.background = None
        self.ax.figure.canvas.draw_idle()
    
    def get_center(self):
        """Get current center position."""
        return self.center.copy()
    
    def get_obstacle(self):
        """Get obstacle dictionary."""
        return self.obstacle


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


def solve_left_arm_ik(arm, target_ee, base, max_attempts=30):
    """Solve IK for left arm with multiple initial guesses."""
    target_local = target_ee - base
    dist_to_target = np.linalg.norm(target_local)
    
    # Compute angle to target for smart first guess
    theta_to_target = np.arctan2(target_local[1], target_local[0])
    
    # Try different alpha values for better convergence
    alphas = [0.8, 0.6, 1.0, 0.5]
    
    guesses = [
        # Start with angle pointing toward target
        (theta_to_target, 0.0, 0.0, 0.0, 0.0, 0.0),
        (theta_to_target, 0.3, 0.3, 0.0, 0.0, 0.0),
        (theta_to_target, -0.3, -0.3, 0.0, 0.0, 0.0),
        (theta_to_target, 0.2, 0.2, 0.2, 0.0, 0.0),
        (theta_to_target, -0.2, -0.2, -0.2, 0.0, 0.0),
        # Then try other angles
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0),
        (-np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0),
        (np.pi, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.5, 0.5, 0.0, 0.0, 0.0),
        (0.0, -0.5, -0.5, 0.0, 0.0, 0.0),
        (np.pi/4, 0.3, 0.3, 0.0, 0.0, 0.0),
        (-np.pi/4, -0.3, -0.3, 0.0, 0.0, 0.0),
        (np.pi/3, 0.2, 0.2, 0.0, 0.0, 0.0),
        (-np.pi/3, -0.2, -0.2, 0.0, 0.0, 0.0),
        (3*np.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0),
        (2*np.pi/3, 0.0, 0.0, 0.0, 0.0, 0.0),
        (5*np.pi/6, 0.0, 0.0, 0.0, 0.0, 0.0),
        # Try distributed angles
        (theta_to_target + 0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        (theta_to_target - 0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        (theta_to_target + 0.2, 0.0, 0.0, 0.0, 0.0, 0.0),
        (theta_to_target - 0.2, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]
    
    # Try different tolerances (relaxed for 6-link arm)
    tolerances = [1e-2, 5e-2, 1e-1, 2e-1]  # 1cm, 5cm, 10cm, 20cm
    
    for alpha in alphas:
        for tol in tolerances:
            for guess in guesses[:max_attempts]:
                try:
                    config = arm.ik_iterative(
                        target_local, theta_init=guess, max_iters=500, tol=tol, alpha=alpha
                    )
                    
                    if config is not None:
                        # Verify the solution is actually close
                        actual_pos = arm.forward_kinematics(*config)
                        error = np.linalg.norm(actual_pos - target_local)
                        if error < tol * 2 and arm.is_valid_configuration(*config):
                            return np.array(config)
                except:
                    continue
    
    return None


def animate_rrt_path(dual_arm, path, obstacles, save_gif=False):
    """Animate the planned path."""
    print(f"\n=== Animating RRT* Planned Path ===")
    print(f"Path length: {len(path)} configurations")
    
    detailed_path = interpolate_path_detailed(path, points_per_segment=15)
    print(f"Detailed path length: {len(detailed_path)} configurations")
    
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.set_aspect('equal')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 4)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    
    # Plot obstacles
    for obstacle in obstacles:
        if obstacle['type'] == 'circle':
            circle = Circle(obstacle['center'], obstacle['radius'], color='gray', 
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
        
        title_text.set_text(f'RRT* Animation - 6-Link Left Arm\n'
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
    print("✓ Animation complete!")


def interactive_rrt_planner():
    """Interactive RRT* planner - left arm only, no computation during dragging."""
    print("=" * 70)
    print("Interactive RRT* Path Planning - 6-Link Left Arm")
    print("=" * 70)
    print("\nInstructions:")
    print("1. Drag the green circle to set start end-effector position")
    print("2. Drag the red star to set goal end-effector position")
    print("3. Drag the gray obstacle to reposition it")
    print("4. Click 'Run RRT*' to plan and animate the path")
    print("   (All computation happens only when you click Run)")
    print("=" * 70)
    
    # Create dual-arm system (but we only care about left arm)
    left_arm = SixLinkArm(L1=0.7, L2=0.6, L3=0.5, L4=0.4, L5=0.3, L6=0.25, name="Left6Link")
    right_arm = SixLinkArm(L1=0.7, L2=0.6, L3=0.5, L4=0.4, L5=0.3, L6=0.25, name="Right6Link")
    
    # Initial obstacle
    obstacles = [{'type': 'circle', 'center': [-0.3, -0.8], 'radius': 0.3}]
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0, obstacles=obstacles)
    
    # Initial start/goal positions (left arm only)
    start_ee_left = np.array([-2.75, 0.0])
    goal_ee_left = np.array([0.25, 0.0])
    
    # Set up interactive figure
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.set_aspect('equal')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 4)
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax.set_title('Interactive RRT* Planner - Drag points and obstacle, then click Run', 
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Draw workspace constraint (reachable region for left arm)
    if hasattr(left_arm, 'link_lengths'):
        left_max_reach = np.sum(left_arm.link_lengths)
    elif hasattr(left_arm, 'L1') and hasattr(left_arm, 'L2'):
        left_max_reach = left_arm.L1 + left_arm.L2
    else:
        left_max_reach = 2.0  # Default fallback
    left_reach_circle = Circle(dual_arm.left_base, left_max_reach, 
                              fill=False, linestyle='--', color='blue', 
                              alpha=0.3, linewidth=2, label='Left Reach', zorder=2)
    ax.add_patch(left_reach_circle)
    
    # Plot base
    ax.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', 
            color='darkblue', markersize=18, label='Left Base', 
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    # Create draggable points (simple, no validation)
    start_point = DraggablePoint(
        start_ee_left.copy(), ax, dual_arm.left_base, left_max_reach,
        color='green', marker='o', size=15, label='Start'
    )
    goal_point = DraggablePoint(
        goal_ee_left.copy(), ax, dual_arm.left_base, left_max_reach,
        color='red', marker='*', size=20, label='Goal'
    )
    
    # Create draggable obstacle
    draggable_obstacle = DraggableObstacle(obstacles[0], ax, dual_arm)
    
    # Connect all draggables
    start_point.connect()
    goal_point.connect()
    draggable_obstacle.connect()
    
    # Status text
    status_text = ax.text(0.02, 0.95, 'Ready - Drag points and click Run RRT*', 
                         transform=ax.transAxes, fontsize=11,
                         bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8),
                         zorder=11, verticalalignment='top')
    
    # Add Run button
    ax_button = plt.axes([0.85, 0.02, 0.12, 0.05])
    button = Button(ax_button, 'Run RRT*', color='lightgreen', hovercolor='green')
    
    # Store references for button callback
    planner_state = {
        'dual_arm': dual_arm,
        'left_arm': left_arm,
        'start_point': start_point,
        'goal_point': goal_point,
        'obstacle': draggable_obstacle,
        'fig': fig,
        'ax': ax,
        'status_text': status_text,
        'button': button,
        'planning': False
    }
    
    def run_planner(event):
        """Callback for Run button - all computation happens here."""
        if planner_state['planning']:
            return
        
        # Get current positions
        start_ee_left = planner_state['start_point'].get_position()
        goal_ee_left = planner_state['goal_point'].get_position()
        
        # Input validation
        if np.allclose(start_ee_left, goal_ee_left):
            planner_state['status_text'].set_text('Error: Start and goal are the same!')
            planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
            fig.canvas.draw_idle()
            return
        
        planner_state['planning'] = True
        button.label.set_text('Planning...')
        button.color = 'yellow'
        fig.canvas.draw_idle()
        
        try:
            print("\n" + "=" * 70)
            print("🚀 Running RRT* Planner...")
            print("=" * 70)
            
            # Update obstacle
            obstacle = planner_state['obstacle'].get_obstacle()
            planner_state['dual_arm'].obstacles = [obstacle]
            
            print(f"Start EE: {start_ee_left}")
            print(f"Goal EE: {goal_ee_left}")
            print(f"Obstacle: {obstacle}")
            
            # Solve IK for left arm only
            print("\n🔍 Solving inverse kinematics for left arm...")
            planner_state['status_text'].set_text('Solving IK...')
            fig.canvas.draw_idle()
            
            left_start_config = solve_left_arm_ik(
                planner_state['left_arm'], start_ee_left, planner_state['dual_arm'].left_base
            )
            left_goal_config = solve_left_arm_ik(
                planner_state['left_arm'], goal_ee_left, planner_state['dual_arm'].left_base
            )
            
            if left_start_config is None:
                planner_state['status_text'].set_text('Error: Could not solve IK for start!')
                planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
                fig.canvas.draw_idle()
                planner_state['planning'] = False
                button.label.set_text('Run RRT*')
                button.color = 'lightgreen'
                print("❌ Could not solve IK for start position")
                return
            
            if left_goal_config is None:
                planner_state['status_text'].set_text('Error: Could not solve IK for goal!')
                planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
                fig.canvas.draw_idle()
                planner_state['planning'] = False
                button.label.set_text('Run RRT*')
                button.color = 'lightgreen'
                print("❌ Could not solve IK for goal position")
                return
            
            print("✓ Found valid configurations")
            
            # Fix right arm at zeros (we don't care about it)
            right_config = np.zeros(6)
            start_config = np.concatenate([left_start_config, right_config])
            goal_config = np.concatenate([left_goal_config, right_config])
            
            # Verify configurations are valid with obstacles
            if not planner_state['dual_arm'].is_valid_configuration(start_config):
                print("⚠️  Start config collides with obstacle, trying to adjust...")
                # Try small perturbations
                for _ in range(20):
                    perturbation = np.random.uniform(-0.1, 0.1, size=left_start_config.shape)
                    test_left = left_start_config + perturbation
                    test_config = np.concatenate([test_left, right_config])
                    if planner_state['dual_arm'].is_valid_configuration(test_config):
                        start_config = test_config
                        break
            
            if not planner_state['dual_arm'].is_valid_configuration(goal_config):
                print("⚠️  Goal config collides with obstacle, trying to adjust...")
                for _ in range(20):
                    perturbation = np.random.uniform(-0.1, 0.1, size=left_goal_config.shape)
                    test_left = left_goal_config + perturbation
                    test_config = np.concatenate([test_left, right_config])
                    if planner_state['dual_arm'].is_valid_configuration(test_config):
                        goal_config = test_config
                        break
            
            # Compute distance between start and goal in C-space
            config_distance = np.linalg.norm(start_config - goal_config)
            print(f"  C-space distance: {config_distance:.3f} radians")
            print(f"  Start config: {start_config[:6]}")
            print(f"  Goal config: {goal_config[:6]}")
            
            # Check if direct path is blocked
            from src.rrt_star import RRTStar as RRTStarClass
            temp_planner = RRTStarClass(planner_state['dual_arm'], step_size=0.2)
            direct_path_valid = temp_planner.is_path_valid(start_config, goal_config, num_checks=10)
            print(f"  Direct path valid: {direct_path_valid}")
            if not direct_path_valid:
                print("  ⚠️  Direct path is blocked - RRT* must navigate around obstacle")
            
            # Create RRT* planner with adjusted parameters
            # For 6-link arms, we need larger step_size and goal_threshold
            # Using workspace cost to optimize for shorter end-effector paths
            # When use_workspace_cost=True, rewire_radius is interpreted in workspace units (meters)
            planner = RRTStar(
                planner_state['dual_arm'],
                max_iterations=1500,  # More iterations for complex paths
                step_size=0.3,  # Larger step size for faster exploration
                goal_threshold=1.2,  # Much larger goal threshold for 6-link arms (C-space)
                rewire_radius=0.8,  # Workspace rewire radius in meters (when use_workspace_cost=True)
                verbose=True,
                use_workspace_cost=True  # Use workspace distance for cost optimization
            )
            
            print("\n🔍 Planning path using RRT*...")
            planner_state['status_text'].set_text('Planning path with RRT*...')
            fig.canvas.draw_idle()
            
            # Set up tree visualization
            tree_lines = []  # Store line objects for tree edges
            tree_nodes = []  # Store scatter points for tree nodes
            
            def visualize_tree_progress(tree, iteration):
                """Callback to visualize tree growth during planning."""
                # Clear previous tree visualization
                for line in tree_lines:
                    line.remove()
                tree_lines.clear()
                for node in tree_nodes:
                    node.remove()
                tree_nodes.clear()
                
                # Convert tree nodes to workspace positions (end-effector positions)
                node_positions = []
                for node in tree:
                    left_config, _ = planner_state['dual_arm']._split_configuration(node['config'])
                    left_ee_local = planner_state['left_arm'].forward_kinematics(*left_config)
                    left_ee_global = planner_state['dual_arm'].left_base + left_ee_local
                    node_positions.append(left_ee_global)
                
                # Draw tree edges (connect parent to child)
                for i, node in enumerate(tree):
                    if node['parent'] is not None:
                        parent_pos = node_positions[node['parent']]
                        child_pos = node_positions[i]
                        line, = planner_state['ax'].plot(
                            [parent_pos[0], child_pos[0]],
                            [parent_pos[1], child_pos[1]],
                            'b-', alpha=0.2, linewidth=0.5, zorder=2
                        )
                        tree_lines.append(line)
                
                # Draw tree nodes
                if len(node_positions) > 0:
                    positions_array = np.array(node_positions)
                    scatter = planner_state['ax'].scatter(
                        positions_array[:, 0], positions_array[:, 1],
                        c='blue', s=3, alpha=0.4, zorder=3
                    )
                    tree_nodes.append(scatter)
                
                # Update status
                planner_state['status_text'].set_text(
                    f'Planning... {iteration}/{planner.max_iterations} iterations\n'
                    f'Tree size: {len(tree)} nodes'
                )
                
                # Update display
                planner_state['fig'].canvas.draw_idle()
                planner_state['fig'].canvas.flush_events()
            
            path = planner.plan(start_config, goal_config, progress_callback=visualize_tree_progress)
            
            if path is None:
                planner_state['status_text'].set_text('Planning failed - no path found')
                planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='orange', alpha=0.8))
                fig.canvas.draw_idle()
                planner_state['planning'] = False
                button.label.set_text('Run RRT*')
                button.color = 'lightgreen'
                print("❌ Planning failed - no path found")
                return
            
            print(f"✓ Path found with {len(path)} configurations")
            
            # Compute actual workspace path length
            workspace_path_length = 0.0
            for i in range(len(path) - 1):
                left_config1, _ = planner_state['dual_arm']._split_configuration(path[i])
                left_config2, _ = planner_state['dual_arm']._split_configuration(path[i+1])
                left_ee1_local = planner_state['left_arm'].forward_kinematics(*left_config1)
                left_ee2_local = planner_state['left_arm'].forward_kinematics(*left_config2)
                left_ee1 = planner_state['dual_arm'].left_base + left_ee1_local
                left_ee2 = planner_state['dual_arm'].left_base + left_ee2_local
                workspace_path_length += np.linalg.norm(left_ee2 - left_ee1)
            
            # Compute straight-line distance
            left_start_config, _ = planner_state['dual_arm']._split_configuration(start_config)
            left_goal_config, _ = planner_state['dual_arm']._split_configuration(goal_config)
            left_start_ee_local = planner_state['left_arm'].forward_kinematics(*left_start_config)
            left_goal_ee_local = planner_state['left_arm'].forward_kinematics(*left_goal_config)
            start_ee_global = planner_state['dual_arm'].left_base + left_start_ee_local
            goal_ee_global = planner_state['dual_arm'].left_base + left_goal_ee_local
            straight_line_dist = np.linalg.norm(goal_ee_global - start_ee_global)
            
            print(f"\n📊 Path Analysis:")
            print(f"  Workspace path length: {workspace_path_length:.3f} meters")
            print(f"  Straight-line distance: {straight_line_dist:.3f} meters")
            print(f"  Path efficiency: {straight_line_dist/workspace_path_length*100:.1f}% (lower is better)")
            
            # Update status
            planner_state['status_text'].set_text(
                f'Path found! Length: {workspace_path_length:.2f}m (straight: {straight_line_dist:.2f}m)'
            )
            planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            fig.canvas.draw_idle()
            
            # Close interactive window and animate
            plt.close(planner_state['fig'])
            
            # Animate the path
            animate_rrt_path(planner_state['dual_arm'], path, [obstacle], save_gif=False)
            
        except Exception as e:
            print(f"❌ Error during planning: {e}")
            import traceback
            traceback.print_exc()
            planner_state['status_text'].set_text(f'Error: {str(e)[:50]}')
            planner_state['status_text'].set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
            fig.canvas.draw_idle()
            planner_state['planning'] = False
            button.label.set_text('Run RRT*')
            button.color = 'lightgreen'
    
    button.on_clicked(run_planner)
    
    ax.legend(loc='upper left', fontsize=10)
    plt.tight_layout()
    plt.show()
    
    print("\n✓ Interactive interface closed")


def main():
    """Run the interactive RRT* planner."""
    try:
        interactive_rrt_planner()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
