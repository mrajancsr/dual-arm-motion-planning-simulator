"""
Determine θ_home: Visual Calibration

This script helps you determine the simulator joint angles (θ1, θ2) that
visually match the real robot's coiled HOME pose.

Process:
1. Move robot to HOME (physical coiled pose)
2. Adjust θ1, θ2 in simulator until rendering matches real robot
3. Save θ_home for use in FK/visualization offset
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
import os
import sys

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from robot.robot_config import LINK1_LENGTH, LINK2_LENGTH, ROBOT_BASE_POSITION

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "home_pose.json")

def draw_arm(ax, theta1, theta2, base_pos, L1, L2, color='blue', alpha=1.0, label=None):
    """Draw 2-link arm at given angles."""
    base_x, base_y = base_pos
    
    # Compute joint positions
    x1 = base_x + L1 * np.cos(theta1)
    y1 = base_y + L1 * np.sin(theta1)
    
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    # Draw links
    ax.plot([base_x, x1], [base_y, y1], 'o-', color=color, linewidth=3, 
            markersize=8, alpha=alpha, label=label)
    ax.plot([x1, x2], [y1, y2], 'o-', color=color, linewidth=3, 
            markersize=8, alpha=alpha)
    
    # Draw base
    ax.plot(base_x, base_y, 's', color='black', markersize=12, zorder=10)
    
    return (x1, y1), (x2, y2)

def main():
    print("=" * 60)
    print("Determine θ_home (Visual Calibration)")
    print("=" * 60)
    print("\nThis script helps you find the simulator angles that match")
    print("your robot's coiled HOME pose.")
    print("\nSteps:")
    print("1. Move your robot to HOME pose (coiled rest position)")
    print("2. Visually compare the simulator rendering to your real robot")
    print("3. Adjust θ1 and θ2 until they match")
    print("\nPress Enter when robot is at HOME...")
    input()
    
    # Load HOME pose (for reference)
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r") as f:
            home_data = json.load(f)
        print(f"\nHardware HOME: u_sh={home_data['hardware_home']['shoulder_lift']['normalized']:.3f}, "
              f"u_el={home_data['hardware_home']['elbow_flex']['normalized']:.3f}")
    else:
        print("\n⚠️  home_pose.json not found. Run record_home_pose.py first.")
        return
    
    # Initialize with current simulator_home (or default)
    if 'simulator_home' in home_data:
        theta1_home = home_data['simulator_home']['theta1']
        theta2_home = home_data['simulator_home']['theta2']
    else:
        theta1_home = 0.0
        theta2_home = 0.0
    
    print(f"\nCurrent θ_home: θ1={theta1_home:.4f}, θ2={theta2_home:.4f}")
    print("\nInteractive adjustment:")
    print("  Arrow keys: ↑/↓ adjust θ1, ←/→ adjust θ2")
    print("  'q' to quit and save")
    print("  'r' to reset to (0, 0)")
    print("  Click on the plot window to focus it for keyboard input")
    
    # Interactive visualization
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Calculate plot limits based on base position and arm reach
    base_x, base_y = ROBOT_BASE_POSITION
    max_reach = LINK1_LENGTH + LINK2_LENGTH
    margin = 0.05  # 5cm margin
    
    ax.set_xlim(base_x - max_reach - margin, base_x + max_reach + margin)
    ax.set_ylim(base_y - max_reach - margin, base_y + max_reach + margin)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Simulator Arm Rendering (adjust to match real robot)\n↑/↓: θ1, ←/→: θ2, q: save, r: reset')
    
    # Draw workspace circle for reference
    circle = plt.Circle(ROBOT_BASE_POSITION, LINK1_LENGTH + LINK2_LENGTH, 
                       fill=False, linestyle='--', color='gray', alpha=0.3)
    ax.add_patch(circle)
    
    # Store lines for updating
    line1 = None
    line2 = None
    base_marker = None
    elbow_marker = None
    ee_marker = None
    
    def update_arm_display():
        """Update the arm visualization."""
        nonlocal line1, line2, base_marker, elbow_marker, ee_marker
        
        # Clear previous arm
        if line1 is not None:
            line1.remove()
        if line2 is not None:
            line2.remove()
        if base_marker is not None:
            base_marker.remove()
        if elbow_marker is not None:
            elbow_marker.remove()
        if ee_marker is not None:
            ee_marker.remove()
        
        # Compute joint positions
        base_x, base_y = ROBOT_BASE_POSITION
        x1 = base_x + LINK1_LENGTH * np.cos(theta1_home)
        y1 = base_y + LINK1_LENGTH * np.sin(theta1_home)
        x2 = x1 + LINK2_LENGTH * np.cos(theta1_home + theta2_home)
        y2 = y1 + LINK2_LENGTH * np.sin(theta1_home + theta2_home)
        
        # Draw new arm
        line1, = ax.plot([base_x, x1], [base_y, y1], 'o-', color='blue', 
                        linewidth=3, markersize=8, label='Current')
        line2, = ax.plot([x1, x2], [y1, y2], 'o-', color='blue', 
                        linewidth=3, markersize=8)
        base_marker = ax.scatter([base_x], [base_y], c='black', s=100, 
                                 marker='s', zorder=10)
        elbow_marker = ax.scatter([x1], [y1], c='blue', s=80, marker='o', 
                                 zorder=9, edgecolors='black', linewidths=1)
        ee_marker = ax.scatter([x2], [y2], c='blue', s=120, marker='D', 
                              zorder=9, edgecolors='black', linewidths=1)
        
        ax.set_title(f'θ1={theta1_home:.4f}, θ2={theta2_home:.4f} | ↑/↓: θ1, ←/→: θ2, q: save, r: reset')
        fig.canvas.draw()
        print(f"  θ1={theta1_home:.4f}, θ2={theta2_home:.4f}")
    
    # Initial render
    update_arm_display()
    plt.tight_layout()
    
    # Keyboard event handler
    step_size = 0.05  # 0.05 radians per keypress
    
    def on_key(event):
        nonlocal theta1_home, theta2_home
        if event.key == 'up':
            theta1_home += step_size
            update_arm_display()
        elif event.key == 'down':
            theta1_home -= step_size
            update_arm_display()
        elif event.key == 'right':
            theta2_home += step_size
            update_arm_display()
        elif event.key == 'left':
            theta2_home -= step_size
            update_arm_display()
        elif event.key == 'r':
            theta1_home = 0.0
            theta2_home = 0.0
            update_arm_display()
            print("  Reset to (0, 0)")
        elif event.key == 'q':
            # Save and close
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, "r") as f:
                    home_data = json.load(f)
            else:
                home_data = {}
            
            home_data['theta_home'] = {
                "theta1": theta1_home,
                "theta2": theta2_home
            }
            
            with open(CALIBRATION_FILE, "w") as f:
                json.dump(home_data, f, indent=2)
            
            print(f"\n✓ θ_home saved: θ1={theta1_home:.4f}, θ2={theta2_home:.4f}")
            print(f"  This offset will be applied to FK/visualization")
            print(f"  Planner will still use θ_sim (zero-centered)")
            print(f"  When θ_sim = (0, 0), FK will use θ_physical = θ_home")
            plt.close()
    
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show(block=True)
    
    # Update calibration file
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r") as f:
            home_data = json.load(f)
    else:
        home_data = {}
    
    home_data['theta_home'] = {
        "theta1": theta1_home,
        "theta2": theta2_home
    }
    
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(home_data, f, indent=2)
    
    print(f"\n✓ θ_home saved: θ1={theta1_home:.4f}, θ2={theta2_home:.4f}")
    print(f"  This offset will be applied to FK/visualization")
    print(f"  Planner will still use θ_sim (zero-centered)")
    print(f"  When θ_sim = (0, 0), FK will use θ_physical = θ_home")

if __name__ == "__main__":
    main()

