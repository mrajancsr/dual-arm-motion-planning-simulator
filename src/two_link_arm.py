from typing import Optional, Tuple, List

import matplotlib.pyplot as plt
import numpy as np


class TwoLinkArm:
    def __init__(self, L1=1.0, L2=0.7, name="Arm"):
        self.L1 = L1
        self.L2 = L2
        self.name = name

    # Given (theta_1, theta_2) --> (x,y)
    def forward_kinematics(self, theta1: float, theta2: float):
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)

        return np.array([x, y])

    # Given (x,y) --> (theta_1, theta_2) via geometric solution
    def ik_geometric(self, x: float, y: float):
        L1, L2 = self.L1, self.L2
        r2 = x * x + y * y
        r = np.sqrt(r2)

        # reachability condition for 2R case
        if r > (L1 + L2) or r < abs(L2 - L1):
            return None
        c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        s2_pos = np.sqrt(max(0.0, 1 - c2 * c2))
        sols = []
        for s2 in (s2_pos, -s2_pos):
            t2 = np.arctan2(s2, c2)
            k1 = L1 + L2 * np.cos(t2)
            k2 = L2 * np.sin(t2)
            t1 = np.arctan2(y, x) - np.arctan2(k2, k1)

            sols.append((t1, t2))
        return sols  # [(01,02)_elbow_up, (01,02)_elbow_down]

    # Given (theta_1, theta_2) --> Jacobian matrix
    def jacobian(self, t1: float, t2: float) -> np.ndarray:
        s1, c1 = np.sin(t1), np.cos(t1)
        s12, c12 = np.sin(t1 + t2), np.cos(t1 + t2)
        J = np.array(
            [
                [-self.L1 * s1 - self.L2 * s12, -self.L2 * s12],
                [self.L1 * c1 + self.L2 * c12, self.L2 * c12],
            ],
            dtype=np.float64,
        )
        return J

    # Given (x,y) --> (theta_1, theta_2) via iterative solution
    def ik_iterative(
        self,
        x_goal: float,
        y_goal: float,
        theta_init=(0.0, 0.0),
        max_iters: int = 200,
        tol: float = 1e-3,
        alpha: float = 0.6,
        joint_limits: Optional[int] = None,
    ):
        t1, t2 = float(theta_init[0]), float(theta_init[1])
        p_goal = np.array([x_goal, y_goal])

        for _ in range(max_iters):
            pos = self.forward_kinematics(t1, t2)
            error = p_goal - pos
            J = self.jacobian(t1, t2)

            dtheta = alpha * (J.T @ error)

            if np.linalg.norm(dtheta) > 0.1:
                dtheta = 0.1 * dtheta / np.linalg.norm(dtheta)

            if np.linalg.norm(error) < tol:
                return (t1, t2)

            t1 += dtheta[0]
            t2 += dtheta[1]

        return None

    def get_num_joints(self) -> int:
        """Get the number of joints (2 for 2-link arm)."""
        return 2
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Get joint angle limits.
        
        Returns:
            List of (min, max) tuples for each joint.
            Default: (-π, π) for both joints.
        """
        return [(-np.pi, np.pi), (-np.pi, np.pi)]
    
    def __repr__(self):
        return f"{self.name}(L1={self.L1}, L2={self.L2})"


class DualArm:
    """Holds two planar arms that can compute their FK independently"""

    def __init__(self, L1=1.0, L2=0.7, separation=2.0, 
                 left_arm: Optional[TwoLinkArm] = None,
                 right_arm: Optional[TwoLinkArm] = None):
        """
        Initialize dual arm system.
        
        Args:
            L1: Link 1 length (used if arms not provided)
            L2: Link 2 length (used if arms not provided)
            separation: Distance between arm bases
            left_arm: Left arm instance (defaults to TwoLinkArm if None)
            right_arm: Right arm instance (defaults to TwoLinkArm if None)
        """
        if left_arm is None:
            self.left_arm = TwoLinkArm(L1=L1, L2=L2, name="LeftArm")
        else:
            self.left_arm = left_arm
        
        if right_arm is None:
            self.right_arm = TwoLinkArm(L1=L1, L2=L2, name="RightArm")
        else:
            self.right_arm = right_arm

        # fixed bases, left at (-separation/2,0) right at (separation/2, 0)
        self.left_base = np.array([-separation / 2, 0.0])
        self.right_base = np.array([separation / 2, 0.0])

    def compute_positions(
        self, left_angles, right_angles
    ) -> Tuple[np.ndarray, np.ndarray]:
        left_pos = self.left_arm.forward_kinematics(*left_angles)
        right_pos = self.right_arm.forward_kinematics(*right_angles)
        return left_pos, right_pos

    def compute_fk_points(self, arm: TwoLinkArm, base: np.ndarray, theta1, theta2):
        """Returns joint and end effector position in base frame

        Args:
            arm: Robot arm instance (must have L1, L2 attributes for 2-link arms)
            base: Base position
            theta1: First joint angle
            theta2: Second joint angle

        Returns:
            Tuple of (base, joint, end_effector) positions
        """
        # For now, assume 2-link arm structure (can be extended later)
        if hasattr(arm, 'L1') and hasattr(arm, 'L2'):
            L1, L2 = arm.L1, arm.L2
        else:
            # Fallback: assume unit lengths (shouldn't happen with proper arm types)
            L1, L2 = 1.0, 0.7
        
        joint = base + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
        end_eff = joint + np.array(
            [L2 * np.cos(theta1 + theta2), L2 * np.sin(theta1 + theta2)]
        )
        return base, joint, end_eff

    def plot_arms(self, left_angles, right_angles, left_goal=None, right_goal=None):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-0.5, 3)

        # Compute FK points
        lb, lj, le = self.compute_fk_points(self.left_arm, self.left_base, *left_angles)
        rb, rj, re = self.compute_fk_points(
            self.right_arm, self.right_base, *right_angles
        )

        # Plot arms
        ax.plot(
            [lb[0], lj[0], le[0]],
            [lb[1], lj[1], le[1]],
            "-o",
            color="tab:blue",
            label="Left Arm",
        )
        ax.plot(
            [rb[0], rj[0], re[0]],
            [rb[1], rj[1], re[1]],
            "-o",
            color="tab:red",
            label="Right Arm",
        )

        # Plot goals
        if left_goal is not None:
            ax.plot(
                left_goal[0],
                left_goal[1],
                "x",
                color="blue",
                markersize=10,
                label="Left Goal",
            )
        if right_goal is not None:
            ax.plot(
                right_goal[0],
                right_goal[1],
                "x",
                color="red",
                markersize=10,
                label="Right Goal",
            )

        ax.legend()
        ax.set_title("Dual-Arm 2R Robot Visualization")
        plt.show()


if __name__ == "__main__":
    # Import the new modules
    from workspace_generator import WorkspaceGenerator, DualArmWorkspaceGenerator
    from cspace_generator import CSpaceGenerator, DualArmCSpaceGenerator
    
    print("=== Dual-Arm Motion Planning Simulator Demo ===\n")
    
    # Create dual arm system
    dual = DualArm(L1=1.0, L2=0.7, separation=2.0)
    print("Created dual-arm system with L1=1.0, L2=0.7, separation=2.0")

    # Define goal positions for each arm
    left_goal = np.array([-0.5, 1.0])
    right_goal = np.array([1.5, 1.0])
    print(f"Left goal: {left_goal}")
    print(f"Right goal: {right_goal}")

    # --- Transform goals into each arm's local coordinate frame ---
    left_goal_local = left_goal - dual.left_base
    right_goal_local = right_goal - dual.right_base

    # --- Run IK in local coordinates ---
    print("\nComputing inverse kinematics...")
    left_angles = dual.left_arm.ik_iterative(
        left_goal_local[0],
        left_goal_local[1],
        theta_init=(np.deg2rad(45), np.deg2rad(30)),
        alpha=0.6,
    )
    right_angles = dual.right_arm.ik_iterative(
        right_goal_local[0],
        right_goal_local[1],
        theta_init=(np.deg2rad(30), np.deg2rad(-20)),
        alpha=0.6,
    )

    if left_angles and right_angles:
        print(f"Left arm angles: {np.rad2deg(left_angles)} degrees")
        print(f"Right arm angles: {np.rad2deg(right_angles)} degrees")
        
        # Plot basic arm configuration
        print("\nPlotting arm configuration...")
        dual.plot_arms(left_angles, right_angles, left_goal, right_goal)
        
        # Demonstrate workspace visualization
        print("\nGenerating workspace visualizations...")
        print("1. Single arm workspace...")
        ws_gen = WorkspaceGenerator(dual.left_arm, resolution=50)
        ws_gen.plot_workspace(show_boundary=True, show_points=True)
        
        print("2. Dual arm workspace...")
        dual_ws_gen = DualArmWorkspaceGenerator(dual, resolution=50)
        dual_ws_gen.plot_dual_workspace(show_boundary=True, show_points=True)
        
        # Demonstrate C-space visualization
        print("\nGenerating configuration space visualizations...")
        print("3. Single arm C-space...")
        cspace_gen = CSpaceGenerator(dual.left_arm, resolution=50)
        cspace_gen.plot_cspace(show_invalid=True)
        
        print("4. C-space heatmap...")
        cspace_gen.plot_cspace_heatmap()
        
        print("5. Dual arm C-space (2D projections)...")
        dual_cspace_gen = DualArmCSpaceGenerator(dual, resolution=20)
        dual_cspace_gen.plot_dual_cspace_2d()
        
        print("\nDemo completed! Check the generated plots.")
    else:
        print("Failed to find valid inverse kinematics solutions.")
