from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


class TwoLinkArm:
    def __init__(self, L1=1.0, L2=0.7, name="Arm"):
        self.L1 = L1
        self.L2 = L2
        self.name = name

    def forward_kinematics(self, theta1: float, theta2: float):
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)

        return np.array([x, y])

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

    def __repr__(self):
        return f"{self.name}(L1={self.L1}, L2={self.L2})"


class DualArm:
    """Holds two planar arms that can compute their FK independently"""

    def __init__(self, L1=1.0, L2=0.7, separation=2.0):
        self.left_arm: TwoLinkArm = TwoLinkArm(L1=L1, L2=L2, name="LeftArm")
        self.right_arm: TwoLinkArm = TwoLinkArm(L1=L1, L2=L2, name="RightArm")

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
        """Returns join and end effector position in base frame

        Args:
            arm (TwoLinkArm): _description_
            base (np.ndarray): _description_
            theta1 (_type_): _description_
            theta2 (_type_): _description_

        Returns:
            _type_: _description_
        """
        joint = base + np.array([arm.L1 * np.cos(theta1), arm.L1 * np.sin(theta1)])
        end_eff = joint + np.array(
            [arm.L2 * np.cos(theta1 + theta2), arm.L2 * np.sin(theta1 + theta2)]
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
    dual = DualArm(L1=1.0, L2=0.7, separation=2.0)

    # Define goal positions for each arm
    left_goal = np.array([-0.5, 1.0])
    right_goal = np.array([1.5, 1.0])

    # --- Transform goals into each arm's local coordinate frame ---
    left_goal_local = left_goal - dual.left_base
    right_goal_local = right_goal - dual.right_base

    # --- Run IK in local coordinates ---
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

    # Plot results
    if left_angles and right_angles:
        dual.plot_arms(left_angles, right_angles, left_goal, right_goal)
