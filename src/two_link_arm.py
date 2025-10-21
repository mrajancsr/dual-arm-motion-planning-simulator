import numpy as np
from typing import Tuple


class TwoLinkArm:
    def __init__(self, L1=1.0, L2=0.7, name="Arm"):
        self.L1 = L1
        self.L2 = L2
        self.name = name
    
    def forward_kinematics(self, theta1: float, theta2: float):
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)

        return np.array([x, y])
    
    def __repr__(self):
        return f"{self.name}(L1={self.L1}, L2={self.L2})"


class DualArmSystem:
    """Holds two planar arms that can compute their FK independently"""
    def __init__(self):
        self.left_arm: TwoLinkArm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
        self.right_arm: TwoLinkArm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")

    def compute_positions(self, left_angles, right_angles) -> Tuple[np.ndarray, np.ndarray]:
        left_pos = self.left_arm.forward_kinematics(*left_angles)
        right_pos = self.right_arm.forward_kinematics(*right_angles)
        return left_pos, right_pos