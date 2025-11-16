"""
Backward Compatibility Shim for two_link_arm

This module provides backward compatibility for code that imports from
.two_link_arm. New code should import directly from .arms.two_link_arm
and .dual_arm_system.

DEPRECATED: Use direct imports from arms.two_link_arm and dual_arm_system instead.
"""

# Re-export from new locations for backward compatibility
from .arms.two_link_arm import TwoLinkArm
from .dual_arm_system import DualArm

__all__ = ['TwoLinkArm', 'DualArm']
