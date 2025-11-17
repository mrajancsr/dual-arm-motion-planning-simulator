"""
RRT* Motion Planning Algorithm

Implements RRT* (Rapidly-exploring Random Tree Star) for dual-arm motion planning
in configuration space. Works with any arm type (2-link, 6-link, etc.).

Key implementation details (per Nico's refactor):
- Uses DualArm.is_valid_configuration(config) which takes a single numpy array
- Format: [θ1_left, θ2_left, θ1_right, θ2_right] for 2-link arms
- Format: [θ1_left...θ6_left, θ1_right...θ6_right] for 6-link arms
- Validates both individual arm validity (joint limits, workspace) and inter-arm collisions
- Samples random configs in C-space and validates them during tree expansion
"""

from typing import List, Optional, Tuple, Dict
import numpy as np

try:
    from .dual_arm_system import DualArm
except ImportError:
    from dual_arm_system import DualArm


class RRTStar:
    """
    RRT* planner for dual-arm systems in configuration space.
    
    Takes start and goal configurations in C-space and plans a collision-free path.
    """
    
    def __init__(self, dual_arm: DualArm, max_iterations: int = 5000,
                 step_size: float = 0.1, goal_threshold: float = 0.1,
                 rewire_radius: float = 0.5):
        """
        Initialize RRT* planner.
        
        Args:
            dual_arm: DualArm system instance
            max_iterations: Maximum number of iterations
            step_size: Maximum step size in configuration space (radians)
            goal_threshold: Distance threshold to consider goal reached
            rewire_radius: Radius for RRT* rewiring optimization
        """
        self.dual_arm = dual_arm
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.rewire_radius = rewire_radius
        
        # Get joint limits for sampling
        self.left_limits = dual_arm.left_arm.get_joint_limits()
        self.right_limits = dual_arm.right_arm.get_joint_limits()
        self.config_dim = dual_arm.left_arm.get_num_joints() + dual_arm.right_arm.get_num_joints()
    
    def plan(self, start_config: np.ndarray, goal_config: np.ndarray) -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal configuration.
        
        Args:
            start_config: Start configuration as numpy array
            goal_config: Goal configuration as numpy array
            
        Returns:
            List of configurations forming the path, or None if planning fails
        """
        # Validate start and goal
        if not self.dual_arm.is_valid_configuration(start_config):
            raise ValueError("Start configuration is invalid")
        if not self.dual_arm.is_valid_configuration(goal_config):
            raise ValueError("Goal configuration is invalid")
        
        # Initialize tree with start node
        tree = [{
            'config': start_config.copy(),
            'parent': None,
            'cost': 0.0
        }]
        
        goal_node_idx = None
        
        for iteration in range(self.max_iterations):
            # Sample random configuration (with goal bias)
            if np.random.random() < 0.1:  # 10% chance to sample goal
                sample_config = goal_config.copy()
            else:
                sample_config = self.sample_random_config()
            
            # Find nearest node in tree
            nearest_idx = self.find_nearest_node(tree, sample_config)
            nearest_node = tree[nearest_idx]
            
            # Steer towards sample
            new_config = self.steer(nearest_node['config'], sample_config)
            
            # Check if path from nearest to new is valid
            if not self.is_path_valid(nearest_node['config'], new_config):
                continue
            
            # Calculate cost to reach new node
            cost_to_new = nearest_node['cost'] + self.distance(nearest_node['config'], new_config)
            
            # Find neighbors within rewire radius
            neighbors = self.find_neighbors(tree, new_config, self.rewire_radius)
            
            # Choose best parent from neighbors
            best_parent_idx = nearest_idx
            best_cost = cost_to_new
            
            for neighbor_idx in neighbors:
                neighbor = tree[neighbor_idx]
                cost_via_neighbor = neighbor['cost'] + self.distance(neighbor['config'], new_config)
                if cost_via_neighbor < best_cost:
                    if self.is_path_valid(neighbor['config'], new_config):
                        best_parent_idx = neighbor_idx
                        best_cost = cost_via_neighbor
            
            # Add new node to tree
            new_node_idx = len(tree)
            tree.append({
                'config': new_config.copy(),
                'parent': best_parent_idx,
                'cost': best_cost
            })
            
            # Rewire neighbors
            self.rewire(tree, new_node_idx, neighbors)
            
            # Check if goal is reached
            if self.distance(new_config, goal_config) < self.goal_threshold:
                goal_node_idx = new_node_idx
                break
        
        # Reconstruct path if goal reached
        if goal_node_idx is not None:
            path = self.reconstruct_path(tree, goal_node_idx)
            # Add goal config to path if reachable from last node
            last_node_config = path[-1]
            if self.is_path_valid(last_node_config, goal_config, num_checks=5):
                path.append(goal_config)
            return path
        
        return None
    
    def sample_random_config(self) -> np.ndarray:
        """
        Sample a random valid configuration in C-space.
        
        Returns:
            Random configuration as numpy array
        """
        max_attempts = 100
        for _ in range(max_attempts):
            # Sample from joint limits
            config_parts = []
            
            # Sample left arm joints
            for limits in self.left_limits:
                config_parts.append(np.random.uniform(limits[0], limits[1]))
            
            # Sample right arm joints
            for limits in self.right_limits:
                config_parts.append(np.random.uniform(limits[0], limits[1]))
            
            config = np.array(config_parts)
            
            # Check validity
            if self.dual_arm.is_valid_configuration(config):
                return config
        
        # Fallback: return zero configuration if sampling fails
        return np.zeros(self.config_dim)
    
    def find_nearest_node(self, tree: List[Dict], config: np.ndarray) -> int:
        """
        Find nearest node in tree to given configuration.
        
        Args:
            tree: List of tree nodes
            config: Target configuration
            
        Returns:
            Index of nearest node
        """
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, node in enumerate(tree):
            dist = self.distance(node['config'], config)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def steer(self, from_config: np.ndarray, to_config: np.ndarray) -> np.ndarray:
        """
        Steer from one configuration towards another.
        
        Args:
            from_config: Starting configuration
            to_config: Target configuration
            
        Returns:
            New configuration moved step_size towards target
        """
        direction = to_config - from_config
        dist = np.linalg.norm(direction)
        
        if dist <= self.step_size:
            return to_config.copy()
        
        # Move step_size in direction of target
        new_config = from_config + (direction / dist) * self.step_size
        return new_config
    
    def is_path_valid(self, config1: np.ndarray, config2: np.ndarray, 
                     num_checks: int = 10) -> bool:
        """
        Check if path segment between two configurations is collision-free.
        
        Args:
            config1: Start configuration
            config2: End configuration
            num_checks: Number of intermediate points to check
            
        Returns:
            True if path is valid, False otherwise
        """
        # Check endpoints
        if not self.dual_arm.is_valid_configuration(config1):
            return False
        if not self.dual_arm.is_valid_configuration(config2):
            return False
        
        # Check intermediate points
        for i in range(1, num_checks):
            alpha = i / num_checks
            intermediate = config1 + alpha * (config2 - config1)
            if not self.dual_arm.is_valid_configuration(intermediate):
                return False
        
        return True
    
    def find_neighbors(self, tree: List[Dict], config: np.ndarray, 
                      radius: float) -> List[int]:
        """
        Find all nodes within radius of given configuration.
        
        Args:
            tree: List of tree nodes
            config: Target configuration
            radius: Search radius
            
        Returns:
            List of neighbor node indices
        """
        neighbors = []
        for i, node in enumerate(tree):
            if self.distance(node['config'], config) <= radius:
                neighbors.append(i)
        return neighbors
    
    def rewire(self, tree: List[Dict], new_node_idx: int, neighbors: List[int]):
        """
        Rewire tree for RRT* optimization.
        
        Args:
            tree: List of tree nodes (modified in place)
            new_node_idx: Index of newly added node
            neighbors: List of neighbor node indices
        """
        new_node = tree[new_node_idx]
        
        for neighbor_idx in neighbors:
            if neighbor_idx == new_node_idx:
                continue
            
            neighbor = tree[neighbor_idx]
            cost_via_new = new_node['cost'] + self.distance(new_node['config'], neighbor['config'])
            
            if cost_via_new < neighbor['cost']:
                # Check if path is valid
                if self.is_path_valid(new_node['config'], neighbor['config']):
                    neighbor['parent'] = new_node_idx
                    neighbor['cost'] = cost_via_new
                    # Update costs of subtree
                    self.update_subtree_costs(tree, neighbor_idx)
    
    def update_subtree_costs(self, tree: List[Dict], node_idx: int):
        """
        Update costs of all nodes in subtree after rewiring.
        
        Args:
            tree: List of tree nodes
            node_idx: Root of subtree to update
        """
        node = tree[node_idx]
        parent_cost = tree[node['parent']]['cost'] if node['parent'] is not None else 0.0
        parent_config = tree[node['parent']]['config'] if node['parent'] is not None else node['config']
        
        node['cost'] = parent_cost + self.distance(parent_config, node['config'])
        
        # Update children
        for i, other_node in enumerate(tree):
            if other_node['parent'] == node_idx:
                self.update_subtree_costs(tree, i)
    
    def reconstruct_path(self, tree: List[Dict], goal_node_idx: int) -> List[np.ndarray]:
        """
        Reconstruct path from tree.
        
        Args:
            tree: List of tree nodes
            goal_node_idx: Index of goal node
            
        Returns:
            List of configurations from start to goal
        """
        path = []
        current_idx = goal_node_idx
        
        while current_idx is not None:
            path.append(tree[current_idx]['config'].copy())
            current_idx = tree[current_idx]['parent']
        
        # Reverse to get path from start to goal
        path.reverse()
        return path
    
    def distance(self, config1: np.ndarray, config2: np.ndarray) -> float:
        """
        Compute Euclidean distance between two configurations.
        
        Args:
            config1: First configuration
            config2: Second configuration
            
        Returns:
            Euclidean distance
        """
        return np.linalg.norm(config1 - config2)

