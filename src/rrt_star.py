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
from scipy.spatial import KDTree

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
                 rewire_radius: float = 0.5, verbose: bool = False,
                 use_kdtree: bool = True, workspace_weight: float = 0.3,
                 use_adaptive_step: bool = True):
        """
        Initialize RRT* planner.
        
        Args:
            dual_arm: DualArm system instance
            max_iterations: Maximum number of iterations
            step_size: Maximum step size in configuration space (radians)
            goal_threshold: Distance threshold to consider goal reached
            rewire_radius: Radius for RRT* rewiring optimization
            verbose: Whether to print progress updates
            use_kdtree: Whether to use KD-tree for nearest neighbor search (default: True)
            workspace_weight: Weight for workspace distance in metric (0.0-1.0, default: 0.3)
            use_adaptive_step: Whether to use adaptive step sizing (default: True)
        """
        self.dual_arm = dual_arm
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.rewire_radius = rewire_radius
        self.verbose = verbose
        
        # Get joint limits for sampling
        self.left_limits = dual_arm.left_arm.get_joint_limits()
        self.right_limits = dual_arm.right_arm.get_joint_limits()
        self.config_dim = dual_arm.left_arm.get_num_joints() + dual_arm.right_arm.get_num_joints()
        
        # Phase 1: KD-tree optimization
        self.use_kdtree = use_kdtree
        self.tree_configs = []
        self.kdtree = None
        
        # Phase 2: Distance metric improvements
        self.workspace_weight = workspace_weight
        self.joint_weights = self._compute_joint_weights()
        
        # Phase 3: Adaptive step size
        self.use_adaptive_step = use_adaptive_step
    
    def plan(self, start_config: np.ndarray, goal_config: np.ndarray, 
             progress_callback=None) -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal configuration.
        
        Args:
            start_config: Start configuration as numpy array
            goal_config: Goal configuration as numpy array
            progress_callback: Optional callback function(tree, iteration) called every 10 iterations
            
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
        best_goal_cost = float('inf')
        
        for iteration in range(self.max_iterations):
            # Print progress every 1000 iterations
            if self.verbose and (iteration + 1) % 1000 == 0:
                goal_status = f", goal found" if goal_node_idx is not None else ""
                print(f"  RRT* Progress: {iteration + 1}/{self.max_iterations} iterations "
                      f"({100 * (iteration + 1) / self.max_iterations:.1f}%) - "
                      f"Tree size: {len(tree)} nodes{goal_status}")
            
            # Call progress callback every 10 iterations
            if progress_callback is not None and (iteration + 1) % 10 == 0:
                progress_callback(tree, iteration + 1)
            
            # Sample random configuration (with goal bias)
            if np.random.random() < 0.2:  # 20% chance to sample goal (increased for faster convergence)
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
                # Update best goal if this path is better
                if best_cost < best_goal_cost:
                    goal_node_idx = new_node_idx
                    best_goal_cost = best_cost
                    if self.verbose:
                        print(f"  ✓ Goal reached at iteration {iteration + 1}/{self.max_iterations} "
                              f"(cost: {best_cost:.3f})")
                    # Early termination - stop once we have a valid path
                    print(f"[RRT*] Goal found! Terminating early at iteration {iteration + 1}", flush=True)
                    break
        
        # Store final tree for API access
        self._last_tree = tree
        
        # Reconstruct path if goal reached
        if goal_node_idx is not None:
            if self.verbose:
                print(f"  ✓ Final best path found with cost: {best_goal_cost:.3f}")
            path = self.reconstruct_path(tree, goal_node_idx)
            # Add goal config to path if reachable from last node
            last_node_config = path[-1]
            if self.is_path_valid(last_node_config, goal_config, num_checks=5):
                path.append(goal_config)
            return path
        
        if self.verbose:
            print(f"  ❌ No path found after {self.max_iterations} iterations")
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
        Uses KD-tree for O(log n) search if enabled, otherwise O(n) linear search.
        
        Args:
            tree: List of tree nodes
            config: Target configuration
            
        Returns:
            Index of nearest node
        """
        if not self.use_kdtree:
            # Fallback to linear search
            return self._find_nearest_linear(tree, config)
        
        # Rebuild KDTree when tree grows
        if self.kdtree is None or len(tree) != len(self.tree_configs):
            self.tree_configs = [node['config'] for node in tree]
            self.kdtree = KDTree(self.tree_configs)
        
        dist, idx = self.kdtree.query(config, k=1)
        return int(idx)
    
    def _find_nearest_linear(self, tree: List[Dict], config: np.ndarray) -> int:
        """
        Linear search for nearest node (fallback method for comparison/debugging).
        
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
    
    def _estimate_clearance(self, config: np.ndarray) -> float:
        """
        Estimate minimum distance to nearest obstacle.
        Returns large value if far from obstacles.
        
        Args:
            config: Configuration to check
            
        Returns:
            Estimated clearance distance (minimum distance to nearest obstacle)
        """
        if not self.dual_arm.obstacles:
            return float('inf')
        
        # Get arm link positions
        segments = self.dual_arm._get_all_link_segments(config)
        
        min_dist = float('inf')
        for segment in segments:
            for obstacle in self.dual_arm.obstacles:
                # Compute distance from segment midpoint to obstacle center
                midpoint = (segment[0] + segment[1]) / 2
                if obstacle['type'] == 'circle':
                    center = np.array(obstacle['center'])
                    dist = np.linalg.norm(midpoint - center) - obstacle['radius']
                    min_dist = min(min_dist, dist)
        
        return max(min_dist, 0.01)  # Avoid zero
    
    def steer(self, from_config: np.ndarray, to_config: np.ndarray) -> np.ndarray:
        """
        Steer from one configuration towards another with adaptive step size.
        Uses smaller steps near obstacles, larger steps in open space.
        
        Args:
            from_config: Starting configuration
            to_config: Target configuration
            
        Returns:
            New configuration moved towards target
        """
        direction = to_config - from_config
        dist = np.linalg.norm(direction)
        
        if dist <= self.step_size:
            return to_config.copy()
        
        # Adaptive step size based on obstacle proximity
        if self.use_adaptive_step:
            clearance = self._estimate_clearance(from_config)
            
            if clearance < 0.2:  # Very close to obstacle
                step = self.step_size * 0.5
            elif clearance > 0.5:  # Far from obstacles
                step = self.step_size * 1.5
            else:  # Medium distance
                step = self.step_size
        else:
            step = self.step_size
        
        # Move step distance in direction of target
        new_config = from_config + (direction / dist) * step
        return new_config
    
    def is_path_valid(self, config1: np.ndarray, config2: np.ndarray, 
                     num_checks: int = 5) -> bool:
        """
        Check if path segment between two configurations is collision-free.
        
        Args:
            config1: Start configuration
            config2: End configuration
            num_checks: Number of intermediate points to check (reduced for speed)
            
        Returns:
            True if path is valid, False otherwise
        """
        # Check endpoints
        if not self.dual_arm.is_valid_configuration(config1):
            return False
        if not self.dual_arm.is_valid_configuration(config2):
            return False
        
        # Check intermediate points (reduced from 10 to 5 for speed)
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
    
    def _angular_distance(self, angle1: float, angle2: float) -> float:
        """
        Compute shortest angular distance (handles wrapping at ±π).
        
        Args:
            angle1: First angle in radians
            angle2: Second angle in radians
            
        Returns:
            Shortest angular distance (always positive)
        """
        diff = angle2 - angle1
        # Wrap to [-π, π]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return abs(diff)
    
    def _compute_joint_weights(self) -> np.ndarray:
        """
        Compute weights for each joint (earlier joints weighted higher).
        Base joints matter more than end-effector joints for overall configuration.
        
        Returns:
            Array of joint weights
        """
        n_left = self.dual_arm.left_arm.get_num_joints()
        n_right = self.dual_arm.right_arm.get_num_joints()
        
        # Exponential decay: base joints = 1.0, end joints = 0.3
        left_weights = np.linspace(1.0, 0.3, n_left)
        right_weights = np.linspace(1.0, 0.3, n_right)
        
        return np.concatenate([left_weights, right_weights])
    
    def _workspace_distance(self, config1: np.ndarray, config2: np.ndarray) -> float:
        """
        Compute Euclidean distance between end-effector positions in workspace.
        
        Args:
            config1: First configuration
            config2: Second configuration
            
        Returns:
            Sum of end-effector distances for both arms
        """
        left1, right1 = self.dual_arm._split_configuration(config1)
        left2, right2 = self.dual_arm._split_configuration(config2)
        
        # Compute end-effector positions
        left_ee1 = self.dual_arm.left_arm.forward_kinematics(*left1)
        left_ee2 = self.dual_arm.left_arm.forward_kinematics(*left2)
        right_ee1 = self.dual_arm.right_arm.forward_kinematics(*right1)
        right_ee2 = self.dual_arm.right_arm.forward_kinematics(*right2)
        
        left_dist = np.linalg.norm(left_ee1 - left_ee2)
        right_dist = np.linalg.norm(right_ee1 - right_ee2)
        
        return left_dist + right_dist
    
    def distance(self, config1: np.ndarray, config2: np.ndarray) -> float:
        """
        Hybrid distance metric combining:
        1. Angular-wrapped joint distances (weighted by importance)
        2. Task-space (workspace) end-effector distance
        
        Args:
            config1: First configuration
            config2: Second configuration
            
        Returns:
            Weighted combination of joint-space and workspace distance
        """
        # Component 1: Joint-space distance with angular wrapping and weighting
        joint_diffs = []
        for i in range(len(config1)):
            joint_diffs.append(self._angular_distance(config1[i], config2[i]))
        joint_diffs = np.array(joint_diffs)
        
        weighted_joint_dist = np.sqrt(np.sum(self.joint_weights * joint_diffs**2))
        
        # Component 2: Workspace distance
        workspace_dist = self._workspace_distance(config1, config2)
        
        # Combine: (1-workspace_weight) * joint-space + workspace_weight * task-space
        # Default: 70% joint-space, 30% task-space
        return (1 - self.workspace_weight) * weighted_joint_dist + \
               self.workspace_weight * workspace_dist
    
    def _serialize_tree(self, tree: List[Dict]) -> Dict:
        """
        Convert tree to JSON-serializable format for API/visualization.
        
        Args:
            tree: List of tree nodes
            
        Returns:
            Dictionary with serialized tree data
        """
        return {
            'nodes': [
                {
                    'id': i,
                    'config': node['config'].tolist(),
                    'parent': node['parent'],
                    'cost': float(node['cost'])
                }
                for i, node in enumerate(tree)
            ]
        }
    
    def plan_single_arm(self, arm: str, start_config: np.ndarray, 
                       goal_config: np.ndarray,
                       progress_callback=None) -> Optional[List[np.ndarray]]:
        """
        Plan for single arm while keeping other arm stationary.
        
        This method plans a path where only one arm moves, while the other
        stays fixed at its starting position. Useful for handoff planning
        where arms move sequentially.
        
        Args:
            arm: 'left' or 'right' - which arm to plan for
            start_config: Starting configuration (full dual-arm config)
            goal_config: Goal configuration (full dual-arm config)
            progress_callback: Optional callback(tree, iteration)
            
        Returns:
            List of configurations forming path, or None if planning fails
        """
        # Determine which joints belong to which arm
        num_joints = self.dual_arm.left_arm.get_num_joints()
        
        if arm == 'left':
            active_indices = list(range(num_joints))
            fixed_indices = list(range(num_joints, num_joints * 2))
        else:  # right
            active_indices = list(range(num_joints, num_joints * 2))
            fixed_indices = list(range(num_joints))
        
        # Get fixed joint values (inactive arm stays here)
        fixed_joints = start_config[fixed_indices].copy()
        
        if self.verbose:
            print(f"[RRT* Single Arm] Planning for {arm} arm", flush=True)
            print(f"  Active joints: {active_indices}", flush=True)
            print(f"  Fixed joints: {fixed_indices} at {fixed_joints}", flush=True)
        
        print(f"[RRT* Single Arm] Initializing tree with start config...", flush=True)
        
        # Initialize tree with start configuration
        tree = [{
            'config': start_config.copy(),
            'parent': None,
            'cost': 0.0
        }]
        
        goal_node_idx = None
        best_goal_cost = float('inf')
        
        # Main RRT* loop
        print(f"[RRT* Single Arm] Starting main loop ({self.max_iterations} iterations)...", flush=True)
        for iteration in range(self.max_iterations):
            if iteration % 100 == 0:
                print(f"[RRT* Single Arm] Iteration {iteration}...", flush=True)
            # Sample random configuration (only active arm)
            if np.random.random() < 0.2:  # Goal bias
                random_config = goal_config.copy()
            else:
                # Sample only active joints
                random_config = start_config.copy()
                for idx in active_indices:
                    random_config[idx] = np.random.uniform(-np.pi, np.pi)
                # Keep fixed joints
                random_config[fixed_indices] = fixed_joints
            
            # Find nearest node
            nearest_idx = self.find_nearest_node(tree, random_config)
            nearest_config = tree[nearest_idx]['config']
            
            # Steer toward random config (respecting active/fixed joints)
            new_config = self.steer(nearest_config, random_config)
            # Ensure fixed joints stay fixed
            new_config[fixed_indices] = fixed_joints
            
            # Check if path is valid
            if not self.is_path_valid(nearest_config, new_config):
                continue
            
            # Find nodes within rewire radius
            near_indices = self.find_neighbors(tree, new_config, self.rewire_radius)
            
            # Find best parent
            best_parent_idx = nearest_idx
            best_cost = tree[nearest_idx]['cost'] + self.distance(nearest_config, new_config)
            
            for near_idx in near_indices:
                near_config = tree[near_idx]['config']
                tentative_cost = tree[near_idx]['cost'] + self.distance(near_config, new_config)
                
                if tentative_cost < best_cost:
                    if self.is_path_valid(near_config, new_config):
                        best_parent_idx = near_idx
                        best_cost = tentative_cost
            
            # Add new node
            new_node = {
                'config': new_config,
                'parent': best_parent_idx,
                'cost': best_cost
            }
            tree.append(new_node)
            new_node_idx = len(tree) - 1
            
            # Rewire tree
            for near_idx in near_indices:
                near_config = tree[near_idx]['config']
                new_cost = best_cost + self.distance(new_config, near_config)
                
                if new_cost < tree[near_idx]['cost']:
                    if self.is_path_valid(new_config, near_config):
                        tree[near_idx]['parent'] = new_node_idx
                        tree[near_idx]['cost'] = new_cost
            
            # Check if goal reached
            dist_to_goal = self.distance(new_config, goal_config)
            if dist_to_goal < self.goal_threshold:
                if best_cost < best_goal_cost:
                    goal_node_idx = new_node_idx
                    best_goal_cost = best_cost
                    if self.verbose:
                        print(f"  Goal reached at iteration {iteration+1}, cost: {best_goal_cost:.3f}")
                    # Early termination - stop once we have a valid path
                    print(f"[RRT* Single Arm] Goal found! Terminating early at iteration {iteration + 1}", flush=True)
                    
                    # Update progress one last time
                    if progress_callback is not None:
                        progress_callback(tree, iteration + 1)
                    
                    break
            
            # Progress callback
            if progress_callback is not None and (iteration + 1) % 10 == 0:
                progress_callback(tree, iteration + 1)
        
        # Store final tree
        self._last_tree = tree
        
        # Reconstruct path if goal reached
        if goal_node_idx is not None:
            if self.verbose:
                print(f"  [Single Arm] Path found with cost: {best_goal_cost:.3f}")
            path = self.reconstruct_path(tree, goal_node_idx)
            # Add goal config if reachable
            if self.is_path_valid(path[-1], goal_config):
                path.append(goal_config)
            return path
        
        if self.verbose:
            print(f"  [Single Arm] No path found after {self.max_iterations} iterations")
        return None

