#!/usr/bin/env python3
"""
Script to generate demo configurations with pre-computed planning results.
Run this script to create/update the demo JSON files in the demos/ directory.
"""

import sys
import json
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src import (
    DualArm, TwoLinkArm, ThreeLinkArm, SixLinkArm,
    HandoffPlanner
)


def create_arm(arm_type, params):
    """Create an arm instance."""
    if arm_type == '2-link':
        return TwoLinkArm(
            L1=params.get('L1', 1.0),
            L2=params.get('L2', 0.7),
            name=params.get('name', 'Arm')
        )
    elif arm_type == '3-link':
        return ThreeLinkArm(
            L1=params.get('L1', 1.0),
            L2=params.get('L2', 0.8),
            L3=params.get('L3', 0.6),
            name=params.get('name', 'Arm')
        )
    elif arm_type == '6-link':
        return SixLinkArm(
            L1=params.get('L1', 0.5),
            L2=params.get('L2', 0.4),
            L3=params.get('L3', 0.3),
            L4=params.get('L4', 0.25),
            L5=params.get('L5', 0.2),
            L6=params.get('L6', 0.15),
            name=params.get('name', 'Arm')
        )


def run_planning(config):
    """Run planning and return results."""
    print(f"\n{'='*60}")
    print(f"Planning for: {config['metadata']['name']}")
    print(f"{'='*60}")
    
    # Extract config
    arm_type = config['config']['arm_type']
    item_start = np.array(config['config']['item_start'])
    item_goal = np.array(config['config']['item_goal'])
    left_base_x = config['config']['left_base_x']
    right_base_x = config['config']['right_base_x']
    obstacles = config['config'].get('obstacles', [])
    rrt_params = config['config'].get('rrt_params', {})
    arm_params = config['config']['arm_params']
    
    print(f"Item start: {item_start}")
    print(f"Item goal: {item_goal}")
    print(f"Left base: [{left_base_x}, 0]")
    print(f"Right base: [{right_base_x}, 0]")
    print(f"Obstacles: {len(obstacles)}")
    
    # Create arms
    left_arm = create_arm(arm_type, {'name': 'LeftArm', **arm_params})
    right_arm = create_arm(arm_type, {'name': 'RightArm', **arm_params})
    
    # Create dual-arm system
    separation = abs(right_base_x - left_base_x)
    dual_arm = DualArm(
        left_arm=left_arm,
        right_arm=right_arm,
        separation=separation,
        obstacles=obstacles
    )
    
    # Override base positions
    dual_arm.left_base = np.array([left_base_x, 0.0])
    dual_arm.right_base = np.array([right_base_x, 0.0])
    
    # Create handoff planner
    handoff_planner = HandoffPlanner(dual_arm)
    
    # Track tree and iterations
    tree_data = []
    iterations_count = [0]
    
    def progress_callback(tree, iteration):
        iterations_count[0] = iteration
        if iteration % 50 == 0:
            print(f"  Iteration {iteration}, tree size: {len(tree)}")
        # Store tree snapshot (only every 10 iterations to save memory)
        if iteration % 10 == 0:
            tree_data.clear()
            tree_data.extend(tree)
    
    # Determine strategy
    import time
    start_time = time.time()
    
    strategy = handoff_planner.determine_arms(item_start, item_goal)
    print(f"\nStrategy: {strategy}")
    
    # Plan
    if not strategy['needs_handoff']:
        # Single arm solution
        chosen_arm = strategy['chosen_arm']
        path = handoff_planner._plan_single_arm_task(
            arm=chosen_arm,
            start_pos=item_start,
            goal_pos=item_goal,
            rrt_params=rrt_params,
            progress_callback=progress_callback
        )
        
        result = {
            'strategy': strategy,
            'handoff_point': None,
            'phases': [{
                'arm': chosen_arm,
                'from': 'start',
                'to': 'goal',
                'path_length': len(path) if path else 0
            }],
            'full_path': path
        }
    else:
        # Handoff solution
        handoff_point = handoff_planner.find_handoff_point(item_start, item_goal, obstacles)
        print(f"Handoff point: {handoff_point}")
        
        # Phase 1: Grab
        print("\nPhase 1: Grab")
        grab_path = handoff_planner._plan_single_arm_task(
            arm=strategy['grab_arm'],
            start_pos=item_start,
            goal_pos=handoff_point,
            rrt_params=rrt_params,
            progress_callback=progress_callback
        )
        
        if not grab_path:
            print("ERROR: Grab phase failed!")
            return None
        
        # Phase 2: Delivery
        print("\nPhase 2: Delivery")
        iterations_count[0] = 0  # Reset for phase 2
        delivery_path = handoff_planner._plan_single_arm_task(
            arm=strategy['delivery_arm'],
            start_pos=handoff_point,
            goal_pos=item_goal,
            rrt_params=rrt_params,
            progress_callback=progress_callback
        )
        
        if not delivery_path:
            print("ERROR: Delivery phase failed!")
            return None
        
        # Stitch paths
        full_path = handoff_planner._stitch_paths(grab_path, delivery_path, strategy)
        
        result = {
            'strategy': strategy,
            'handoff_point': handoff_point,
            'phases': [
                {
                    'arm': strategy['grab_arm'],
                    'from': 'start',
                    'to': 'handoff',
                    'path_length': len(grab_path)
                },
                {
                    'arm': strategy['delivery_arm'],
                    'from': 'handoff',
                    'to': 'goal',
                    'path_length': len(delivery_path)
                }
            ],
            'full_path': full_path
        }
    
    end_time = time.time()
    planning_time = end_time - start_time
    
    if not result or not result.get('full_path'):
        print("ERROR: No path found!")
        return None
    
    print(f"\nSUCCESS!")
    print(f"Planning time: {planning_time:.2f}s")
    print(f"Path length: {len(result['full_path'])} waypoints")
    print(f"Tree size: {len(tree_data)} nodes")
    print(f"Iterations: {iterations_count[0]}")
    
    # Convert tree to serializable format (sample to keep size reasonable)
    # Store every 5th node to reduce file size
    # Create mapping from original indices to sampled indices
    sampled_indices = list(range(0, len(tree_data), 5))
    index_map = {orig_idx: sampled_idx for sampled_idx, orig_idx in enumerate(sampled_indices)}
    
    sampled_tree = []
    for sampled_idx, orig_idx in enumerate(sampled_indices):
        node = tree_data[orig_idx]
        # Remap parent index to sampled array index, or set to None if parent not in sample
        parent_orig = node['parent']
        parent_sampled = None
        if parent_orig is not None and parent_orig >= 0:
            # Find the closest sampled ancestor
            current = parent_orig
            while current >= 0 and current not in index_map:
                # Find parent of current node
                if current < len(tree_data) and tree_data[current].get('parent') is not None:
                    current = tree_data[current]['parent']
                else:
                    break
            if current in index_map:
                parent_sampled = index_map[current]
        
        sampled_tree.append({
            'id': sampled_idx,
            'config': node['config'].tolist(),
            'parent': parent_sampled,
            'cost': float(node['cost'])
        })
    
    # Convert path to serializable format with end-effector positions
    path_with_ee = []
    for config_item in result['full_path']:
        left_config, right_config = dual_arm._split_configuration(config_item)
        left_ee = dual_arm.left_arm.forward_kinematics(*left_config)
        right_ee = dual_arm.right_arm.forward_kinematics(*right_config)
        
        path_with_ee.append({
            'config': config_item.tolist(),
            'left_ee': (dual_arm.left_base + left_ee).tolist(),
            'right_ee': (dual_arm.right_base + right_ee).tolist()
        })
    
    return {
        'strategy': result['strategy'],
        'handoff_point': result['handoff_point'].tolist() if result['handoff_point'] is not None else None,
        'phases': result['phases'],
        'path': path_with_ee,
        'tree': {
            'nodes': sampled_tree
        },
        'metrics': {
            'planning_time': planning_time,
            'path_length': len(result['full_path']),
            'tree_size': len(tree_data),
            'iterations': iterations_count[0]
        }
    }


def generate_demo_configs():
    """Generate demo configurations."""
    
    demos = []
    
    # Demo 1: Simple - Single arm solution, no obstacles
    demos.append({
        'id': 'simple-demo',
        'metadata': {
            'name': 'Simple Reach',
            'description': 'Single arm can reach both start and goal. No obstacles.',
            'difficulty': 'simple',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.5, 1.2],
            'item_goal': [-0.3, 1.5],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [],
            'rrt_params': {
                'max_iterations': 3000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    # Demo 2: Medium - Handoff required with obstacles
    demos.append({
        'id': 'medium-handoff',
        'metadata': {
            'name': 'Handoff Required',
            'description': 'Item requires handoff between arms. One obstacle to navigate.',
            'difficulty': 'medium',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.5, 1.5],
            'item_goal': [0.5, 1.5],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [
                {'type': 'circle', 'center': [0, 1.0], 'radius': 0.3}
            ],
            'rrt_params': {
                'max_iterations': 5000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    # Demo 3: Multiple obstacles - Wall of obstacles (3 obstacles)
    demos.append({
        'id': 'wall-obstacles',
        'metadata': {
            'name': 'Obstacle Wall',
            'description': 'Navigate through a wall of 3 obstacles blocking the direct path.',
            'difficulty': 'medium',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.6, 1.6],
            'item_goal': [0.6, 1.6],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [
                {'type': 'circle', 'center': [-0.3, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [0, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [0.3, 1.0], 'radius': 0.2}
            ],
            'rrt_params': {
                'max_iterations': 6000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    # Demo 4: Medium - Extended wall (5 obstacles)
    demos.append({
        'id': 'extended-wall',
        'metadata': {
            'name': 'Extended Wall',
            'description': 'Navigate through an extended wall of 5 obstacles.',
            'difficulty': 'medium',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.6, 1.6],
            'item_goal': [0.6, 1.6],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [
                {'type': 'circle', 'center': [-0.4, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [-0.2, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [0, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [0.2, 1.0], 'radius': 0.2},
                {'type': 'circle', 'center': [0.4, 1.0], 'radius': 0.2}
            ],
            'rrt_params': {
                'max_iterations': 7000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    # Demo 5: Complex - Double wall (6 obstacles)
    demos.append({
        'id': 'double-wall',
        'metadata': {
            'name': 'Double Wall',
            'description': 'Navigate through two layers of obstacles (6 total).',
            'difficulty': 'complex',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.6, 1.6],
            'item_goal': [0.6, 1.6],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [
                {'type': 'circle', 'center': [-0.3, 0.9], 'radius': 0.2},
                {'type': 'circle', 'center': [0, 0.9], 'radius': 0.2},
                {'type': 'circle', 'center': [0.3, 0.9], 'radius': 0.2},
                {'type': 'circle', 'center': [-0.3, 1.1], 'radius': 0.2},
                {'type': 'circle', 'center': [0, 1.1], 'radius': 0.2},
                {'type': 'circle', 'center': [0.3, 1.1], 'radius': 0.2}
            ],
            'rrt_params': {
                'max_iterations': 8000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    # Demo 6: Complex - Triple wall (9 obstacles)
    demos.append({
        'id': 'triple-wall',
        'metadata': {
            'name': 'Triple Wall',
            'description': 'Navigate through three layers of obstacles (9 total).',
            'difficulty': 'complex',
            'created': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
            'version': '1.0'
        },
        'config': {
            'arm_type': '2-link',
            'arm_params': {'L1': 1.0, 'L2': 0.7},
            'item_start': [-0.6, 1.6],
            'item_goal': [0.6, 1.6],
            'left_base_x': -1.0,
            'right_base_x': 1.0,
            'obstacles': [
                {'type': 'circle', 'center': [-0.3, 0.85], 'radius': 0.18},
                {'type': 'circle', 'center': [0, 0.85], 'radius': 0.18},
                {'type': 'circle', 'center': [0.3, 0.85], 'radius': 0.18},
                {'type': 'circle', 'center': [-0.3, 1.0], 'radius': 0.18},
                {'type': 'circle', 'center': [0, 1.0], 'radius': 0.18},
                {'type': 'circle', 'center': [0.3, 1.0], 'radius': 0.18},
                {'type': 'circle', 'center': [-0.3, 1.15], 'radius': 0.18},
                {'type': 'circle', 'center': [0, 1.15], 'radius': 0.18},
                {'type': 'circle', 'center': [0.3, 1.15], 'radius': 0.18}
            ],
            'rrt_params': {
                'max_iterations': 9000,
                'step_size': 0.15,
                'goal_threshold': 0.2,
                'use_kdtree': True,
                'workspace_weight': 0.3,
                'use_adaptive_step': True
            }
        }
    })
    
    return demos


def main():
    """Main function to generate all demos."""
    print("Generating demo configurations...")
    
    demos = generate_demo_configs()
    demos_dir = Path(__file__).parent / 'demos'
    demos_dir.mkdir(exist_ok=True)
    
    for demo in demos:
        print(f"\n{'#'*60}")
        print(f"# Generating: {demo['metadata']['name']}")
        print(f"{'#'*60}")
        
        # Run planning to get results
        saved_results = run_planning(demo)
        
        if saved_results:
            demo['saved_results'] = saved_results
            
            # Save to file
            output_file = demos_dir / f"{demo['id']}.json"
            with open(output_file, 'w') as f:
                json.dump(demo, f, indent=2)
            
            print(f"\n✓ Saved to {output_file}")
        else:
            print(f"\n✗ Failed to generate {demo['id']}")
    
    print(f"\n{'='*60}")
    print("Demo generation complete!")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()

