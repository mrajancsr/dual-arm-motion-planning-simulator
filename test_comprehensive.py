#!/usr/bin/env python3
"""
Comprehensive Test Suite for Dual-Arm Motion Planning

Tests multiple arm types (2-link, 3-link, 6-link) with various configurations
and optimization settings. Provides performance comparisons and benchmarks.
"""

import numpy as np
import time
import argparse
from typing import Dict, List, Tuple, Optional
from src import (
    DualArm, TwoLinkArm, ThreeLinkArm, SixLinkArm,
    RRTStar, ParallelRRTStar
)


# ============================================================================
# Configuration Presets
# ============================================================================

ARM_CONFIGS = {
    '2-link': {
        'class': TwoLinkArm,
        'params': {'L1': 1.0, 'L2': 0.7},
        'start': np.array([0.5, 0.3, -0.2, 0.4]),
        'goal': np.array([1.0, -0.5, 0.8, -0.3]),
        'separation': 2.0,
        'max_iterations': 3000,
        'step_size': 0.15,
        'goal_threshold': 0.2
    },
    '3-link': {
        'class': ThreeLinkArm,
        'params': {'L1': 1.0, 'L2': 0.8, 'L3': 0.6},
        'start': np.array([0.3, 0.2, 0.1, -0.2, 0.1, -0.1]),
        'goal': np.array([0.8, -0.3, 0.2, 0.5, -0.2, 0.3]),
        'separation': 2.5,
        'max_iterations': 4000,
        'step_size': 0.12,
        'goal_threshold': 0.25
    },
    '6-link': {
        'class': SixLinkArm,
        'params': {'L1': 0.5, 'L2': 0.4, 'L3': 0.3, 'L4': 0.25, 'L5': 0.2, 'L6': 0.15},
        'start': np.array([0.3, 0.2, 0.1, -0.1, 0.15, -0.05, -0.2, 0.1, -0.1, 0.05, -0.15, 0.1]),
        'goal': np.array([0.8, -0.3, 0.2, 0.1, -0.2, 0.15, 0.5, -0.2, 0.3, -0.1, 0.1, -0.05]),
        'separation': 2.0,
        'max_iterations': 8000,
        'step_size': 0.08,
        'goal_threshold': 0.3
    }
}

OPTIMIZATION_CONFIGS = {
    'baseline': {
        'use_kdtree': False,
        'workspace_weight': 0.0,
        'use_adaptive_step': False
    },
    'kdtree-only': {
        'use_kdtree': True,
        'workspace_weight': 0.0,
        'use_adaptive_step': False
    },
    'distance-only': {
        'use_kdtree': False,
        'workspace_weight': 0.3,
        'use_adaptive_step': False
    },
    'adaptive-only': {
        'use_kdtree': False,
        'workspace_weight': 0.0,
        'use_adaptive_step': True
    },
    'all-optimizations': {
        'use_kdtree': True,
        'workspace_weight': 0.3,
        'use_adaptive_step': True
    }
}


# ============================================================================
# Test Functions
# ============================================================================

def create_dual_arm(arm_type: str, obstacles: Optional[List[Dict]] = None) -> DualArm:
    """Create a dual-arm system of the specified type."""
    config = ARM_CONFIGS[arm_type]
    
    left_arm = config['class'](**config['params'], name=f"Left{arm_type}")
    right_arm = config['class'](**config['params'], name=f"Right{arm_type}")
    
    return DualArm(
        left_arm=left_arm,
        right_arm=right_arm,
        separation=config['separation'],
        obstacles=obstacles
    )


def test_basic_planning(arm_type: str, opt_config: str, verbose: bool = False) -> Dict:
    """
    Test basic RRT* planning without obstacles.
    
    Returns:
        Dictionary with results including success, time, path_length, etc.
    """
    if verbose:
        print(f"\n{'='*70}")
        print(f"Testing {arm_type} with {opt_config} optimization")
        print(f"{'='*70}")
    
    # Create dual-arm system
    dual_arm = create_dual_arm(arm_type)
    arm_config = ARM_CONFIGS[arm_type]
    opt_params = OPTIMIZATION_CONFIGS[opt_config]
    
    # Get start and goal
    start = arm_config['start']
    goal = arm_config['goal']
    
    if verbose:
        print(f"Configuration space: {len(start)}D")
        print(f"Optimizations: {opt_config}")
    
    # Create planner
    planner = RRTStar(
        dual_arm,
        max_iterations=arm_config['max_iterations'],
        step_size=arm_config['step_size'],
        goal_threshold=arm_config['goal_threshold'],
        verbose=False,
        **opt_params
    )
    
    # Plan
    start_time = time.time()
    path = planner.plan(start, goal)
    planning_time = time.time() - start_time
    
    # Results
    result = {
        'arm_type': arm_type,
        'optimization': opt_config,
        'success': path is not None,
        'planning_time': planning_time,
        'path_length': len(path) if path else 0,
        'iterations_max': arm_config['max_iterations']
    }
    
    if verbose:
        if path:
            print(f"✓ Path found: {len(path)} waypoints in {planning_time:.3f}s")
        else:
            print(f"✗ Failed in {planning_time:.3f}s")
    
    return result


def test_obstacle_planning(arm_type: str, verbose: bool = False) -> Dict:
    """Test planning with obstacles (uses all optimizations)."""
    if verbose:
        print(f"\n{'='*70}")
        print(f"Testing {arm_type} with obstacles")
        print(f"{'='*70}")
    
    # Create obstacles
    obstacles = [
        {'type': 'circle', 'center': [0.0, 1.0], 'radius': 0.3},
    ]
    
    dual_arm = create_dual_arm(arm_type, obstacles=obstacles)
    arm_config = ARM_CONFIGS[arm_type]
    
    start = arm_config['start']
    goal = arm_config['goal']
    
    # Use all optimizations for obstacle scenarios
    planner = RRTStar(
        dual_arm,
        max_iterations=arm_config['max_iterations'] * 2,  # More iterations for obstacles
        step_size=arm_config['step_size'],
        goal_threshold=arm_config['goal_threshold'],
        use_kdtree=True,
        workspace_weight=0.3,
        use_adaptive_step=True,  # Critical for obstacle navigation
        verbose=False
    )
    
    start_time = time.time()
    path = planner.plan(start, goal)
    planning_time = time.time() - start_time
    
    # Verify no collisions
    all_valid = all(dual_arm.is_valid_configuration(config) for config in path) if path else False
    
    result = {
        'arm_type': arm_type,
        'success': path is not None,
        'planning_time': planning_time,
        'path_length': len(path) if path else 0,
        'all_valid': all_valid
    }
    
    if verbose:
        if path:
            print(f"✓ Path found: {len(path)} waypoints in {planning_time:.3f}s")
            print(f"  All configurations valid: {all_valid}")
        else:
            print(f"✗ Failed in {planning_time:.3f}s")
    
    return result


def test_parallel_planning(arm_type: str, num_workers: int = 4, verbose: bool = False) -> Dict:
    """Test parallel RRT* planning."""
    if verbose:
        print(f"\n{'='*70}")
        print(f"Testing {arm_type} with parallel planning ({num_workers} workers)")
        print(f"{'='*70}")
    
    dual_arm = create_dual_arm(arm_type)
    arm_config = ARM_CONFIGS[arm_type]
    
    start = arm_config['start']
    goal = arm_config['goal']
    
    # Create parallel planner
    planner = ParallelRRTStar(
        dual_arm,
        num_workers=num_workers,
        max_iterations=arm_config['max_iterations'],
        step_size=arm_config['step_size'],
        goal_threshold=arm_config['goal_threshold'],
        verbose=False
    )
    
    start_time = time.time()
    path = planner.plan(start, goal, verbose=verbose)
    planning_time = time.time() - start_time
    
    result = {
        'arm_type': arm_type,
        'num_workers': num_workers,
        'success': path is not None,
        'planning_time': planning_time,
        'path_length': len(path) if path else 0
    }
    
    if verbose and not verbose:  # Only if not already printed by planner
        if path:
            print(f"✓ Path found: {len(path)} waypoints in {planning_time:.3f}s")
        else:
            print(f"✗ Failed in {planning_time:.3f}s")
    
    return result


# ============================================================================
# Comparison and Analysis
# ============================================================================

def compare_optimizations(arm_type: str, verbose: bool = True):
    """Compare all optimization configurations for a given arm type."""
    if verbose:
        print(f"\n{'='*70}")
        print(f"OPTIMIZATION COMPARISON: {arm_type}")
        print(f"{'='*70}")
    
    results = []
    
    for opt_name in OPTIMIZATION_CONFIGS.keys():
        result = test_basic_planning(arm_type, opt_name, verbose=False)
        results.append(result)
        
        if verbose:
            status = "✓" if result['success'] else "✗"
            print(f"{status} {opt_name:20s}: {result['planning_time']:6.3f}s, "
                  f"{result['path_length']:3d} waypoints")
    
    # Find baseline and best
    baseline = next(r for r in results if r['optimization'] == 'baseline')
    best = min([r for r in results if r['success']], 
               key=lambda x: x['planning_time'], default=None)
    
    if verbose and baseline['success'] and best:
        speedup = baseline['planning_time'] / best['planning_time']
        path_improvement = (baseline['path_length'] - best['path_length']) / baseline['path_length'] * 100
        
        print(f"\n📊 Best configuration: {best['optimization']}")
        print(f"   Speedup: {speedup:.2f}x")
        print(f"   Path improvement: {path_improvement:.1f}%")
    
    return results


def benchmark_all_arms(verbose: bool = True):
    """Benchmark all arm types with best optimization settings."""
    if verbose:
        print(f"\n{'='*70}")
        print(f"BENCHMARK: All Arm Types (with all optimizations)")
        print(f"{'='*70}")
    
    results = []
    
    for arm_type in ARM_CONFIGS.keys():
        result = test_basic_planning(arm_type, 'all-optimizations', verbose=False)
        results.append(result)
        
        if verbose:
            status = "✓" if result['success'] else "✗"
            config_dim = len(ARM_CONFIGS[arm_type]['start'])
            print(f"{status} {arm_type:10s} ({config_dim:2d}D): {result['planning_time']:6.3f}s, "
                  f"{result['path_length']:3d} waypoints")
    
    return results


# ============================================================================
# Main Test Runner
# ============================================================================

def run_all_tests(arm_types: List[str] = None, test_parallel: bool = False):
    """Run comprehensive test suite."""
    if arm_types is None:
        arm_types = list(ARM_CONFIGS.keys())
    
    print("\n" + "="*70)
    print("COMPREHENSIVE DUAL-ARM MOTION PLANNING TEST SUITE")
    print("="*70)
    
    all_results = {}
    
    # 1. Basic planning comparison
    print("\n" + "="*70)
    print("TEST 1: Optimization Comparison")
    print("="*70)
    
    for arm_type in arm_types:
        results = compare_optimizations(arm_type, verbose=True)
        all_results[f'{arm_type}_optimizations'] = results
    
    # 2. Benchmark all arms
    benchmark_results = benchmark_all_arms(verbose=True)
    all_results['benchmark'] = benchmark_results
    
    # 3. Obstacle avoidance
    print("\n" + "="*70)
    print("TEST 2: Obstacle Avoidance")
    print("="*70)
    
    obstacle_results = []
    for arm_type in arm_types:
        result = test_obstacle_planning(arm_type, verbose=True)
        obstacle_results.append(result)
    all_results['obstacles'] = obstacle_results
    
    # 4. Parallel planning (optional)
    if test_parallel:
        print("\n" + "="*70)
        print("TEST 3: Parallel Planning")
        print("="*70)
        
        parallel_results = []
        for arm_type in arm_types:
            result = test_parallel_planning(arm_type, num_workers=4, verbose=True)
            parallel_results.append(result)
        all_results['parallel'] = parallel_results
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    total_tests = sum(len(v) if isinstance(v, list) else 1 
                     for v in all_results.values())
    successful_tests = sum(
        sum(1 for r in v if r.get('success', False)) if isinstance(v, list)
        else (1 if v.get('success', False) else 0)
        for v in all_results.values()
    )
    
    print(f"Total tests: {total_tests}")
    print(f"Successful: {successful_tests}")
    print(f"Success rate: {successful_tests/total_tests*100:.1f}%")
    
    return all_results


# ============================================================================
# Command Line Interface
# ============================================================================

def main():
    """Main entry point with argument parsing."""
    parser = argparse.ArgumentParser(
        description='Comprehensive test suite for dual-arm motion planning'
    )
    
    parser.add_argument(
        '--arm-types',
        nargs='+',
        choices=['2-link', '3-link', '6-link'],
        default=['2-link', '3-link', '6-link'],
        help='Arm types to test (default: all)'
    )
    
    parser.add_argument(
        '--test',
        choices=['optimization', 'benchmark', 'obstacles', 'parallel', 'all'],
        default='all',
        help='Which test to run (default: all)'
    )
    
    parser.add_argument(
        '--parallel',
        action='store_true',
        help='Include parallel planning tests'
    )
    
    parser.add_argument(
        '--quick',
        action='store_true',
        help='Quick test (only 2-link arm, baseline vs all-optimizations)'
    )
    
    args = parser.parse_args()
    
    # Quick mode
    if args.quick:
        print("\n🚀 Quick Test Mode")
        arm_type = '2-link'
        
        print("\n1. Baseline:")
        baseline = test_basic_planning(arm_type, 'baseline', verbose=True)
        
        print("\n2. All Optimizations:")
        optimized = test_basic_planning(arm_type, 'all-optimizations', verbose=True)
        
        if baseline['success'] and optimized['success']:
            speedup = baseline['planning_time'] / optimized['planning_time']
            improvement = (baseline['path_length'] - optimized['path_length']) / baseline['path_length'] * 100
            print(f"\n📊 Results:")
            print(f"   Speedup: {speedup:.2f}x")
            print(f"   Path improvement: {improvement:.1f}%")
        
        return
    
    # Full test suite
    if args.test == 'all':
        run_all_tests(arm_types=args.arm_types, test_parallel=args.parallel)
    elif args.test == 'optimization':
        for arm_type in args.arm_types:
            compare_optimizations(arm_type, verbose=True)
    elif args.test == 'benchmark':
        benchmark_all_arms(verbose=True)
    elif args.test == 'obstacles':
        for arm_type in args.arm_types:
            test_obstacle_planning(arm_type, verbose=True)
    elif args.test == 'parallel':
        for arm_type in args.arm_types:
            test_parallel_planning(arm_type, verbose=True)


if __name__ == "__main__":
    main()

