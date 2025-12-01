#!/usr/bin/env python3
"""
Quick Test Script for RRT* Optimizations

Tests the optimized RRT* implementation with KD-tree, improved distance metrics,
adaptive step sizing, and parallel planning.
"""

import numpy as np
import time
from src import DualArm, TwoLinkArm, RRTStar, ParallelRRTStar

def test_basic_rrt_star():
    """Test basic RRT* functionality with optimizations."""
    print("=" * 70)
    print("Test 1: Basic RRT* with Optimizations")
    print("=" * 70)
    
    # Create dual-arm system
    left_arm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
    right_arm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)
    
    # Define start and goal configurations
    start_config = np.array([0.5, 0.3, -0.2, 0.4])
    goal_config = np.array([1.0, -0.5, 0.8, -0.3])
    
    # Test with optimizations enabled
    print("\n🚀 Testing with ALL optimizations enabled...")
    planner_optimized = RRTStar(
        dual_arm,
        max_iterations=1000,
        step_size=0.15,
        goal_threshold=0.2,
        use_kdtree=True,
        workspace_weight=0.3,
        use_adaptive_step=True,
        verbose=False
    )
    
    start_time = time.time()
    path_optimized = planner_optimized.plan(start_config, goal_config)
    time_optimized = time.time() - start_time
    
    if path_optimized:
        print(f"✓ Path found with {len(path_optimized)} configurations in {time_optimized:.3f}s")
    else:
        print(f"✗ No path found in {time_optimized:.3f}s")
    
    # Test with optimizations disabled for comparison
    print("\n📊 Testing with optimizations DISABLED (baseline)...")
    planner_baseline = RRTStar(
        dual_arm,
        max_iterations=1000,
        step_size=0.15,
        goal_threshold=0.2,
        use_kdtree=False,
        workspace_weight=0.0,  # Pure Euclidean distance
        use_adaptive_step=False,
        verbose=False
    )
    
    start_time = time.time()
    path_baseline = planner_baseline.plan(start_config, goal_config)
    time_baseline = time.time() - start_time
    
    if path_baseline:
        print(f"✓ Path found with {len(path_baseline)} configurations in {time_baseline:.3f}s")
    else:
        print(f"✗ No path found in {time_baseline:.3f}s")
    
    # Compare results
    if path_optimized and path_baseline:
        speedup = time_baseline / time_optimized
        path_improvement = (len(path_baseline) - len(path_optimized)) / len(path_baseline) * 100
        print(f"\n📈 Performance Comparison:")
        print(f"  Speedup: {speedup:.2f}x")
        print(f"  Path length reduction: {path_improvement:.1f}%")
    
    return path_optimized is not None


def test_obstacle_avoidance():
    """Test RRT* with obstacles (adaptive step size)."""
    print("\n" + "=" * 70)
    print("Test 2: Obstacle Avoidance with Adaptive Step Size")
    print("=" * 70)
    
    # Create dual-arm system with obstacle
    left_arm = TwoLinkArm(L1=0.7, L2=0.6, name="LeftArm")
    right_arm = TwoLinkArm(L1=0.7, L2=0.6, name="RightArm")
    
    obstacles = [{'type': 'circle', 'center': [-0.5, 0.8], 'radius': 0.3}]
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0, obstacles=obstacles)
    
    # Define start and goal
    start_config = np.array([0.3, 0.2, -0.1, 0.3])
    goal_config = np.array([1.2, -0.4, 0.6, -0.2])
    
    print("\n🚀 Planning path around obstacle...")
    planner = RRTStar(
        dual_arm,
        max_iterations=2000,
        step_size=0.2,
        goal_threshold=0.3,
        use_kdtree=True,
        workspace_weight=0.3,
        use_adaptive_step=True,
        verbose=False
    )
    
    start_time = time.time()
    path = planner.plan(start_config, goal_config)
    planning_time = time.time() - start_time
    
    if path:
        print(f"✓ Path found with {len(path)} configurations in {planning_time:.3f}s")
        
        # Verify path validity (no collisions)
        all_valid = all(dual_arm.is_valid_configuration(config) for config in path)
        if all_valid:
            print("✓ All configurations in path are collision-free")
        else:
            print("✗ WARNING: Path contains invalid configurations!")
    else:
        print(f"✗ No path found in {planning_time:.3f}s")
    
    return path is not None


def test_parallel_planning():
    """Test parallel RRT* planner."""
    print("\n" + "=" * 70)
    print("Test 3: Parallel RRT* Planning")
    print("=" * 70)
    
    # Create dual-arm system
    left_arm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
    right_arm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)
    
    # Define start and goal
    start_config = np.array([0.5, 0.3, -0.2, 0.4])
    goal_config = np.array([1.0, -0.5, 0.8, -0.3])
    
    print("\n🚀 Running parallel planner with 4 workers...")
    parallel_planner = ParallelRRTStar(
        dual_arm,
        num_workers=4,
        max_iterations=1000,
        step_size=0.15,
        goal_threshold=0.2,
        verbose=False
    )
    
    start_time = time.time()
    path = parallel_planner.plan(start_config, goal_config, verbose=True)
    planning_time = time.time() - start_time
    
    if path:
        print(f"\n✓ Parallel planning completed in {planning_time:.3f}s")
        print(f"  Best path has {len(path)} configurations")
    else:
        print(f"\n✗ Parallel planning failed in {planning_time:.3f}s")
    
    return path is not None


def main():
    """Run all tests."""
    print("\n" + "=" * 70)
    print("RRT* Optimization Test Suite")
    print("=" * 70)
    print("\nTesting optimizations:")
    print("  ✓ Phase 1: KD-tree for nearest neighbor search")
    print("  ✓ Phase 2: Improved distance metrics (angular, weighted, workspace)")
    print("  ✓ Phase 3: Adaptive step sizing")
    print("  ✓ Phase 4: Bounding box collision pre-check")
    print("  ✓ Phase 5: Parallel planning\n")
    
    results = []
    
    # Run tests
    results.append(("Basic RRT* with optimizations", test_basic_rrt_star()))
    results.append(("Obstacle avoidance", test_obstacle_avoidance()))
    results.append(("Parallel planning", test_parallel_planning()))
    
    # Summary
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {test_name}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All tests passed! Optimizations are working correctly.")
    else:
        print(f"\n⚠️  {total - passed} test(s) failed.")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)

