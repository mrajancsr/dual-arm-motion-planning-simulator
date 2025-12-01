#!/bin/bash
# Convenient test runner script
# Usage: ./run_tests.sh [options]

VENV_PYTHON="./venv/bin/python"

# Check if virtual environment exists
if [ ! -f "$VENV_PYTHON" ]; then
    echo "Error: Virtual environment not found at $VENV_PYTHON"
    echo "Please create virtual environment first: python -m venv venv"
    exit 1
fi

# Parse command
case "$1" in
    "quick"|"q")
        echo "Running quick test..."
        $VENV_PYTHON test_comprehensive.py --quick
        ;;
    "optimization"|"opt")
        echo "Running optimization comparison..."
        $VENV_PYTHON test_comprehensive.py --test optimization "${@:2}"
        ;;
    "benchmark"|"bench")
        echo "Running benchmark..."
        $VENV_PYTHON test_comprehensive.py --test benchmark "${@:2}"
        ;;
    "obstacles"|"obs")
        echo "Running obstacle tests..."
        $VENV_PYTHON test_comprehensive.py --test obstacles "${@:2}"
        ;;
    "parallel"|"par")
        echo "Running parallel tests..."
        $VENV_PYTHON test_comprehensive.py --test parallel --parallel "${@:2}"
        ;;
    "all")
        echo "Running full test suite..."
        $VENV_PYTHON test_comprehensive.py "${@:2}"
        ;;
    "interactive"|"gui")
        echo "Running interactive obstacle GUI..."
        $VENV_PYTHON test_rrt_star_obstacles.py
        ;;
    "3link")
        echo "Running 3-link arm tests..."
        $VENV_PYTHON test_three_link_arm.py
        ;;
    "help"|"--help"|"-h"|"")
        echo "Usage: ./run_tests.sh [command] [options]"
        echo ""
        echo "Commands:"
        echo "  quick, q         - Quick test (2-link, baseline vs optimized)"
        echo "  optimization, opt - Compare optimization configurations"
        echo "  benchmark, bench - Benchmark all arm types"
        echo "  obstacles, obs   - Test obstacle avoidance"
        echo "  parallel, par    - Test parallel planning"
        echo "  all              - Run full test suite (default)"
        echo "  interactive, gui - Open interactive obstacle GUI"
        echo "  3link            - Test 3-link arm with visualization"
        echo "  help, -h         - Show this help message"
        echo ""
        echo "Examples:"
        echo "  ./run_tests.sh quick"
        echo "  ./run_tests.sh optimization --arm-types 2-link 3-link"
        echo "  ./run_tests.sh all --parallel"
        echo "  ./run_tests.sh gui"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Run './run_tests.sh help' for usage information"
        exit 1
        ;;
esac

