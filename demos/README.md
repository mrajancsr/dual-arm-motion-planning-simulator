# Demo Configurations

This directory contains predefined demo configurations for the dual-arm motion planning simulator.

## Demo JSON Schema

Each demo JSON file has the following structure:

```json
{
  "id": "unique-demo-id",
  "metadata": {
    "name": "Demo Name",
    "description": "Brief description of the demo scenario",
    "difficulty": "simple|medium|complex",
    "created": "ISO timestamp",
    "version": "1.0"
  },
  "config": {
    "arm_type": "2-link|3-link|6-link",
    "arm_params": {
      "L1": 1.0,
      "L2": 0.7,
      ...
    },
    "item_start": [x, y],
    "item_goal": [x, y],
    "left_base_x": -1.0,
    "right_base_x": 1.0,
    "obstacles": [
      {
        "type": "circle",
        "center": [x, y],
        "radius": r
      }
    ],
    "rrt_params": {
      "max_iterations": 5000,
      "step_size": 0.15,
      "goal_threshold": 0.2,
      "use_kdtree": true,
      "workspace_weight": 0.3,
      "use_adaptive_step": true
    }
  },
  "saved_results": {
    "strategy": {
      "needs_handoff": true/false,
      "grab_arm": "left|right",
      "delivery_arm": "left|right",
      "chosen_arm": "left|right"
    },
    "handoff_point": [x, y] or null,
    "phases": [
      {
        "arm": "left|right",
        "from": "start|handoff",
        "to": "handoff|goal",
        "path_length": n
      }
    ],
    "path": [...],
    "tree": {
      "nodes": [...]
    },
    "metrics": {
      "planning_time": seconds,
      "path_length": n,
      "tree_size": n,
      "iterations": n
    }
  }
}
```

## Available Demos

1. **simple-demo.json** - Single arm solution, no obstacles
2. **medium-handoff.json** - Handoff required with obstacles
3. **complex-obstacles.json** - Complex multi-obstacle scenario

## Adding New Demos

Users can save their own demos through the web interface. Custom demos are saved as `custom-<uuid>.json`.


