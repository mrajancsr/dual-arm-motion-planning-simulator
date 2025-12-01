# Dual-Arm Handoff Capability

## Current Implementation

The current RRT* planner operates in the **full configuration space** of both arms simultaneously:
- Configuration: `[θ1_left, θ2_left, ..., θ1_right, θ2_right, ...]`
- Both arms can move at the same time
- No explicit handoff logic

## What This Means for Handoffs

### ✅ What SHOULD Work:
- The planner can find paths where both arms coordinate
- If a valid path exists where one arm releases and the other grabs, the planner will find it naturally
- The arms can be in different positions at different times along the path

### ⚠️ Limitations:
- **No grasp constraints**: The planner doesn't know which arm(s) should be holding the object at each point
- **No explicit handoff detection**: It doesn't identify "this is a handoff moment"
- **Collision detection**: Arms can't pass through each other, which may prevent some handoff configurations

## To Add Explicit Handoff Support (Future Work):

### Option 1: Grasp State in Configuration
Add a discrete grasp state to the configuration:
```python
config = [θ1_left, θ2_left, θ1_right, θ2_right, grasp_state]
# grasp_state: 0=left only, 1=both, 2=right only
```

### Option 2: Task-Space Constraints
- Add constraint that at least one arm must be at the item position
- Add special handoff waypoints where both arms are at item
- Validate that item doesn't "fall" between handoffs

### Option 3: Bi-Manual Planning
- Plan in phases: left-arm-only → handoff → right-arm-only
- Use different planners for each phase
- Stitch paths together at handoff points

## Testing the Current System

To test if handoffs work with current implementation:

1. **Set up scenario**:
   - Place LEFT base on left side
   - Place RIGHT base on right side  
   - Place START ITEM near left base
   - Place GOAL ITEM near right base

2. **What should happen**:
   - Left arm picks up item
   - Left arm moves item to middle area
   - If planner finds a path, it may involve both arms at some point
   - Right arm takes over (if reachable)
   - Right arm moves to goal

3. **What might fail**:
   - If no continuous path exists in joint space
   - If collision constraints prevent certain transitions
   - If the configuration space is too complex (may need more iterations)

## Recommendation

**For now**: Test with the current implementation. The dual-arm C-space planner is quite powerful and may handle simple handoffs naturally.

**If handoffs fail**: We can add explicit grasp state tracking and handoff constraints together.

## Example Test Scenario

```javascript
// Left base at x = -1.5
// Right base at x = 1.5
// Start item at (0, 1.5)  - reachable by both
// Goal item at (0, 2.5)   - reachable by both

// The planner should find a path where:
// - One or both arms move the item from start to goal
// - Arms coordinate to avoid collisions
// - Item is always "held" by at least one arm (implicitly)
```

If this doesn't work, we'll know we need explicit handoff logic.

