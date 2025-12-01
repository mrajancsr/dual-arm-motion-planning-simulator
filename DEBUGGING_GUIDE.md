# Debugging Guide - Handoff Planning

## Quick Debugging Steps

### 1. Restart the Web App

The backend has been updated with better logging. Restart to see progress:

```bash
# Stop everything
pkill -f "python.*run_server"
pkill -f "npm.*dev"

# Clean logs
rm -f backend.log frontend.log

# Restart
./run_webapp.sh
```

### 2. Monitor Logs in Real-Time

Open two terminal windows:

**Terminal 1 - Backend Log:**
```bash
tail -f backend.log
```

**Terminal 2 - Frontend Log:**
```bash
tail -f frontend.log
```

### 3. Test Planning

1. Open browser: http://localhost:5173
2. Drag bases to opposite sides (e.g., left at -1.5, right at 1.5)
3. Drag START item to left side (e.g., [-0.8, 1.2])
4. Drag GOAL item to right side (e.g., [0.8, 1.2])
5. Click "Start Planning"

### 4. What You Should See

**In Backend Log:**
```
[Handoff Planning] Starting for job <uuid>
  Item start: [-0.8  1.2]
  Item goal: [0.8 1.2]
  Left base: [-1.5, 0]
  Right base: [1.5, 0]
[Handoff Planning] Creating HandoffPlanner...
[Handoff Planning] Executing plan_with_handoff...
[HandoffPlanner] Strategy: {...}
[HandoffPlanner] Handoff required: left -> right
[HandoffPlanner] Finding handoff point...
[find_handoff_point] Left workspace: 900 points
[find_handoff_point] Right workspace: 900 points
[find_handoff_point] Intersection size: 112
[HandoffPlanner] Handoff point: [-0.1  1.2]
[HandoffPlanner] Planning grab phase (left arm)...
[_get_config_for_position] left arm: global=[-0.8  1.2], local=[0.2 1.2]
[_get_config_for_position] ✓ Valid config found
[RRT* Single Arm] Planning for left arm
  Goal reached at iteration X
[HandoffPlanner] Grab phase complete: X waypoints
[HandoffPlanner] Planning delivery phase (right arm)...
[RRT* Single Arm] Planning for right arm
[HandoffPlanner] Delivery phase complete: X waypoints
[Handoff Planning] SUCCESS - Path found with X waypoints
  Strategy: {...}
  Handoff point: [-0.1, 1.2]
  Phases: [...]
```

**In Browser UI:**
- Status indicator shows "Planning..."
- Progress bar updates
- Current phase message (e.g., "Analyzing workspace...")
- Handoff Plan panel appears when complete
- Handoff point visualized on canvas

## Common Issues & Solutions

### Issue 1: No Logs Appearing

**Symptom:** Backend log is empty or only shows HTTP requests

**Solution:**
```bash
# Check if using venv Python
which python  # Should be in project's venv

# Make sure backend is using venv
cd backend
../venv/bin/python run_server.py
```

### Issue 2: "Module not found" Error

**Symptom:** `ModuleNotFoundError: No module named 'src.handoff_planner'`

**Solution:**
```bash
# Reinstall in development mode
./venv/bin/pip install -e .

# Verify
./venv/bin/python -c "from src import HandoffPlanner; print('✓ OK')"
```

### Issue 3: Handoff Plan Not Showing in UI

**Symptom:** Planning completes but no handoff plan panel

**Check:**
1. Open browser console (F12)
2. Look for errors
3. Check network tab for `/api/status/<job_id>` response
4. Verify response includes `strategy`, `handoff_point`, `phases`

**Fix:**
```javascript
// In browser console, check the response:
fetch('http://localhost:5001/api/status/<job_id>')
  .then(r => r.json())
  .then(console.log)
```

### Issue 4: Planning Takes Too Long

**Symptom:** Planner runs but never completes

**Solutions:**
1. **Reduce iterations:**
   ```jsx
   // In frontend/src/App.jsx, reduce max_iterations
   max_iterations: 1000  // Instead of 5000
   ```

2. **Check if items are unreachable:**
   - Verify items are within workspace circles
   - Check if workspace circles overlap (for handoff)

3. **Simplify scenario:**
   - Remove all obstacles
   - Place items closer to bases
   - Ensure clear handoff region

### Issue 5: "No workspace intersection found"

**Symptom:** Error: `No workspace intersection found between arms`

**Solution:**
- Move arm bases closer together
- Typical separation: 2.0 to 3.0 units
- Workspaces must overlap for handoff

## Diagnostic Commands

### Check Backend Status

```bash
# Is backend running?
ps aux | grep "python.*run_server"

# Which Python?
ps aux | grep "python.*run_server" | awk '{print $2}' | xargs lsof -p | grep "/Python"

# Check imports
./venv/bin/python -c "
from src import HandoffPlanner, DualArm, TwoLinkArm
print('All imports successful')
"
```

### Check Frontend Status

```bash
# Is frontend running?
ps aux | grep "npm.*dev"

# Check browser
curl http://localhost:5173
```

### Test Handoff Planner Directly

```bash
./venv/bin/python <<EOF
from src import DualArm, TwoLinkArm, HandoffPlanner
import numpy as np

left_arm = TwoLinkArm(L1=1.0, L2=0.7)
right_arm = TwoLinkArm(L1=1.0, L2=0.7)
dual_arm = DualArm(left_arm, right_arm, separation=2.0)

planner = HandoffPlanner(dual_arm)
strategy = planner.determine_arms(
    np.array([-0.8, 1.2]),  # Start
    np.array([0.8, 1.2])    # Goal
)
print(f"Strategy: {strategy}")
EOF
```

## Performance Tips

### Speed Up Planning

1. **Use KD-tree:** Already enabled by default
2. **Reduce max_iterations:** Start with 1000, increase if needed
3. **Increase step_size:** Try 0.2 instead of 0.15
4. **Simplify obstacles:** Remove unnecessary obstacles

### Improve Success Rate

1. **Ensure reachability:** Items within workspace circles
2. **Clear handoff region:** Workspaces must overlap
3. **Avoid tight spaces:** Give arms room to maneuver
4. **Start with simple scenarios:** No obstacles first

## Expected Performance

### Timing (typical 2-link handoff scenario):

- **Workspace analysis:** < 0.1s
- **Handoff point selection:** < 0.1s
- **Grab phase (500 iter):** 0.5-2s
- **Delivery phase (500 iter):** 0.5-2s
- **Total:** 1-4 seconds

### Success Rates:

- **Simple handoff (no obstacles):** ~95%
- **With obstacles:** ~70-80%
- **Tight workspaces:** ~50-60%

## Logging Improvements Made

### Added flush=True to all print statements:
- Backend planning worker
- HandoffPlanner methods
- RRT* progress updates

### Added progress phases:
- "Analyzing workspace..."
- "Planning grab phase..."
- "Planning delivery phase..."
- "Stitching paths..."

### Added error tracking:
- Full traceback in job['error_trace']
- Detailed validation errors
- IK failure messages

## Next Steps if Still Not Working

1. **Collect full logs:**
   ```bash
   ./run_webapp.sh > output.log 2>&1
   ```

2. **Test each component:**
   - Workspace generation
   - Arm determination
   - Handoff point selection
   - Single-arm RRT*

3. **Check browser console:**
   - Network errors
   - JavaScript errors
   - API response format

4. **Restart everything fresh:**
   ```bash
   pkill -f "python"
   pkill -f "npm"
   rm -f backend.log frontend.log
   rm -f .backend.pid .frontend.pid
   ./run_webapp.sh
   ```

## Summary

The main updates for better debugging:

✅ All print statements now flush immediately  
✅ Progress phases show current activity  
✅ Error tracebacks saved in job results  
✅ Frontend displays current phase  
✅ Real-time log monitoring possible  

**Now restart the app and you should see detailed progress!** 🚀

