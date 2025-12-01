# 🔍 IK Reachability Debug

## Problem

Getting "Item start or goal position is unreachable by both arms" even for seemingly valid configurations.

## What I Added

Detailed logging to `check_reachability()` to see exactly why IK is failing:

```python
def check_reachability(self, point: np.ndarray, verbose: bool = False) -> Dict:
    # ... distance checks ...
    
    if verbose:
        print(f"Left IK FAILED (returned None) for global {point}, local {local_left}, dist={dist_left:.3f}")
        print(f"Right IK SUCCESS for global {point}, local {local_right}")
        # etc.
```

## How to Debug

1. **Web app is running** - Open http://localhost:5173

2. **Set up a scenario:**
   - Drag the item start (green circle) somewhere
   - Drag the item goal (red circle) somewhere  
   - Make sure both look reachable in the UI

3. **Click "Start Planning"**

4. **Check the backend logs:**
   ```bash
   tail -f backend.log
   ```

5. **Look for these lines:**
   ```
   [determine_arms] Checking start pos [-1.378, 0.755]...
   [check_reachability] Left IK FAILED (returned None) for global [-1.378 0.755], local [0.481 0.755], dist=0.895
   [check_reachability] Right IK FAILED (returned None) for global [-1.378 0.755], local [-2.237 0.755], dist=2.360
   [determine_arms] Start reach: left=False, right=False
   ```

## What to Look For

### If IK fails unexpectedly:

**Check 1: Is the point in front of the base?**
- Local X should be > 0 (in front of arm)
- If local X < 0, point is behind the base → IK will fail

**Check 2: Is distance within range?**
- For 2-link arm: L1=1.0, L2=0.7 → max_reach=1.7, min_reach=0.3
- Distance should be: 0.3 ≤ dist ≤ 1.7

**Check 3: Is IK returning None when it shouldn't?**
- If distance is OK and point is in front, but IK still returns None
- This might be a bug in the IK implementation itself

### If both arms can't reach:

**Scenario:** Item at [0, 2.5], bases at [-1, 0] and [1, 0]
- Left dist: sqrt(1 + 6.25) = 2.69 > 1.7 ❌
- Right dist: sqrt(1 + 6.25) = 2.69 > 1.7 ❌
- **Valid failure** - item is too far!

### If UI shows something different:

**Coordinate mismatch possibilities:**
1. Frontend drawing arms with different bases than backend?
2. Item positions sent to backend don't match UI positions?
3. Arm link lengths different between frontend and backend?

## Expected Output (Working Scenario)

```bash
[Handoff Planning] Starting for job abc123...
  Item start: [0. 1.5]
  Item goal: [0. 2.5]
  Left base: [-1, 0]
  Right base: [1, 0]

[determine_arms] Checking start pos [0. 1.5]...
[check_reachability] Left IK SUCCESS for global [0. 1.5], local [1. 1.5]
[check_reachability] Right IK SUCCESS for global [0. 1.5], local [-1. 1.5]
[determine_arms] Start reach: left=True, right=True

[determine_arms] Checking goal pos [0. 2.5]...
[check_reachability] Left IK SUCCESS for global [0. 2.5], local [1. 2.5]
[check_reachability] Right IK FAILED (returned None) for global [0. 2.5], local [-1. 2.5], dist=2.693
[determine_arms] Goal reach: left=True, right=False

[Handoff Planning] Strategy: {'grab_arm': 'either', 'delivery_arm': 'left', ...}
```

## Next Steps

Once you try planning and I see the detailed logs, I can determine:

1. Is IK actually failing (and why)?
2. Is there a coordinate system mismatch?
3. Is there a bug in the IK implementation?
4. Are the arm link lengths correct?

**Try it now and share the backend.log output!**

