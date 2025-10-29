# ✅ SOLUTION: Stage World File Path Issue

## Problem You Had

You ran:
```bash
ros2 run stage_ros2 stage_ros2 install/uncertainty_slam/share/uncertainty_slam/worlds/cave.world
```

And got:
```
[FATAL] [stage_ros2]: The world file cave.world does not exist.
```

---

## Root Cause

**stage_ros2 requires ABSOLUTE paths, not relative paths.**

When you give it a relative path like `install/uncertainty_slam/...`, it tries to find the file relative to wherever Stage internally looks, which fails.

---

## ✅ SOLUTION

Use one of these methods:

### Method 1: Use ${PWD} (Recommended)
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run stage_ros2 stage_ros2 "${PWD}/src/uncertainty_slam/worlds/cave.world"
```

`${PWD}` expands to your current directory's full path.

---

### Method 2: Use Helper Script (Easiest)
```bash
cd ~/slam_uncertainty_ws
./RUN_SYSTEM.sh
```

Then select option 1 to see all correct commands.

---

### Method 3: Hardcode Full Path
```bash
ros2 run stage_ros2 stage_ros2 "/home/rohan/slam_uncertainty_ws/src/uncertainty_slam/worlds/cave.world"
```

This always works but isn't portable.

---

## Complete Working Commands

Open 5 terminals and run:

### Terminal 1: Stage
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run stage_ros2 stage_ros2 "${PWD}/src/uncertainty_slam/worlds/cave.world"
```

### Terminal 2: SLAM
```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

### Terminal 3: Uncertainty
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run uncertainty_slam uncertainty_node
```

### Terminal 4: RViz
```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d "${PWD}/install/uncertainty_slam/share/uncertainty_slam/config/uncertainty_slam.rviz"
```

### Terminal 5: Control
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Verification

Before running, check the file exists:
```bash
cd ~/slam_uncertainty_ws
ls -l "${PWD}/src/uncertainty_slam/worlds/cave.world"
```

Should show:
```
-rw-r--r-- 1 rohan rohan 1475 Oct 28 23:43 ...cave.world
```

✅ **File exists and is ready!**

---

## Why Your Original Command Failed

```bash
# WRONG (what you tried)
ros2 run stage_ros2 stage_ros2 install/uncertainty_slam/...
                                ^^^^^^^^
                                Relative path - stage_ros2 can't find it

# CORRECT
ros2 run stage_ros2 stage_ros2 "${PWD}/src/uncertainty_slam/..."
                                ^^^^^^
                                Absolute path - always works
```

---

## Quick Test

Just test Stage first to verify it works:

```bash
cd ~/slam_uncertainty_ws
source install/setup.bash
ros2 run stage_ros2 stage_ros2 "$(pwd)/src/uncertainty_slam/worlds/cave.world"
```

**Expected:** Stage window opens showing:
- 12x12 meter room
- Gray walls
- Colored obstacle blocks
- Blue robot at position (-4, -4)

**If this works:** Continue with the other 4 terminals!

---

## All Helper Files Created

For your convenience, I've created:

1. **RUN_SYSTEM.sh** - Interactive launcher (run this!)
2. **CORRECT_COMMANDS.txt** - Copy-paste ready commands
3. **SOLUTION.md** - This file
4. **TESTING_GUIDE.md** - Complete testing guide
5. **ALL_CHANGES_COMPLETE.md** - Summary of all fixes

---

## One-Line Test Command

Want to test everything at once? Try:

```bash
cd ~/slam_uncertainty_ws && source install/setup.bash && ros2 run stage_ros2 stage_ros2 "${PWD}/src/uncertainty_slam/worlds/cave.world"
```

If Stage opens successfully, you're good to go! 🚀

---

## Summary

**Problem:** Relative paths don't work with stage_ros2
**Solution:** Use `"${PWD}/..."` for absolute paths
**Status:** ✅ Ready to test with correct commands

**Next step:** Run the helper script or use the corrected commands above!
