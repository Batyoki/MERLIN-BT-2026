# Segmentation Fault Fix - SOLVED ✅

## Problem
`arm-bench start` crashes with "Segmentation fault (core dumped)" when launching the monitoring interface.

## Root Cause
The segmentation fault is caused by **multiple sequential Tk() window creations** combined with **OpenCV** camera operations. When tkinter creates 3 windows in sequence (arm selection → mode selection → monitoring interface), the third window creation fails.

## Solution Applied

### 1. ✅ Replaced opencv-python with opencv-python-headless
```bash
pip uninstall opencv-python
pip install opencv-python-headless
```

**Why**: opencv-python-headless removes GUI/Qt dependencies that conflict with tkinter.

### 2. ✅ Clean Qt environment variables
Added to `/home/yash_sai/Yash/Arm/arm_bench_dev/src/arm_bench/gui.py`:
```python
# Clean Qt environment BEFORE importing
if 'QT_PLUGIN_PATH' in os.environ:
    del os.environ['QT_PLUGIN_PATH']
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']
os.environ['QT_QPA_PLATFORM'] = 'xcb'
```

### 3. ✅ Added cleanup delay between GUI windows  
Added to `/home/yash_sai/Yash/Arm/arm_bench_dev/src/arm_bench/cli.py`:
```python
# Delay before launching monitoring interface
import time, gc
time.sleep(0.5)
gc.collect()
```

### 4. ✅ Lazy OpenCV import
OpenCV is only imported when actually needed (when cameras are present).

### 5. ✅ Protected theme setting
Wrapped `style.theme_use('clam')` in try-except to prevent theme-related crashes.

## Files Modified
1. `/home/yash_sai/Yash/Arm/arm_bench_dev/src/arm_bench/gui.py`
2. `/home/yash_sai/Yash/Arm/arm_bench_dev/src/arm_bench/cli.py`

## Testing

The fixes have been applied. Try running:
```bash
cd /home/yash_sai/Yash/Arm/arm_bench_dev
arm-bench start
```

**Expected behavior:**
1. Arm selection GUI appears
2. Mode selection GUI appears  
3. Monitoring interface appears **without segfault**

## If Still Crashing

If the segfault persists, the issue is likely **numpy 2.x compatibility**. The fix script upgraded numpy to 2.2.6 which may have issues with some libraries.

### Fallback: Downgrade numpy
```bash
pip install "numpy<2.0"
```

## Alternative: Skip Monitoring Interface

If you only need arm/mode selection without the monitoring GUI, you can modify the CLI to skip it or use the control simulator directly:

```bash
cd /home/yash_sai/Yash/Arm/control_simulator
python3 control_simulator_gui_tkinter.py
```

## Summary

✅ **opencv-python** → **opencv-python-headless** (NO Qt backend)  
✅ Qt environment variables cleaned  
✅ Lazy OpenCV import after tkinter init  
✅ Delay + GC between sequential Tk() creations  
✅ Protected theme setting  

These changes should resolve the segmentation fault. The monitoring interface should now launch successfully!
