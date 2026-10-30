# Control Simulator - Quick Start Guide

## ⚡ Installation (5 minutes)

### Step 1: Install MATLAB Engine for Python
```bash
# Find your MATLAB installation
which matlab
# Should show something like: /usr/local/MATLAB/R2023b/bin/matlab

# Navigate to engine folder (adjust version number)
cd /usr/local/MATLAB/R2023b/extern/engines/python

# Install
sudo python3 setup.py install
```

### Step 2: Install Python Dependencies
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
pip3 install -r requirements.txt
```

## 🚀 Running the Simulator

### Option 1: GUI (Recommended)
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
python3 control_simulator_gui.py
```

### Option 2: Shell Script
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
./run_simulator.sh
```

## 🧪 Testing (Before First Use)

Run the test suite to verify everything works:
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
python3 test_simulator.py
```

This will:
- Start MATLAB engine (~10-15 seconds)
- Test all three controllers (PID, LQR, MPC)
- Run a quick parameter tuning
- Generate comparison plots
- Save results as `test_comparison.png`

**Expected time:** 3-5 minutes

## 📖 Basic Usage

### 1. Run a Simple Simulation

**In the GUI:**
1. Wait for "MATLAB engine ready" in status bar
2. Select **PID** algorithm
3. Keep default parameters (Kp=1.0, Ki=0.1, Kd=0.05)
4. Click **"Run Simulation"**
5. View results in "Time Response" tab
6. Check metrics in "Performance Metrics" tab

### 2. Tune Parameters Automatically

**In the GUI:**
1. Select **PID** algorithm
2. Set parameter bounds (e.g., Kp: 0.5-10.0)
3. Select objective: **ISE** (recommended for fast tuning)
4. Select method: **nelder-mead** (fastest)
5. Set max iterations: **30** (good starting point)
6. Click **"Auto-Tune Parameters"**
7. Wait for completion (~2-5 minutes)
8. Optimal parameters will load automatically
9. Click "Yes" when asked to run simulation

### 3. Compare Multiple Controllers

**Steps:**
1. Run PID simulation (keep results)
2. Switch to LQR algorithm
3. Run LQR simulation
4. Switch to MPC algorithm
5. Run MPC simulation
6. Click **"Comparison"** tab
7. View overlay of all three responses

### 4. Save and Load

**Save:**
- Click "Save Results"
- Choose location and filename (e.g., `my_tuning_session.json`)

**Load:**
- Click "Load Configuration"
- Select saved JSON file
- History and comparisons will restore

## 🎯 Example Workflows

### Workflow 1: Quick PID Tuning
```
1. Algorithm: PID
2. Bounds: Kp (0.5-10), Ki (0.01-2), Kd (0.01-1)
3. Objective: ISE
4. Method: nelder-mead
5. Iterations: 30
6. Click "Auto-Tune"
```
**Time:** ~2 minutes  
**Result:** Good general-purpose PID gains

### Workflow 2: Minimize Overshoot
```
1. Algorithm: PID
2. Bounds: Kp (0.5-5), Ki (0.01-0.5), Kd (0.05-0.5)
3. Objective: custom (weights overshoot heavily)
4. Method: differential_evolution
5. Iterations: 50
6. Click "Auto-Tune"
```
**Time:** ~5 minutes  
**Result:** Conservative PID with minimal overshoot

### Workflow 3: Compare PID vs LQR
```
1. Run PID with default params
2. Run LQR with default params (Q_pos=10, Q_vel=1, R=0.1)
3. View "Comparison" tab
4. Check metrics: which has better settling time? overshoot?
```

## 🛠️ Troubleshooting

### "MATLAB engine not ready"
**Solution:** Wait 10-15 seconds after GUI starts. Status bar shows "MATLAB engine ready"

### Simulation freezes GUI
**Shouldn't happen** - GUI is multithreaded. If it does:
- Click "Stop" button
- Check log for errors

### "Error: Service not ready"
**Cause:** MATLAB crashed or didn't start properly  
**Solution:**
```bash
# Restart GUI
# Or manually test MATLAB:
python3 -c "import matlab.engine; eng=matlab.engine.start_matlab(); print('OK')"
```

### Parameters don't make sense
**Cause:** Optimization diverged  
**Solutions:**
- Reduce parameter bounds (narrower search)
- Use nelder-mead instead of differential_evolution
- Increase iterations
- Try different initial values

### "No module named 'PyQt5'"
```bash
pip3 install PyQt5
```

## 📊 Understanding Metrics

| Metric | Good Value | What It Means |
|--------|-----------|---------------|
| `rise_time` | < 2s | How fast system responds |
| `settling_time` | < 3s | Time to reach steady-state |
| `overshoot_percent` | < 10% | How much it overshoots |
| `ise` | Minimize | Overall tracking error |
| `control_effort` | Minimize | Energy used by controller |

**Trade-offs:**
- ↓ ISE usually → ↑ Control effort (aggressive control)
- ↓ Overshoot usually → ↑ Rise time (slower response)
- Balance depends on your application!

## 💡 Tips & Tricks

1. **Start with step response** - easiest to tune
2. **Use ISE for first pass** - fastest convergence
3. **Refine with custom objective** - balance multiple goals
4. **Grid search for exploration** - see parameter landscape
5. **Save after good tuning** - don't lose progress!
6. **Compare before/after** - validate improvement

## 🔬 Advanced: Custom Control Algorithm

Want to add your own controller?

**1. Create MATLAB file:** `matlab_scripts/mycontroller_controller.m`
```matlab
function [u, state] = mycontroller_controller(error, dt, params, prev_state)
    % Your algorithm here
    % params(1), params(2), ... are tunable parameters
    u = params(1) * error;  % Simple example
    state = [];
end
```

**2. Update run_control_sim.m:**
Add case in switch statement:
```matlab
case 'mycontroller'
    [u, controller_state] = mycontroller_controller(e, dt, params, controller_state);
```

**3. Update GUI:**
Edit `control_simulator_gui.py`, in `on_algorithm_changed()`:
```python
elif algorithm == 'MyController':
    params = [
        ('Gain', 1.0, 0.1, 10.0),
    ]
```

**4. Test:**
- Restart GUI
- Select "MyController"
- Run simulation

## 📞 Getting Help

1. **Check README.md** - comprehensive documentation
2. **Run test_simulator.py** - verify setup
3. **Check MATLAB scripts** - see algorithm details
4. **Review log in GUI** - shows detailed errors

## 🎉 You're Ready!

Start with:
```bash
python3 control_simulator_gui.py
```

Try the example workflows above, then experiment with your own parameter sets and objectives.

**Have fun tuning! 🎛️**
