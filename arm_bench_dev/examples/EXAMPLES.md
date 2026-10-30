# Control Algorithm Development Studio - Examples

## Quick Start Examples

### Example 1: Basic PD Controller

This is the simplest controller - proportional and derivative feedback.

**Load Configuration:**
```bash
# From the studio: File → Load Config → pd_controller_config.json
```

**Or manually create:**

1. **Control Logic:**
```python
# PD Controller for Position Control
error = reference - state[0]
error_dot = -state[1]

control_output = Kp * error + Kd * error_dot

# Saturation
max_control = 10.0
control_output = np.clip(control_output, -max_control, max_control)
```

2. **Parameters:**
- Kp = 15.0, bounds = [0.1, 100.0]
- Kd = 2.5, bounds = [0.01, 10.0]

3. **Simulation:**
- Time step: 0.01s
- Steps: 1000 (10 seconds)
- Initial state: [0.0, 0.0]
- Reference: 1.0
- Plant: Double Integrator

4. **Expected Results:**
- Settling time: ~2-3 seconds
- Overshoot: ~10-20%
- Steady-state error: ~0

---

### Example 2: PID Controller with Anti-Windup

Add integral action for zero steady-state error.

**Load Configuration:**
```bash
# From the studio: File → Load Config → pid_controller_config.json
```

**Parameters:**
- Kp = 20.0
- Ki = 5.0
- Kd = 3.0

**Expected Improvement:**
- Faster settling time
- Zero steady-state error
- Slightly more overshoot (tune Ki to reduce)

---

### Example 3: Comparison Study

Compare PD vs PID performance:

1. Load PD config
2. Run simulation
3. Go to **Compare** tab
4. Click "Add Current"
5. Load PID config
6. Run simulation
7. Click "Add Current"
8. Click "Run Comparison"

**View:**
- Side-by-side response curves
- IAE comparison in table
- Choose best performer

---

### Example 4: Parameter Optimization

Find optimal gains automatically:

1. Load PD config
2. Set wider bounds:
   - Kp: [0.1, 50.0]
   - Kd: [0.01, 10.0]
3. Go to **Optimization** tab
4. Select "Python Differential Evolution"
5. Click "Optimize Parameters"
6. Wait 2-3 minutes
7. View optimized parameters

**Typical Optimized Results:**
```
Kp = 18.5
Kd = 2.8
IAE = 1.23
Settling time = 2.1s
Overshoot = 8.5%
```

---

### Example 5: Robustness Testing

Test controller with different plants:

**Same Controller, Different Plants:**

1. Define PD controller (Kp=15, Kd=2.5)
2. Test with Double Integrator
3. Record IAE
4. Change to Mass-Spring-Damper
5. Run simulation
6. Compare performance

**Questions to Answer:**
- Does the controller work for all plants?
- Which plant is hardest to control?
- Do we need different gains for different plants?

---

### Example 6: Grid Search

Manual parameter search:

**Test Multiple Kp Values:**

| Kp  | Kd  | IAE   | Settling Time | Overshoot |
|-----|-----|-------|---------------|-----------|
| 5   | 1.0 | 5.23  | 6.8s          | 2%        |
| 10  | 1.5 | 2.45  | 4.2s          | 8%        |
| 15  | 2.0 | 1.67  | 2.8s          | 15%       |
| 20  | 2.5 | 1.34  | 2.1s          | 22%       |
| 25  | 3.0 | 1.28  | 1.8s          | 28%       |

**Analysis:**
- Higher Kp → Faster response, more overshoot
- Need to balance speed vs overshoot
- Kp=15-20 seems optimal for this case

---

### Example 7: Advanced State Feedback

Implement full state feedback (LQR-style):

**Control Logic:**
```python
# State feedback: u = -K * x
# where x = [position, velocity]

# Define gain vector
K = np.array([K1, K2])

# Error state
error_state = reference - state[0]
state_with_error = np.array([error_state, -state[1]])

# Control law
control_output = K @ state_with_error

# Saturation
control_output = np.clip(control_output, -10.0, 10.0)
```

**Parameters:**
- K1 = 20.0 (position feedback gain)
- K2 = 4.0 (velocity feedback gain)

**Note:** This is equivalent to PD control but formulated differently.

---

### Example 8: Nonlinear Control

Add nonlinear terms:

**Control Logic:**
```python
# Nonlinear PD Controller

error = reference - state[0]
error_dot = -state[1]

# Nonlinear error function (sigmoid-like)
error_nl = np.tanh(error * 2.0)  # Saturates for large errors

# Control law
control_output = Kp * error_nl + Kd * error_dot

control_output = np.clip(control_output, -10.0, 10.0)
```

**Advantage:**
- Smoother control for large errors
- Less aggressive initially
- Can prevent overshoot

---

### Example 9: Model-Based Control

Use plant model in controller:

**Control Logic:**
```python
# Model-based feedforward + PD feedback

error = reference - state[0]
error_dot = -state[1]

# Feedback
control_fb = Kp * error + Kd * error_dot

# Feedforward (assuming double integrator)
# To track reference, need u = d²ref/dt²
# For step input, this is 0
control_ff = 0.0

# Total control
control_output = control_fb + control_ff

control_output = np.clip(control_output, -10.0, 10.0)
```

**When to Use:**
- Known plant dynamics
- Want faster tracking
- Can compute desired acceleration

---

### Example 10: Multi-Objective Optimization

Optimize for multiple goals:

**Optimization Setup:**
1. Define PD controller
2. Set parameter bounds
3. Modify cost function in code to include:
   - Minimize IAE (tracking error)
   - Minimize control effort
   - Minimize overshoot

**Custom Cost Function:**
```python
# In matlab_integration_enhanced.py, modify:
def objective(x):
    # ... run simulation ...
    iae = np.sum(np.abs(error)) * dt
    control_effort = np.sum(np.abs(control)) * dt
    overshoot = calculate_overshoot(response, reference)
    
    # Weighted sum
    w1, w2, w3 = 1.0, 0.1, 0.5
    cost = w1 * iae + w2 * control_effort + w3 * overshoot
    return cost
```

---

## Tips for Each Example

### PD Controller
- Start with Kp = 10-20 for position control
- Kd should be 10-20% of Kp
- Increase Kd if oscillations occur
- Increase Kp for faster response

### PID Controller
- Add integral action sparingly
- Ki should be much smaller than Kp (1-5)
- Always implement anti-windup
- May need to retune Kp and Kd

### State Feedback
- Equivalent to PD but different formulation
- Useful for multi-input-multi-output systems
- Can design using LQR theory
- Gains relate: Kp ≈ K1, Kd ≈ K2

### Nonlinear Control
- Use for large operating ranges
- Can improve performance vs linear
- Harder to analyze theoretically
- Test extensively before hardware

### Model-Based Control
- Requires accurate plant model
- Can achieve superior performance
- Robust to model errors with feedback
- Feedforward improves tracking

---

## Workflow Tips

1. **Always start simple** - PD before PID
2. **Visualize first** - Look at response before metrics
3. **One change at a time** - Don't change multiple parameters
4. **Save good configs** - Document what works
5. **Compare systematically** - Use comparison tab
6. **Optimize after manual** - Get close manually, then optimize
7. **Validate on hardware** - Simulation ≠ Reality
8. **Add safety** - Always saturate control output

---

## Common Issues and Fixes

### Issue: Oscillations
**Fix:** Increase Kd (damping)

### Issue: Slow response
**Fix:** Increase Kp (stiffness)

### Issue: Steady-state error
**Fix:** Add Ki (integral action)

### Issue: Large overshoot
**Fix:** Decrease Kp or increase Kd

### Issue: Chattering
**Fix:** Add saturation, reduce gains

### Issue: Unstable
**Fix:** Reduce all gains, check plant model

---

## Next Steps

After mastering these examples:

1. **Try with real hardware**
   ```bash
   arm-bench create-control position --output my_controller.py
   # Edit with your tuned parameters
   arm-bench deploy-control my_controller.py
   arm-bench launch-control position
   ```

2. **Experiment with different plants**
   - Add your own plant models
   - Test robustness

3. **Implement advanced controllers**
   - Sliding mode control
   - Adaptive control
   - Neural network control

4. **Share your configurations**
   - Save and document successful configs
   - Build a library of controllers

---

**Happy Experimenting!** 🎮
