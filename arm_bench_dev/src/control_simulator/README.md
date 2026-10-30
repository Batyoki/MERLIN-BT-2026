# Control Algorithm Simulator & Parameter Tuner

A comprehensive control system simulation and parameter tuning tool with MATLAB engine integration and a modern PyQt5 GUI.

## Features

### 🎯 Core Capabilities
- **MATLAB Engine Integration**: Execute control algorithms in MATLAB from Python
- **Multiple Control Algorithms**: PID, LQR, and MPC controllers included
- **Automatic Parameter Tuning**: Optimize controller parameters using:
  - Differential Evolution
  - Nelder-Mead
  - Powell Method
- **Real-time Visualization**: Live plotting of system response and control effort
- **Performance Metrics**: Comprehensive analysis including:
  - Rise time, settling time, overshoot
  - ISE, IAE, ITAE objectives
  - Control effort and steady-state error
- **Comparison Tools**: Side-by-side comparison of different parameter sets
- **Multithreaded Operation**: Non-blocking UI during simulations and tuning

### Parameters from the script (`@param`)
A control law can declare its tunable parameters, bounds and descriptions itself; the GUI, the tuner and the MATLAB engine read
that declaration:

```matlab
% @param min=0 max=50 desc="proportional gain"
Kp = 8.0;
u = Kp*e + Ki*integral + Kd*derivative;
```

See `examples/` for scripts, `param_parser.py` for the rules (also in `docs/CONTROL_SIMULATOR_WALKTHROUGH.md` at the repository
root, which has a complete worked session) and `python_sim.py` for a MATLAB-free dry run used by the tests.

### 📊 Included Control Algorithms

#### 1. **PID Controller** (`pid_controller.m`)
Classic Proportional-Integral-Derivative controller with anti-windup
- Parameters: `Kp`, `Ki`, `Kd`
- Best for: Simple systems, fast response

#### 2. **LQR Controller** (`lqr_controller.m`)
Linear Quadratic Regulator with optimal state feedback
- Parameters: `Q_pos`, `Q_vel`, `R`
- Best for: State-space models, optimal control

#### 3. **MPC Controller** (`mpc_controller.m`)
Model Predictive Control with prediction horizon
- Parameters: `Horizon`, `Q_weight`, `R_weight`
- Best for: Constrained systems, complex objectives

## Installation

### Prerequisites
1. **MATLAB** (R2021a or later) with MATLAB Engine API for Python
2. **Python** 3.8 or later

### Install MATLAB Engine for Python
```bash
# Navigate to MATLAB engine folder (adjust path for your MATLAB version)
cd /usr/local/MATLAB/R2023b/extern/engines/python

# Install for your Python environment
python setup.py install
```

### Install Python Dependencies
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
pip install -r requirements.txt
```

## Quick Start

### Running the GUI
```bash
cd /home/yash_sai/Yash/Arm/control_simulator
python control_simulator_gui.py
```

The MATLAB engine will start automatically (takes ~10-15 seconds).

### Basic Workflow

1. **Select Algorithm**: Choose PID, LQR, or MPC from dropdown
2. **Set Parameters**: Adjust controller parameters or use defaults
3. **Configure Simulation**:
   - Simulation time (default: 10s)
   - Reference signal (step, ramp, sine, square)
   - Reference amplitude
4. **Run Simulation**: Click "Run Simulation"
5. **View Results**: Check time response, metrics, and comparison tabs

### Auto-Tuning Workflow

1. **Set Parameter Bounds**: Define min/max values for each parameter
2. **Select Objective**: Choose ISE, IAE, ITAE, or custom
3. **Select Method**: Choose optimization algorithm
4. **Set Max Iterations**: Balance speed vs accuracy (default: 50)
5. **Click "Auto-Tune Parameters"**
6. **Wait for Completion**: Progress shown in log and progress bar
7. **Review Results**: Optimal parameters loaded automatically
8. **Run Simulation**: Test optimal parameters

## GUI Components

### Left Panel: Control & Configuration
- **Algorithm Selection**: Choose control algorithm
- **Controller Parameters**: Adjust gains/weights with min/max bounds
- **Simulation Settings**: Time, reference signal, amplitude
- **Parameter Tuning**: Objective, method, iterations
- **Action Buttons**: Run, tune, stop, save, load
- **Progress Bar**: Shows operation progress
- **Log**: Real-time status and results

### Right Panel: Visualization & Results
- **Time Response Tab**: System output vs reference, control effort
- **Performance Metrics Tab**: Detailed metrics table
- **Comparison Tab**: Overlay multiple simulation runs
- **Tuning History Tab**: Convergence plot during optimization

## Performance Metrics Explained

| Metric | Description | Goal |
|--------|-------------|------|
| `rise_time` | Time from 10% to 90% of final value | Minimize |
| `settling_time` | Time to settle within 2% band | Minimize |
| `overshoot_percent` | Peak overshoot above steady-state | Minimize |
| `steady_state_error` | Final tracking error | Minimize |
| `rms_error` | Root mean square error | Minimize |
| `peak_control` | Maximum control input magnitude | Monitor |
| `control_effort` | Integral of squared control | Minimize |
| `iae` | Integral Absolute Error | Minimize |
| `ise` | Integral Squared Error | Minimize |
| `itae` | Integral Time Absolute Error | Minimize |

## Tuning Objectives

- **ISE** (Integral Squared Error): Penalizes large errors heavily, fast response
- **IAE** (Integral Absolute Error): Balanced, smooth response
- **ITAE** (Integral Time Absolute Error): Penalizes persistent errors, best steady-state
- **Custom**: Weighted combination of ISE, overshoot, settling time, control effort

## Optimization Methods

- **Differential Evolution**: Global optimizer, best for complex landscapes (slower)
- **Nelder-Mead**: Simplex method, good for local optimization (faster)
- **Powell**: Conjugate direction method, efficient for smooth objectives

## File Structure

```
control_simulator/
├── control_simulator_gui.py       # Main GUI application
├── matlab_engine_interface.py     # MATLAB integration layer
├── parameter_tuner.py             # Optimization algorithms
├── requirements.txt               # Python dependencies
├── README.md                      # This file
├── matlab_scripts/
│   ├── run_control_sim.m         # Main simulation function
│   ├── pid_controller.m          # PID implementation
│   ├── lqr_controller.m          # LQR implementation
│   └── mpc_controller.m          # MPC implementation
├── results/                       # Saved simulation results
└── configs/                       # Saved configurations
```

## Saving and Loading

### Save Results
- Click "Save Results" to export simulation and tuning history as JSON
- Includes: parameters, metrics, timestamps, full time-series data

### Load Configuration
- Click "Load Configuration" to import previous results
- Comparison tab updates automatically with loaded data

## Extending the Simulator

### Adding a New Control Algorithm

1. **Create MATLAB Controller** (`matlab_scripts/mycontroller_controller.m`):
```matlab
function [u, state] = mycontroller_controller(error, dt, params, prev_state)
    % Your controller logic here
    % params: array of tunable parameters
    % Return control input u and updated state
end
```

2. **Add to run_control_sim.m**:
```matlab
case 'mycontroller'
    [u, controller_state] = mycontroller_controller(e, dt, params, controller_state);
```

3. **Update GUI** (`control_simulator_gui.py`):
```python
# In on_algorithm_changed method, add:
elif algorithm == 'MyController':
    params = [
        ('Param1', default, min, max),
        ('Param2', default, min, max),
    ]
```

### Customizing the System Model

Edit `run_control_sim.m` to change the plant dynamics:
```matlab
% Example: DC Motor
J = 0.01;  % inertia
b = 0.1;   % viscous friction
K = 0.01;  % motor constant
A = [0, 1; 0, -b/J];
B = [0; K/J];
C = [1, 0];
```

## Troubleshooting

### MATLAB Engine Won't Start
- Verify MATLAB installed: `which matlab`
- Check Python engine installed: `python -c "import matlab.engine"`
- Ensure MATLAB license valid

### Simulation Fails
- Check MATLAB script syntax in log
- Verify parameter ranges are reasonable
- Try default parameters first

### Tuning Takes Too Long
- Reduce max iterations
- Use Powell or Nelder-Mead instead of Differential Evolution
- Reduce simulation time

### GUI Freezes
- Check if simulation/tuning worker threads are running
- Click "Stop" to terminate operation
- Restart application if needed

## Example Use Cases

### 1. Tune PID for Minimum Overshoot
1. Select PID algorithm
2. Set bounds: Kp (0-50), Ki (0-10), Kd (0-5)
3. Objective: Custom (weights overshoot heavily)
4. Method: Differential Evolution
5. Run tuning

### 2. Compare Controllers
1. Run simulation with PID
2. Switch to LQR, run simulation
3. Switch to MPC, run simulation
4. View Comparison tab for overlay

### 3. Grid Search
```python
# From Python console
from parameter_tuner import ParameterTuner
tuner = ParameterTuner(simulator)
results = tuner.grid_search(
    algorithm_name='pid',
    parameter_grids={
        'Kp': [0.5, 1.0, 2.0, 5.0],
        'Ki': [0.01, 0.1, 0.5],
        'Kd': [0.01, 0.05, 0.1]
    }
)
```

## Performance Tips

- Use step reference for fastest tuning
- Start with wide parameter bounds, refine iteratively
- ISE objective converges fastest
- Differential evolution for first pass, Nelder-Mead for refinement
- Save frequently during long tuning runs

## Known Limitations

- Single-input single-output (SISO) systems only (can be extended to MIMO)
- No real-time hardware-in-the-loop (HIL) support
- MATLAB engine startup time (~10-15s)
- Optimization can be time-consuming for complex objectives

## Future Enhancements

- [ ] Export plots as high-res images
- [ ] Real-time parameter adjustment with live update
- [ ] Disturbance rejection analysis
- [ ] Frequency domain analysis (Bode, Nyquist)
- [ ] Robustness analysis (gain/phase margins)
- [ ] Multi-objective optimization (Pareto fronts)
- [ ] Hardware-in-the-loop (HIL) integration
- [ ] Custom plant model GUI editor

## License

MIT License - See LICENSE file for details

## Support

For issues or questions:
1. Check this README
2. Review MATLAB script comments
3. Check Python module docstrings
4. Open an issue on GitHub

## Citation

If you use this tool in research, please cite:
```
@software{control_simulator,
  title={Control Algorithm Simulator and Parameter Tuner},
  author={Your Name},
  year={2025},
  url={https://github.com/yourusername/control_simulator}
}
```

---

**Happy Tuning! 🎛️🚀**
