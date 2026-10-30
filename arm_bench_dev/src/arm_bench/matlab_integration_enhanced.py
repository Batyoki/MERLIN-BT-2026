"""Enhanced MATLAB integration for comprehensive control algorithm development.

This module provides:
- Custom control logic definition
- Parameter tuning with MATLAB optimization
- Multi-parameter comparison
- Performance visualization
- Automated testing and evaluation
"""

import importlib
import os
import sys
import subprocess
import json
from typing import Dict, List, Optional, Tuple, Callable, Any
import numpy as np
from pathlib import Path


def _prepare_qt_environment():
    """Prepare Qt environment for correct plugin loading."""
    for env_var in ('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH'):
        os.environ.pop(env_var, None)
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

    try:
        from PyQt5.QtCore import QLibraryInfo
        plugin_path = QLibraryInfo.location(QLibraryInfo.PluginsPath)
        if plugin_path:
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path
    except Exception:
        pass


class ControlAlgorithm:
    """Represents a custom control algorithm with parameters."""
    
    def __init__(self, name: str = "Custom Controller"):
        self.name = name
        self.parameters = {}
        self.parameter_bounds = {}
        self.control_code = ""
        self.results_history = []
        
    def define_parameters(self, params: Dict[str, float], 
                         bounds: Optional[Dict[str, Tuple[float, float]]] = None):
        """Define control parameters and their bounds for tuning."""
        self.parameters = params.copy()
        if bounds:
            self.parameter_bounds = bounds.copy()
        else:
            # Auto-generate reasonable bounds (±90%)
            for name, value in params.items():
                if isinstance(value, (int, float)) and value != 0:
                    self.parameter_bounds[name] = (value * 0.1, value * 10)
                else:
                    self.parameter_bounds[name] = (-10.0, 10.0)
    
    def set_control_logic(self, code: str):
        """Set custom control logic code."""
        self.control_code = code
    
    def evaluate(self, state: np.ndarray, reference: float, dt: float = 0.01) -> float:
        """Evaluate control output."""
        local_vars = {
            'state': state,
            'reference': reference,
            'dt': dt,
            'np': np,
            **self.parameters
        }
        
        try:
            exec(self.control_code, local_vars)
            return float(local_vars.get('control_output', 0.0))
        except Exception as e:
            print(f"Control evaluation error: {e}")
            return 0.0
    
    def simulate(self, initial_state: np.ndarray, reference: float,
                 time_steps: int, dt: float, 
                 plant_model: Callable) -> Dict[str, np.ndarray]:
        """Run full simulation."""
        state = initial_state.copy()
        states = [state.copy()]
        controls = []
        errors = []
        times = [0.0]
        
        for i in range(time_steps):
            error = reference - state[0]
            control = self.evaluate(state, reference, dt)
            
            # Apply control to plant
            state = plant_model(state, control, dt)
            
            states.append(state.copy())
            controls.append(control)
            errors.append(error)
            times.append(times[-1] + dt)
        
        return {
            'time': np.array(times),
            'state': np.array(states),
            'control': np.array(controls),
            'error': np.array(errors),
            'reference': reference
        }


class MATLABOptimizer:
    """MATLAB-based parameter optimization."""
    
    def __init__(self):
        self.matlab_dir = Path.home() / "arm_bench_matlab"
        self.matlab_dir.mkdir(exist_ok=True)
        self.matlab_available = self._check_matlab_available()
        self.engine = None
        
    def _check_matlab_available(self) -> bool:
        """Check if MATLAB Engine API is importable."""
        try:
            self._ensure_matlab_imported()
            return True
        except ImportError:
            return False

    def _ensure_matlab_imported(self):
        """Ensure matlab.engine is importable, with fallback to local virtualenv site-packages."""
        try:
            matlab_pkg = importlib.import_module('matlab')
            importlib.import_module('matlab.engine')
            return matlab_pkg
        except ImportError as first_error:
            repo_root = Path(__file__).resolve().parents[2]
            candidates = [
                repo_root / 'arm-test' / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}' / 'site-packages',
                repo_root / 'arm-test' / 'lib64' / f'python{sys.version_info.major}.{sys.version_info.minor}' / 'site-packages',
            ]
            for candidate in candidates:
                if candidate.is_dir() and str(candidate) not in sys.path:
                    sys.path.insert(0, str(candidate))
            try:
                matlab_pkg = importlib.import_module('matlab')
                importlib.import_module('matlab.engine')
                return matlab_pkg
            except ImportError as second_error:
                raise ImportError(
                    "MATLAB Engine API not installed or not found in arm-test virtualenv. "
                    f"First error: {first_error}; fallback error: {second_error}"
                )

    def start_engine(self) -> bool:
        """Start MATLAB engine."""
        if self.engine is not None:
            return True

        if not self.matlab_available:
            print("⚠️  MATLAB not found. Using Python-based optimization.")
            return False

        try:
            matlab = self._ensure_matlab_imported()
            print("Starting MATLAB engine...")
            self.engine = matlab.engine.start_matlab()
            self.engine.addpath(str(self.matlab_dir), nargout=0)
            print("✓ MATLAB engine ready")
            return True
        except ImportError as e:
            print("⚠️  MATLAB Engine API not installed: pip install matlabengine")
            print(f"ImportError: {e}")
            return False
        except Exception as e:
            print(f"⚠️  MATLAB engine error: {e}")
            return False
    
    def optimize_parameters(self, algorithm: ControlAlgorithm,
                           simulation_config: Dict,
                           method: str = 'fmincon') -> Dict:
        """Optimize algorithm parameters using MATLAB."""
        if method == 'auto':
            method = 'fmincon' if self.matlab_available else 'python'

        if method == 'fmincon':
            if self.engine is None and not self.start_engine():
                print("⚠️  MATLAB engine unavailable; falling back to Python optimization.")
                method = 'python'

        if self.engine and method != 'python':
            return self._matlab_optimize(algorithm, simulation_config, method)

        return self._python_optimize(algorithm, simulation_config)
    
    def _matlab_optimize(self, algorithm: ControlAlgorithm,
                        config: Dict, method: str) -> Dict:
        """MATLAB-based optimization."""
        # Save configuration
        self._save_config(algorithm, config)
        
        # Create MATLAB optimization script
        script = self._create_matlab_script(algorithm, config, method)
        script_path = self.matlab_dir / "optimize_controller.m"
        script_path.write_text(script)
        
        try:
            # Run optimization
            result = self.engine.optimize_controller(nargout=1)
            
            # Extract results
            optimized = {
                'parameters': {},
                'cost': float(result.get('cost', 0)),
                'metrics': {}
            }
            
            for param in algorithm.parameters.keys():
                if param in result:
                    optimized['parameters'][param] = float(result[param])
            
            # Performance metrics
            for metric in ['settling_time', 'overshoot', 'iae', 'ise', 'steady_state_error']:
                if metric in result:
                    optimized['metrics'][metric] = float(result[metric])
            
            return optimized
            
        except Exception as e:
            print(f"MATLAB optimization error: {e}")
            return self._python_optimize(algorithm, config)
    
    def _python_optimize(self, algorithm: ControlAlgorithm,
                        config: Dict) -> Dict:
        """Python-based optimization fallback."""
        from scipy.optimize import differential_evolution
        
        param_names = list(algorithm.parameters.keys())
        bounds = [algorithm.parameter_bounds[p] for p in param_names]
        
        def objective(x):
            # Update parameters
            for i, name in enumerate(param_names):
                algorithm.parameters[name] = x[i]
            
            # Run simulation
            result = algorithm.simulate(
                config['initial_state'],
                config['reference'],
                config['time_steps'],
                config['dt'],
                config['plant_model']
            )
            
            # Compute cost
            error = result['error']
            cost = np.sum(error**2) * config['dt']  # ISE
            return cost
        
        print("Running Python-based optimization...")
        result = differential_evolution(objective, bounds, maxiter=100, disp=True)
        
        optimized = {
            'parameters': {name: val for name, val in zip(param_names, result.x)},
            'cost': result.fun,
            'metrics': {}
        }
        
        # Update algorithm with optimized parameters
        algorithm.parameters.update(optimized['parameters'])
        
        # Run final simulation for metrics
        sim_result = algorithm.simulate(
            config['initial_state'],
            config['reference'],
            config['time_steps'],
            config['dt'],
            config['plant_model']
        )
        
        optimized['metrics'] = self._compute_metrics(sim_result, config)
        
        return optimized
    
    def _compute_metrics(self, sim_result: Dict, config: Dict) -> Dict:
        """Compute performance metrics."""
        error = sim_result['error']
        time = sim_result['time'][1:]
        dt = config['dt']
        
        metrics = {
            'iae': float(np.sum(np.abs(error)) * dt),
            'ise': float(np.sum(error**2) * dt),
            'itae': float(np.sum(np.abs(error) * time) * dt),
            'max_error': float(np.max(np.abs(error))),
            'steady_state_error': float(np.abs(error[-1]))
        }
        
        # Settling time (2% criterion)
        threshold = 0.02 * config['reference']
        settling_idx = np.where(np.abs(error) > threshold)[0]
        if len(settling_idx) > 0:
            metrics['settling_time'] = float(settling_idx[-1] * dt)
        else:
            metrics['settling_time'] = 0.0
        
        # Overshoot
        response = sim_result['state'][:, 0]
        max_val = np.max(response)
        if max_val > config['reference']:
            metrics['overshoot'] = float(((max_val - config['reference']) / config['reference']) * 100)
        else:
            metrics['overshoot'] = 0.0
        
        return metrics
    
    def _save_config(self, algorithm: ControlAlgorithm, config: Dict):
        """Save configuration to JSON."""
        save_config = {
            'name': algorithm.name,
            'parameters': algorithm.parameters,
            'bounds': algorithm.parameter_bounds,
            'control_code': algorithm.control_code,
            'simulation': {
                'dt': config['dt'],
                'time_steps': config['time_steps'],
                'initial_state': config['initial_state'].tolist(),
                'reference': float(config['reference'])
            }
        }
        
        config_path = self.matlab_dir / "controller_config.json"
        with open(config_path, 'w') as f:
            json.dump(save_config, f, indent=2)
    
    def _create_matlab_script(self, algorithm: ControlAlgorithm,
                             config: Dict, method: str) -> str:
        """Generate MATLAB optimization script."""
        param_names = list(algorithm.parameters.keys())
        x0 = [algorithm.parameters[p] for p in param_names]
        lb = [algorithm.parameter_bounds[p][0] for p in param_names]
        ub = [algorithm.parameter_bounds[p][1] for p in param_names]
        
        return f"""function result = optimize_controller()
    % Load configuration
    config = jsondecode(fileread('controller_config.json'));
    
    % Initial values and bounds
    x0 = {x0};
    lb = {lb};
    ub = {ub};
    
    % Optimization options
    options = optimoptions('{method}', 'Display', 'iter', ...
        'MaxIterations', 200, 'FunctionTolerance', 1e-6);
    
    % Optimize
    [x_opt, fval] = {method}(@(x) cost_function(x, config), x0, ...
        [], [], [], [], lb, ub, [], options);
    
    % Package results
    result = struct();
    result.cost = fval;
{chr(10).join(f"    result.{name} = x_opt({i+1});" for i, name in enumerate(param_names))}
    
    % Final simulation metrics
    [metrics] = simulate_controller(x_opt, config);
    result.iae = metrics.iae;
    result.ise = metrics.ise;
    result.settling_time = metrics.settling_time;
    result.overshoot = metrics.overshoot;
    result.steady_state_error = metrics.sse;
end

function cost = cost_function(params, config)
    metrics = simulate_controller(params, config);
    % Multi-objective: weighted sum
    cost = 1.0 * metrics.ise + 0.5 * abs(metrics.overshoot) + ...
           0.3 * metrics.settling_time;
end

function metrics = simulate_controller(params, config)
    dt = config.simulation.dt;
    steps = config.simulation.time_steps;
    state = config.simulation.initial_state;
    ref = config.simulation.reference;
    
    errors = zeros(steps, 1);
    
    for i = 1:steps
        error = ref - state(1);
        errors(i) = error;
        
        % Control law (customize based on your logic)
        % Example: PD control
        if length(params) >= 2
            control = params(1) * error + params(2) * (-state(2));
        else
            control = params(1) * error;
        end
        
        % Plant model (double integrator example)
        state_dot = [state(2); control];
        state = state + state_dot * dt;
    end
    
    % Compute metrics
    metrics = struct();
    metrics.iae = sum(abs(errors)) * dt;
    metrics.ise = sum(errors.^2) * dt;
    
    % Settling time
    threshold = 0.02 * ref;
    settling_idx = find(abs(errors) > threshold, 1, 'last');
    if isempty(settling_idx)
        metrics.settling_time = 0;
    else
        metrics.settling_time = settling_idx * dt;
    end
    
    % Overshoot (simplified)
    metrics.overshoot = 0;
    metrics.sse = abs(errors(end));
end
"""


# Convenience functions

def launch_control_development():
    """Launch the enhanced control development GUI."""
    try:
        _prepare_qt_environment()
        from control_simulator.control_simulator_gui import ControlSimulatorGUI
        from PyQt5.QtWidgets import QApplication

        app = QApplication.instance()
        if app is None:
            app = QApplication(sys.argv)
        app.setStyle('Fusion')

        window = ControlSimulatorGUI()
        window.show()
        app.exec_()
    except ImportError as e:
        print(f"GUI dependencies not available: {e}")
        print("Install with: pip install PyQt5 matplotlib")


def launch_matlab_tuning():
    """Backward compatibility - launches enhanced GUI."""
    launch_control_development()


if __name__ == '__main__':
    launch_control_development()


def compare_parameter_sets(algorithm: ControlAlgorithm,
                          parameter_sets: List[Dict[str, float]],
                          simulation_config: Dict) -> List[Dict]:
    """Compare multiple parameter configurations."""
    optimizer = MATLABOptimizer()
    results = []
    
    for i, params in enumerate(parameter_sets):
        print(f"\n=== Testing Parameter Set {i+1}/{len(parameter_sets)} ===")
        
        # Update parameters
        algorithm.parameters.update(params)
        
        # Run simulation
        sim_result = algorithm.simulate(
            simulation_config['initial_state'],
            simulation_config['reference'],
            simulation_config['time_steps'],
            simulation_config['dt'],
            simulation_config['plant_model']
        )
        
        # Compute metrics
        metrics = optimizer._compute_metrics(sim_result, simulation_config)
        metrics['parameter_set'] = i + 1
        metrics['parameters'] = params.copy()
        
        results.append({
            'simulation': sim_result,
            'metrics': metrics
        })
        
        print(f"IAE: {metrics['iae']:.4f}, Settling Time: {metrics['settling_time']:.2f}s")
    
    return results


def launch_matlab_tuning():
    """Backward compatibility - launches enhanced GUI."""
    launch_control_development()
