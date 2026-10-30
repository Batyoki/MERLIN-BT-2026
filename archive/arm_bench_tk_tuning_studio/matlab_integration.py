"""
MATLAB integration module for arm-bench
Provides automated parameter tuning for control algorithms
"""
import importlib
import os
import subprocess
import sys
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np


class MATLABEngine:
    """Interface to MATLAB engine for parameter tuning"""
    
    def __init__(self):
        self.matlab_available = self._check_matlab_available()
        self.engine = None
        self.tuning_results = {}
        
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
        """Start MATLAB engine"""
        if not self.matlab_available:
            print("MATLAB is not available")
            return False
        
        try:
            import matlab.engine
            print("Starting MATLAB engine...")
            self.engine = matlab.engine.start_matlab()
            print("MATLAB engine started successfully")
            return True
        except ImportError:
            print("MATLAB Engine API for Python not installed")
            print("Install with: pip install matlabengine")
            return False
        except Exception as e:
            print(f"Error starting MATLAB engine: {e}")
            return False
    
    def stop_engine(self):
        """Stop MATLAB engine"""
        if self.engine:
            self.engine.quit()
            self.engine = None
            print("MATLAB engine stopped")
    
    def tune_pid_parameters(
        self,
        plant_model: Dict,
        performance_criteria: Dict
    ) -> Dict:
        """
        Automatically tune PID parameters using MATLAB
        
        Args:
            plant_model: Dictionary describing the plant (transfer function, etc.)
            performance_criteria: Desired performance (rise time, overshoot, etc.)
            
        Returns:
            Dictionary with tuned PID parameters
        """
        if not self.engine:
            if not self.start_engine():
                return {}
        
        try:
            # Create MATLAB script for PID tuning
            script_path = self._create_pid_tuning_script(plant_model, performance_criteria)
            
            # Run tuning in MATLAB
            print("Running PID tuning in MATLAB...")
            self.engine.cd(os.path.dirname(script_path), nargout=0)
            result = self.engine.eval(f"run('{os.path.basename(script_path)}')", nargout=0)
            
            # Read results
            results_file = script_path.replace('.m', '_results.json')
            if os.path.exists(results_file):
                with open(results_file, 'r') as f:
                    self.tuning_results = json.load(f)
                
                print("PID tuning completed!")
                print(f"Kp: {self.tuning_results.get('Kp')}")
                print(f"Ki: {self.tuning_results.get('Ki')}")
                print(f"Kd: {self.tuning_results.get('Kd')}")
                
                return self.tuning_results
            
        except Exception as e:
            print(f"Error during PID tuning: {e}")
        
        return {}
    
    def _create_pid_tuning_script(
        self,
        plant_model: Dict,
        performance_criteria: Dict
    ) -> str:
        """Create MATLAB script for PID tuning"""
        
        output_dir = os.path.expanduser("~/arm_bench_matlab")
        os.makedirs(output_dir, exist_ok=True)
        
        script_path = os.path.join(output_dir, "pid_tuning.m")
        
        # Extract parameters
        numerator = plant_model.get('numerator', [1])
        denominator = plant_model.get('denominator', [1, 1])
        rise_time = performance_criteria.get('rise_time', 0.5)
        overshoot = performance_criteria.get('overshoot', 10)
        
        matlab_code = f"""% Automated PID Tuning Script
% Generated by arm-bench

%% Define Plant Model
num = {self._list_to_matlab(numerator)};
den = {self._list_to_matlab(denominator)};
plant = tf(num, den);

disp('Plant Transfer Function:');
disp(plant);

%% Design PID Controller
% Using pidtune for automatic tuning
C = pidtune(plant, 'PID');

disp('Tuned PID Controller:');
disp(C);

% Extract PID parameters
Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

%% Evaluate Performance
% Closed-loop system
sys_cl = feedback(C * plant, 1);

% Step response
figure;
step(sys_cl);
title('Closed-Loop Step Response');
grid on;

% Get step info
info = stepinfo(sys_cl);
disp('Performance Metrics:');
disp(info);

%% Save Results
results = struct();
results.Kp = Kp;
results.Ki = Ki;
results.Kd = Kd;
results.RiseTime = info.RiseTime;
results.Overshoot = info.Overshoot;
results.SettlingTime = info.SettlingTime;
results.SteadyStateError = info.SettlingMin;

% Save to JSON
jsonStr = jsonencode(results);
fid = fopen('pid_tuning_results.json', 'w');
fprintf(fid, '%s', jsonStr);
fclose(fid);

disp('Results saved to pid_tuning_results.json');

%% Optimization (Optional)
% If performance criteria not met, use optimization
target_rise_time = {rise_time};
target_overshoot = {overshoot};

if info.RiseTime > target_rise_time || info.Overshoot > target_overshoot
    disp('Optimizing parameters...');
    
    % Define optimization problem
    options = optimoptions('fmincon', 'Display', 'iter');
    
    % Initial guess
    x0 = [Kp; Ki; Kd];
    
    % Bounds
    lb = [0; 0; 0];
    ub = [100; 100; 100];
    
    % Objective function (minimize rise time and overshoot)
    objective = @(x) pid_objective(x, plant, target_rise_time, target_overshoot);
    
    % Optimize
    x_opt = fmincon(objective, x0, [], [], [], [], lb, ub, [], options);
    
    % Update results
    results.Kp = x_opt(1);
    results.Ki = x_opt(2);
    results.Kd = x_opt(3);
    
    % Save optimized results
    jsonStr = jsonencode(results);
    fid = fopen('pid_tuning_results.json', 'w');
    fprintf(fid, '%s', jsonStr);
    fclose(fid);
    
    disp('Optimized results saved');
end

%% Helper function for optimization
function cost = pid_objective(params, plant, target_rt, target_os)
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    
    C = pid(Kp, Ki, Kd);
    sys_cl = feedback(C * plant, 1);
    
    info = stepinfo(sys_cl);
    
    % Cost = weighted sum of deviations
    cost = abs(info.RiseTime - target_rt) + 0.01 * abs(info.Overshoot - target_os);
end
"""
        
        with open(script_path, 'w') as f:
            f.write(matlab_code)
        
        print(f"Created MATLAB tuning script: {script_path}")
        return script_path
    
    def _list_to_matlab(self, lst: List) -> str:
        """Convert Python list to MATLAB array format"""
        return '[' + ' '.join(str(x) for x in lst) + ']'
    
    def optimize_control_gains(
        self,
        robot_params: Dict,
        control_mode: str,
        optimization_objective: str = "minimize_error"
    ) -> Dict:
        """
        Optimize control gains for a robot arm
        
        Args:
            robot_params: Robot parameters (mass, inertia, etc.)
            control_mode: position, velocity, or torque
            optimization_objective: Optimization goal
            
        Returns:
            Optimized control gains
        """
        if not self.engine:
            if not self.start_engine():
                return {}
        
        print(f"Optimizing {control_mode} control gains...")
        
        # Create optimization script
        script_path = self._create_optimization_script(
            robot_params,
            control_mode,
            optimization_objective
        )
        
        try:
            # Run optimization
            self.engine.cd(os.path.dirname(script_path), nargout=0)
            self.engine.eval(f"run('{os.path.basename(script_path)}')", nargout=0)
            
            # Read results
            results_file = script_path.replace('.m', '_results.json')
            if os.path.exists(results_file):
                with open(results_file, 'r') as f:
                    results = json.load(f)
                
                print("Optimization completed!")
                return results
        
        except Exception as e:
            print(f"Error during optimization: {e}")
        
        return {}
    
    def _create_optimization_script(
        self,
        robot_params: Dict,
        control_mode: str,
        objective: str
    ) -> str:
        """Create MATLAB script for control gain optimization"""
        
        output_dir = os.path.expanduser("~/arm_bench_matlab")
        os.makedirs(output_dir, exist_ok=True)
        
        script_path = os.path.join(output_dir, f"{control_mode}_optimization.m")
        
        matlab_code = f"""% Control Gain Optimization
% Mode: {control_mode}
% Objective: {objective}

%% Robot Parameters
% Add your robot-specific parameters here
mass = {robot_params.get('mass', 1.0)};
length = {robot_params.get('length', 0.5)};
inertia = {robot_params.get('inertia', 0.1)};

%% Optimization
disp('Starting optimization...');

% Define optimization variables
if strcmp('{control_mode}', 'position')
    % Position control: optimize Kp, Kd
    x0 = [10, 1];  % Initial guess [Kp, Kd]
    lb = [0.1, 0.01];
    ub = [100, 10];
    
elseif strcmp('{control_mode}', 'velocity')
    % Velocity control: optimize Kp, Ki
    x0 = [5, 0.5];  % Initial guess [Kp, Ki]
    lb = [0.1, 0.01];
    ub = [50, 5];
    
else  % torque
    % Torque control: optimize Kp, Ki, Kd
    x0 = [10, 1, 0.5];  % Initial guess [Kp, Ki, Kd]
    lb = [0.1, 0.01, 0.01];
    ub = [100, 10, 5];
end

% Optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Run optimization
optimal_gains = fmincon(@cost_function, x0, [], [], [], [], lb, ub, [], options);

%% Save Results
results = struct();
if strcmp('{control_mode}', 'position')
    results.Kp = optimal_gains(1);
    results.Kd = optimal_gains(2);
elseif strcmp('{control_mode}', 'velocity')
    results.Kp = optimal_gains(1);
    results.Ki = optimal_gains(2);
else
    results.Kp = optimal_gains(1);
    results.Ki = optimal_gains(2);
    results.Kd = optimal_gains(3);
end

% Save to JSON
jsonStr = jsonencode(results);
fid = fopen('{control_mode}_optimization_results.json', 'w');
fprintf(fid, '%s', jsonStr);
fclose(fid);

disp('Optimization complete!');
disp(results);

%% Cost Function
function cost = cost_function(gains)
    % Simulate system and compute cost
    % This is a simplified example
    
    % Simulation parameters
    dt = 0.01;
    t_end = 5.0;
    t = 0:dt:t_end;
    
    % Desired trajectory (step input)
    desired = ones(size(t));
    
    % Simulate response
    actual = zeros(size(t));
    error = zeros(size(t));
    
    for i = 2:length(t)
        error(i) = desired(i) - actual(i-1);
        
        % Simple control law
        if length(gains) == 2
            u = gains(1) * error(i) + gains(2) * (error(i) - error(i-1))/dt;
        else
            u = gains(1) * error(i) + gains(2) * sum(error(1:i)*dt) + gains(3) * (error(i) - error(i-1))/dt;
        end
        
        % Simple dynamics (second-order system)
        actual(i) = actual(i-1) + u * dt;
    end
    
    % Cost = tracking error + control effort penalty
    cost = sum(error.^2) + 0.01 * sum(abs(diff(actual)));
end
"""
        
        with open(script_path, 'w') as f:
            f.write(matlab_code)
        
        return script_path


class ParameterTuningGUI:
    """GUI for MATLAB parameter tuning"""
    
    def __init__(self):
        self.engine = MATLABEngine()
        
    def launch_tuning_interface(self):
        """Launch parameter tuning GUI"""
        import tkinter as tk
        from tkinter import ttk, messagebox
        
        root = tk.Tk()
        root.title("MATLAB Parameter Tuning")
        root.geometry("600x700")
        
        # Title
        tk.Label(
            root,
            text="🔧 MATLAB Parameter Tuning",
            font=("Segoe UI", 16, "bold")
        ).pack(pady=20)
        
        # Control Mode Selection
        mode_frame = tk.LabelFrame(root, text="Control Mode", padx=20, pady=10)
        mode_frame.pack(fill='x', padx=20, pady=10)
        
        mode_var = tk.StringVar(value="position")
        for mode in ["position", "velocity", "torque"]:
            tk.Radiobutton(
                mode_frame,
                text=mode.capitalize(),
                variable=mode_var,
                value=mode
            ).pack(anchor='w')
        
        # Plant Parameters
        plant_frame = tk.LabelFrame(root, text="Plant Parameters", padx=20, pady=10)
        plant_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(plant_frame, text="Numerator (comma-separated):").pack(anchor='w')
        num_entry = tk.Entry(plant_frame, width=40)
        num_entry.insert(0, "1")
        num_entry.pack(pady=5)
        
        tk.Label(plant_frame, text="Denominator (comma-separated):").pack(anchor='w')
        den_entry = tk.Entry(plant_frame, width=40)
        den_entry.insert(0, "1, 2, 1")
        den_entry.pack(pady=5)
        
        # Performance Criteria
        perf_frame = tk.LabelFrame(root, text="Performance Criteria", padx=20, pady=10)
        perf_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(perf_frame, text="Rise Time (s):").pack(anchor='w')
        rt_entry = tk.Entry(perf_frame, width=20)
        rt_entry.insert(0, "0.5")
        rt_entry.pack(pady=5)
        
        tk.Label(perf_frame, text="Max Overshoot (%):").pack(anchor='w')
        os_entry = tk.Entry(perf_frame, width=20)
        os_entry.insert(0, "10")
        os_entry.pack(pady=5)
        
        # Results Display
        result_frame = tk.LabelFrame(root, text="Tuning Results", padx=20, pady=10)
        result_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        result_text = tk.Text(result_frame, height=10, width=60)
        result_text.pack()
        
        # Buttons
        button_frame = tk.Frame(root)
        button_frame.pack(pady=20)
        
        def run_tuning():
            result_text.delete(1.0, tk.END)
            result_text.insert(tk.END, "Starting MATLAB tuning...\n")
            
            # Parse inputs
            plant_model = {
                'numerator': [float(x.strip()) for x in num_entry.get().split(',')],
                'denominator': [float(x.strip()) for x in den_entry.get().split(',')]
            }
            
            performance_criteria = {
                'rise_time': float(rt_entry.get()),
                'overshoot': float(os_entry.get())
            }
            
            # Run tuning
            results = self.engine.tune_pid_parameters(plant_model, performance_criteria)
            
            if results:
                result_text.insert(tk.END, f"\nTuned Parameters:\n")
                result_text.insert(tk.END, f"Kp = {results.get('Kp', 'N/A')}\n")
                result_text.insert(tk.END, f"Ki = {results.get('Ki', 'N/A')}\n")
                result_text.insert(tk.END, f"Kd = {results.get('Kd', 'N/A')}\n")
                result_text.insert(tk.END, f"\nPerformance:\n")
                result_text.insert(tk.END, f"Rise Time: {results.get('RiseTime', 'N/A')} s\n")
                result_text.insert(tk.END, f"Overshoot: {results.get('Overshoot', 'N/A')} %\n")
                result_text.insert(tk.END, f"Settling Time: {results.get('SettlingTime', 'N/A')} s\n")
            else:
                result_text.insert(tk.END, "\nTuning failed. Check MATLAB installation.\n")
        
        tk.Button(
            button_frame,
            text="Run Tuning",
            command=run_tuning,
            bg="#4CAF50",
            fg="white",
            font=("Segoe UI", 11),
            padx=30,
            pady=10
        ).pack(side='left', padx=10)
        
        tk.Button(
            button_frame,
            text="Close",
            command=root.destroy,
            bg="#f44336",
            fg="white",
            font=("Segoe UI", 11),
            padx=30,
            pady=10
        ).pack(side='left', padx=10)
        
        root.mainloop()


def launch_matlab_tuning():
    """Launch MATLAB parameter tuning interface"""
    try:
        from arm_bench.matlab_integration_enhanced import launch_control_development
        launch_control_development()
    except Exception:
        gui = ParameterTuningGUI()
        gui.launch_tuning_interface()
