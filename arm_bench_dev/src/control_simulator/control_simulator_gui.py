"""
Control Simulator GUI
Multithreaded interface for control algorithm simulation, parameter tuning, and comparison
"""

import sys
import os
import json
import re
import threading
from datetime import datetime
from typing import Dict, List, Any

import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QTextEdit, QGroupBox,
    QGridLayout, QTabWidget, QTableWidget, QTableWidgetItem, QMessageBox,
    QFileDialog, QProgressBar, QDoubleSpinBox, QCheckBox, QSplitter
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor

# Import our modules
sys.path.insert(0, os.path.dirname(__file__))
from control_simulator.matlab_engine_interface import MATLABControlSimulator, extract_parameter_names
from control_simulator.parameter_tuner import ParameterTuner, ParameterBounds
from control_simulator.param_parser import parse_control_script, ParamError


class SimulationWorker(QThread):
    """Worker thread for running simulations"""
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    progress = pyqtSignal(str)
    
    def __init__(self, simulator, algorithm, parameters, sim_time, ref_signal, ref_amplitude, control_law_text=None):
        super().__init__()
        self.simulator = simulator
        self.algorithm = algorithm
        self.parameters = parameters
        self.sim_time = sim_time
        self.ref_signal = ref_signal
        self.ref_amplitude = ref_amplitude
        self.control_law_text = control_law_text
    
    def run(self):
        try:
            self.progress.emit(f"Running {self.algorithm} simulation...")
            results = self.simulator.run_control_simulation(
                algorithm_name=self.algorithm,
                parameters=self.parameters,
                sim_time=self.sim_time,
                reference_signal=self.ref_signal,
                reference_amplitude=self.ref_amplitude,
                control_law_text=self.control_law_text
            )
            self.finished.emit(results)
        except Exception as e:
            self.error.emit(str(e))


class TuningWorker(QThread):
    """Worker thread for parameter tuning"""
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    progress = pyqtSignal(dict)
    
    def __init__(self, tuner, algorithm, param_bounds, objective, method, max_iter, sim_time, ref_signal, ref_amp, control_law_text=None):
        super().__init__()
        self.tuner = tuner
        self.algorithm = algorithm
        self.param_bounds = param_bounds
        self.objective = objective
        self.method = method
        self.max_iter = max_iter
        self.sim_time = sim_time
        self.ref_signal = ref_signal
        self.ref_amp = ref_amp
        self.control_law_text = control_law_text
    
    def run(self):
        try:
            # Set callback for progress updates
            self.tuner.callback = lambda x: self.progress.emit(x)
            
            results = self.tuner.tune_parameters(
                algorithm_name=self.algorithm,
                parameter_bounds=self.param_bounds,
                objective=self.objective,
                method=self.method,
                max_iterations=self.max_iter,
                sim_time=self.sim_time,
                reference_signal=self.ref_signal,
                reference_amplitude=self.ref_amp,
                control_law_text=self.control_law_text
            )
            self.finished.emit(results)
        except Exception as e:
            self.error.emit(str(e))


class ControlSimulatorGUI(QMainWindow):
    """Main GUI for control simulator"""

    # The engine starts in a plain Python thread. A QTimer created in such a thread never fires
    # (the thread has no Qt event loop), so results are sent to the GUI thread with signals.
    engine_started = pyqtSignal(bool)
    log_requested = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.engine_started.connect(self.on_matlab_engine_started)
        self.log_requested.connect(self.log)
        self.setWindowTitle("MERLIN - Control Algorithm Tuning")
        self.setGeometry(100, 100, 1400, 900)
        
        # Initialize simulator and tuner
        self.simulator = None
        self.tuner = None
        self.simulation_history = []
        self.tuning_history = []
        
        # Worker threads
        self.sim_worker = None
        self.tune_worker = None
        
        self.init_ui()
        
        # Start MATLAB engine in background
        self.start_matlab_engine()
    
    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel: Control and parameters
        left_panel = self.create_control_panel()
        
        # Right panel: Visualization and results
        right_panel = self.create_visualization_panel()
        
        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar().showMessage("Ready")
    
    def create_control_panel(self) -> QWidget:
        """Create left control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Algorithm selection
        algo_group = QGroupBox("Algorithm Selection")
        algo_layout = QVBoxLayout()
        
        algo_layout.addWidget(QLabel("Control Algorithm:"))
        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItems(['PID', 'LQR', 'MPC'])
        self.algorithm_combo.currentTextChanged.connect(self.on_algorithm_changed)
        algo_layout.addWidget(self.algorithm_combo)
        
        algo_group.setLayout(algo_layout)
        layout.addWidget(algo_group)
        
        # Parameters group
        self.param_group = QGroupBox("Controller Parameters")
        self.param_layout = QGridLayout()
        self.param_group.setLayout(self.param_layout)
        layout.addWidget(self.param_group)

        # Control law input
        law_group = QGroupBox("Free-Text Control Law")
        law_layout = QVBoxLayout()
        law_label = QLabel("Enter a control law that assigns to u. Declare parameters with a line like "
                           "'% @param min=0 max=50 desc=\"gain\"' above 'Kp = 8;' (see examples/); "
                           "without @param every free name is treated as a parameter.")
        law_label.setWordWrap(True)
        law_layout.addWidget(law_label)
        self.control_law_edit = QTextEdit()
        self.control_law_edit.setPlaceholderText("Example: u = Kp*e + Ki*integral + Kd*derivative;")
        self.control_law_edit.textChanged.connect(self.on_control_law_changed)
        law_layout.addWidget(self.control_law_edit)
        self.param_status_label = QLabel("")
        self.param_status_label.setWordWrap(True)
        law_layout.addWidget(self.param_status_label)
        load_btn = QPushButton("Load script...")
        load_btn.clicked.connect(self.load_script)
        law_layout.addWidget(load_btn)
        law_group.setLayout(law_layout)
        layout.addWidget(law_group)
        
        # Simulation settings
        sim_group = QGroupBox("Simulation Settings")
        sim_layout = QGridLayout()
        
        sim_layout.addWidget(QLabel("Simulation Time (s):"), 0, 0)
        self.sim_time_spin = QDoubleSpinBox()
        self.sim_time_spin.setRange(1.0, 100.0)
        self.sim_time_spin.setValue(10.0)
        sim_layout.addWidget(self.sim_time_spin, 0, 1)
        
        sim_layout.addWidget(QLabel("Reference Signal:"), 1, 0)
        self.ref_signal_combo = QComboBox()
        self.ref_signal_combo.addItems(['step', 'ramp', 'sine', 'square'])
        sim_layout.addWidget(self.ref_signal_combo, 1, 1)
        
        sim_layout.addWidget(QLabel("Reference Amplitude:"), 2, 0)
        self.ref_amp_spin = QDoubleSpinBox()
        self.ref_amp_spin.setRange(0.1, 10.0)
        self.ref_amp_spin.setValue(1.0)
        sim_layout.addWidget(self.ref_amp_spin, 2, 1)
        
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # Tuning settings
        tune_group = QGroupBox("Parameter Tuning")
        tune_layout = QGridLayout()
        
        tune_layout.addWidget(QLabel("Objective:"), 0, 0)
        self.objective_combo = QComboBox()
        self.objective_combo.addItems(['ise', 'iae', 'itae', 'custom'])
        tune_layout.addWidget(self.objective_combo, 0, 1)
        
        tune_layout.addWidget(QLabel("Method:"), 1, 0)
        self.method_combo = QComboBox()
        self.method_combo.addItems(['differential_evolution', 'nelder-mead', 'powell'])
        tune_layout.addWidget(self.method_combo, 1, 1)
        
        tune_layout.addWidget(QLabel("Max Iterations:"), 2, 0)
        self.max_iter_spin = QDoubleSpinBox()
        self.max_iter_spin.setRange(10, 500)
        self.max_iter_spin.setValue(50)
        self.max_iter_spin.setDecimals(0)
        tune_layout.addWidget(self.max_iter_spin, 2, 1)
        
        tune_group.setLayout(tune_layout)
        layout.addWidget(tune_group)
        
        # Buttons
        btn_layout = QVBoxLayout()
        
        self.run_btn = QPushButton("Run Simulation")
        self.run_btn.clicked.connect(self.run_simulation)
        btn_layout.addWidget(self.run_btn)
        
        self.tune_btn = QPushButton("Auto-Tune Parameters")
        self.tune_btn.clicked.connect(self.run_tuning)
        btn_layout.addWidget(self.tune_btn)
        
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_operation)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_btn)
        
        self.save_btn = QPushButton("Save Results")
        self.save_btn.clicked.connect(self.save_results)
        btn_layout.addWidget(self.save_btn)
        
        self.load_btn = QPushButton("Load Configuration")
        self.load_btn.clicked.connect(self.load_configuration)
        btn_layout.addWidget(self.load_btn)

        # Closing this window returns to the MERLIN main window
        self.back_btn = QPushButton("← Back to MERLIN")
        self.back_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.back_btn)
        
        layout.addLayout(btn_layout)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Log
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        layout.addStretch()
        
        # Initialize with PID parameters
        self.on_algorithm_changed('PID')
        
        return panel
    
    def create_visualization_panel(self) -> QWidget:
        """Create right visualization panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Tabs for different views
        self.tab_widget = QTabWidget()
        
        # Tab 1: Time response
        self.response_tab = QWidget()
        response_layout = QVBoxLayout(self.response_tab)
        self.response_canvas = MplCanvas()
        response_layout.addWidget(self.response_canvas)
        self.tab_widget.addTab(self.response_tab, "Time Response")
        
        # Tab 2: Metrics
        self.metrics_tab = QWidget()
        metrics_layout = QVBoxLayout(self.metrics_tab)
        self.metrics_table = QTableWidget()
        self.metrics_table.setColumnCount(2)
        self.metrics_table.setHorizontalHeaderLabels(["Metric", "Value"])
        metrics_layout.addWidget(self.metrics_table)
        self.tab_widget.addTab(self.metrics_tab, "Performance Metrics")
        
        # Tab 3: Comparison
        self.comparison_tab = QWidget()
        comparison_layout = QVBoxLayout(self.comparison_tab)
        self.comparison_canvas = MplCanvas()
        comparison_layout.addWidget(self.comparison_canvas)
        self.tab_widget.addTab(self.comparison_tab, "Comparison")
        
        # Tab 4: Tuning history
        self.tuning_tab = QWidget()
        tuning_layout = QVBoxLayout(self.tuning_tab)
        self.tuning_canvas = MplCanvas()
        tuning_layout.addWidget(self.tuning_canvas)
        self.tab_widget.addTab(self.tuning_tab, "Tuning History")
        
        layout.addWidget(self.tab_widget)
        
        return panel
    
    def on_algorithm_changed(self, algorithm: str):
        """Update parameter inputs when algorithm changes."""
        self.refresh_parameter_inputs()

    def on_control_law_changed(self):
        """Refresh dynamic parameter inputs from the free-text control law."""
        self.refresh_parameter_inputs()

    def refresh_parameter_inputs(self):
        """Populate parameter rows from the free-text control law, falling back to algorithm defaults."""
        previous_param_inputs = getattr(self, 'param_inputs', {})
        previous_bounds_inputs = getattr(self, 'param_bounds_inputs', {})

        while self.param_layout.count():
            child = self.param_layout.takeAt(0)
            widget = child.widget()
            if widget:
                widget.setParent(None)      # hide now; deleteLater alone leaves it drawn until the loop runs
                widget.deleteLater()

        self.param_inputs = {}
        self.param_bounds_inputs = {}

        algorithm = self.algorithm_combo.currentText().upper()
        existing_values = {name: spin.value() for name, spin in previous_param_inputs.items()}
        existing_bounds = {name: previous_bounds_inputs[name] for name in previous_bounds_inputs}

        default_params = {
            'PID': [('Kp', 1.0, 0.0, 100.0), ('Ki', 0.1, 0.0, 50.0), ('Kd', 0.05, 0.0, 20.0)],
            'LQR': [('Q_pos', 10.0, 0.1, 1000.0), ('Q_vel', 1.0, 0.1, 100.0), ('R', 0.1, 0.01, 10.0)],
            'MPC': [('Horizon', 10.0, 5.0, 50.0), ('Q_weight', 10.0, 0.1, 1000.0), ('R_weight', 1.0, 0.1, 100.0)]
        }

        law_text = self.control_law_edit.toPlainText() if hasattr(self, 'control_law_edit') else ''
        declared = {}
        status = ""
        try:
            parsed = parse_control_script(law_text)
        except ParamError as exc:
            parsed = None
            status = f"@param error: {exc}"
        if parsed is not None and parsed.annotated:
            # '@param' declarations: default/min/max come from the script, existing edits are kept
            params = [(p.name, p.default, p.min, p.max) for p in parsed.params]
            declared = {p.name: p for p in parsed.params}
            status = f"{len(params)} parameter(s) read from @param" + "".join(f"\nwarning: {w}" for w in parsed.warnings)
            # keep a value the user typed into a spin box, but not across a changed script default
            last = getattr(self, '_last_declared_defaults', {})
            existing_values = {n: v for n, v in existing_values.items()
                               if n in declared and last.get(n) == declared[n].default}
            existing_bounds = {}
            self._last_declared_defaults = {n: p.default for n, p in declared.items()}
        else:
            parsed_params = extract_parameter_names(law_text) if parsed is not None else []
            if parsed_params:
                params = [(name, 1.0, 0.0, 100.0) for name in parsed_params]
            else:
                params = default_params.get(algorithm, [])
        if hasattr(self, 'param_status_label'):
            self.param_status_label.setText(status)

        for i, (name, default, min_val, max_val) in enumerate(params):
            current_value = existing_values.get(name, default)
            current_min = existing_bounds.get(name, (None, None))[0]
            current_max = existing_bounds.get(name, (None, None))[1]
            if current_min is None:
                current_min = min_val
            if current_max is None:
                current_max = max_val

            self.param_layout.addWidget(QLabel(f"{name}:"), i, 0)

            spin = QDoubleSpinBox()
            spin.setRange(min(min_val, 0.0) if name not in declared else min_val,
                          max(max_val, 100.0) if name not in declared else max_val)
            spin.setValue(current_value)
            if name in declared:
                spec = declared[name]
                spin.setToolTip(f"{spec.desc} [{spec.unit}]".strip() if spec.desc or spec.unit else name)
                if spec.step:
                    spin.setSingleStep(spec.step)
            spin.setDecimals(4)
            self.param_layout.addWidget(spin, i, 1)
            self.param_inputs[name] = spin

            self.param_layout.addWidget(QLabel("Min:"), i, 2)
            min_spin = QDoubleSpinBox()
            min_spin.setRange(-1e6, 1e6)
            min_spin.setValue(current_min.value() if hasattr(current_min, 'value') else current_min)
            min_spin.setDecimals(4)
            self.param_layout.addWidget(min_spin, i, 3)

            self.param_layout.addWidget(QLabel("Max:"), i, 4)
            max_spin = QDoubleSpinBox()
            max_spin.setRange(-1e6, 1e6)
            max_spin.setValue(current_max.value() if hasattr(current_max, 'value') else current_max)
            max_spin.setDecimals(4)
            self.param_layout.addWidget(max_spin, i, 5)

            self.param_bounds_inputs[name] = (min_spin, max_spin)
    
    def load_script(self):
        """Load a control script (with optional @param header) into the control-law box."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load control script", "", "Control scripts (*.m *.txt);;All files (*)"
        )
        if filename:
            with open(filename, 'r') as f:
                self.control_law_edit.setPlainText(f.read())
            self.log(f"Loaded control script {filename}")

    def start_matlab_engine(self):
        """Start MATLAB engine (or the GNU Octave fallback) in the background."""
        def start_engine():
            success = False
            try:
                self.log_requested.emit("Starting MATLAB engine...")
                self.simulator = MATLABControlSimulator()
                success = self.simulator.start_engine()
            except Exception as e:
                self.log_requested.emit(f"MATLAB engine thread error: {e}")
            finally:
                self.engine_started.emit(success)

        thread = threading.Thread(target=start_engine, daemon=True)
        thread.start()

    def on_matlab_engine_started(self, success: bool):
        """Handle the engine startup result (runs on the Qt main thread)."""
        if success:
            self.tuner = ParameterTuner(self.simulator)
            backend = getattr(self.simulator, "backend", None) or "MATLAB"
            self.log(f"{backend} engine started successfully")
            self.statusBar().showMessage(f"{backend} engine ready")
        else:
            self.log("ERROR: Failed to start MATLAB engine (and no GNU Octave fallback)")
            self.statusBar().showMessage("MATLAB engine failed")

    def run_simulation(self):
        """Run control simulation"""
        if self.simulator is None or not self.simulator.is_engine_running():
            QMessageBox.warning(self, "Warning", "MATLAB engine not ready. Please wait...")
            return
        
        if self.sim_worker and self.sim_worker.isRunning():
            QMessageBox.warning(self, "Warning", "Simulation already running")
            return
        
        # Get parameters
        algorithm = self.algorithm_combo.currentText().lower()
        parameters = {name: spin.value() for name, spin in self.param_inputs.items()}
        sim_time = self.sim_time_spin.value()
        ref_signal = self.ref_signal_combo.currentText()
        ref_amp = self.ref_amp_spin.value()
        control_law_text = self.control_law_edit.toPlainText().strip()
        
        # Create worker
        self.sim_worker = SimulationWorker(
            self.simulator, algorithm, parameters, sim_time, ref_signal, ref_amp, control_law_text
        )
        self.sim_worker.finished.connect(self.on_simulation_finished)
        self.sim_worker.error.connect(self.on_error)
        self.sim_worker.progress.connect(self.log)
        
        # UI updates
        self.run_btn.setEnabled(False)
        self.tune_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate
        
        self.log(f"Starting {algorithm.upper()} simulation...")
        self.sim_worker.start()
    
    def on_simulation_finished(self, results: Dict[str, Any]):
        """Handle simulation completion"""
        self.log("Simulation completed")
        
        # Store results
        self.simulation_history.append({
            'timestamp': datetime.now().isoformat(),
            'algorithm': self.algorithm_combo.currentText(),
            'parameters': {name: spin.value() for name, spin in self.param_inputs.items()},
            'results': results
        })
        
        # Update visualizations
        self.plot_response(results)
        self.display_metrics(results['metrics'])
        self.update_comparison()
        
        # UI updates
        self.run_btn.setEnabled(True)
        self.tune_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
        
        self.statusBar().showMessage("Simulation completed", 3000)
    
    def run_tuning(self):
        """Run parameter tuning"""
        if self.simulator is None or not self.simulator.is_engine_running():
            QMessageBox.warning(self, "Warning", "MATLAB engine not ready. Please wait...")
            return
        
        if self.tune_worker and self.tune_worker.isRunning():
            QMessageBox.warning(self, "Warning", "Tuning already running")
            return
        
        # Get settings
        algorithm = self.algorithm_combo.currentText().lower()
        objective = self.objective_combo.currentText()
        method = self.method_combo.currentText()
        max_iter = int(self.max_iter_spin.value())
        sim_time = self.sim_time_spin.value()
        ref_signal = self.ref_signal_combo.currentText()
        ref_amp = self.ref_amp_spin.value()
        
        # Get parameter bounds
        param_bounds = []
        for name, spin in self.param_inputs.items():
            min_spin, max_spin = self.param_bounds_inputs[name]
            param_bounds.append(ParameterBounds(
                name=name,
                min_value=min_spin.value(),
                max_value=max_spin.value(),
                initial_value=spin.value()
            ))
        
        control_law_text = self.control_law_edit.toPlainText().strip()

        # Create worker
        self.tune_worker = TuningWorker(
            self.tuner, algorithm, param_bounds, objective, method,
            max_iter, sim_time, ref_signal, ref_amp, control_law_text
        )
        self.tune_worker.finished.connect(self.on_tuning_finished)
        self.tune_worker.error.connect(self.on_error)
        self.tune_worker.progress.connect(self.on_tuning_progress)
        
        # UI updates
        self.run_btn.setEnabled(False)
        self.tune_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, max_iter)
        
        self.log(f"Starting parameter tuning ({method})...")
        self.tune_worker.start()
    
    def on_tuning_progress(self, progress_data: Dict):
        """Handle tuning progress update"""
        iteration = progress_data.get('iteration', 0)
        score = progress_data.get('score', 0)
        best_score = progress_data.get('best_score', 0)
        
        self.progress_bar.setValue(iteration)
        self.log(f"Iteration {iteration}: score={score:.4f}, best={best_score:.4f}")
        
        # Update tuning history plot
        if hasattr(self, 'tuner') and self.tuner:
            self.plot_tuning_history(self.tuner.tuning_history)
    
    def on_tuning_finished(self, results: Dict[str, Any]):
        """Handle tuning completion"""
        self.log("Tuning completed")
        
        optimal_params = results.get('optimal_parameters', {})
        optimal_score = results.get('optimal_score', 0)
        
        self.log(f"Optimal parameters: {optimal_params}")
        self.log(f"Optimal score: {optimal_score:.4f}")
        
        # Update parameter inputs with optimal values
        for name, value in optimal_params.items():
            if name in self.param_inputs:
                self.param_inputs[name].setValue(value)
        
        # Store tuning results
        self.tuning_history.append({
            'timestamp': datetime.now().isoformat(),
            'algorithm': self.algorithm_combo.currentText(),
            'results': results
        })
        
        # Plot tuning history
        if 'history' in results:
            self.plot_tuning_history(results['history'])
        
        # UI updates
        self.run_btn.setEnabled(True)
        self.tune_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
        
        self.statusBar().showMessage("Tuning completed", 3000)
        
        # Ask if user wants to run simulation with optimal parameters
        reply = QMessageBox.question(
            self, 'Run Simulation',
            'Run simulation with optimal parameters?',
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.run_simulation()
    
    def stop_operation(self):
        """Stop current operation"""
        if self.sim_worker and self.sim_worker.isRunning():
            self.sim_worker.terminate()
            self.log("Simulation stopped")
        
        if self.tune_worker and self.tune_worker.isRunning():
            if self.tuner:
                self.tuner.stop_tuning()
            self.tune_worker.terminate()
            self.log("Tuning stopped")
        
        self.run_btn.setEnabled(True)
        self.tune_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
    
    def plot_response(self, results: Dict[str, Any]):
        """Plot time response"""
        self.response_canvas.figure.clear()
        
        time = np.array(results['time']).flatten()
        states = np.array(results['states']).flatten()
        reference = np.array(results['reference']).flatten()
        control = np.array(results['control']).flatten()
        
        # Two subplots: output and control
        ax1 = self.response_canvas.figure.add_subplot(211)
        ax1.plot(time, reference, 'r--', label='Reference', linewidth=2)
        ax1.plot(time, states, 'b-', label='Output', linewidth=2)
        ax1.set_ylabel('Output')
        ax1.legend()
        ax1.grid(True)

        ax1.set_title('System Response')
        # param_str = ', '.join(f"{k}={v:.3f}" for k, v in (parameters or {}).items())
        # ax1.set_title(f'System Response ({param_str})')        
        ax2 = self.response_canvas.figure.add_subplot(212)
        ax2.plot(time, control, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Input')
        ax2.grid(True)
        ax2.set_title('Control Effort')
        
        self.response_canvas.figure.tight_layout()
        self.response_canvas.draw()
    
    def display_metrics(self, metrics: Dict[str, float]):
        """Display performance metrics in table"""
        self.metrics_table.setRowCount(len(metrics))
        
        for i, (metric, value) in enumerate(metrics.items()):
            self.metrics_table.setItem(i, 0, QTableWidgetItem(metric))
            self.metrics_table.setItem(i, 1, QTableWidgetItem(f"{value:.6f}"))
        
        self.metrics_table.resizeColumnsToContents()
    
    def update_comparison(self):
        """Update comparison plots"""
        if len(self.simulation_history) < 2:
            return
        
        self.comparison_canvas.figure.clear()
        
        # Plot last few simulations
        ax = self.comparison_canvas.figure.add_subplot(111)
        
        for i, sim in enumerate(self.simulation_history[-5:]):  # Last 5
            results = sim['results']
            time = np.array(results['time']).flatten()
            states = np.array(results['states']).flatten()
            label = f"{sim['algorithm']} - {sim['timestamp'][:19]}"
            ax.plot(time, states, label=label, linewidth=2)
        
        # Reference
        if self.simulation_history:
            results = self.simulation_history[-1]['results']
            time = np.array(results['time']).flatten()
            reference = np.array(results['reference']).flatten()
            ax.plot(time, reference, 'k--', label='Reference', linewidth=2)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Output')
        ax.legend()
        ax.grid(True)
        ax.set_title('Simulation Comparison')
        
        self.comparison_canvas.figure.tight_layout()
        self.comparison_canvas.draw()
    
    def plot_tuning_history(self, history: List[Dict]):
        """Plot tuning history"""
        if not history:
            return
        
        self.tuning_canvas.figure.clear()
        
        iterations = [h['iteration'] for h in history]
        scores = [h['score'] for h in history]
        
        ax = self.tuning_canvas.figure.add_subplot(111)
        ax.plot(iterations, scores, 'bo-', linewidth=2, markersize=4)
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Objective Score')
        ax.set_title('Parameter Tuning Progress')
        ax.grid(True)
        
        # Mark best
        best_idx = np.argmin(scores)
        ax.plot(iterations[best_idx], scores[best_idx], 'r*', markersize=15, label='Best')
        ax.legend()
        
        self.tuning_canvas.figure.tight_layout()
        self.tuning_canvas.draw()
    
    def save_results(self):
        """Save simulation and tuning results"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Results", "", "JSON Files (*.json)"
        )
        
        if filename:
            data = {
                'simulation_history': self.simulation_history,
                'tuning_history': self.tuning_history
            }
            
            try:
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2, default=str)
                self.log(f"Results saved to {filename}")
                QMessageBox.information(self, "Success", "Results saved successfully")
            except Exception as e:
                self.log(f"Error saving results: {e}")
                QMessageBox.critical(self, "Error", f"Failed to save results: {e}")
    
    def load_configuration(self):
        """Load configuration from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON Files (*.json)"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                if 'simulation_history' in data:
                    self.simulation_history = data['simulation_history']
                if 'tuning_history' in data:
                    self.tuning_history = data['tuning_history']
                
                self.update_comparison()
                self.log(f"Configuration loaded from {filename}")
                QMessageBox.information(self, "Success", "Configuration loaded successfully")
            except Exception as e:
                self.log(f"Error loading configuration: {e}")
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {e}")
    
    def log(self, message: str):
        """Add message to log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
    
    def on_error(self, error_msg: str):
        """Handle error"""
        self.log(f"ERROR: {error_msg}")
        QMessageBox.critical(self, "Error", error_msg)
        
        self.run_btn.setEnabled(True)
        self.tune_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
    
    def closeEvent(self, event):
        """Handle window close"""
        if self.simulator and self.simulator.is_engine_running():
            reply = QMessageBox.question(
                self, 'Close',
                'Stop MATLAB engine and close application?',
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.simulator.stop_engine()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


class MplCanvas(FigureCanvas):
    """Matplotlib canvas widget"""
    
    def __init__(self, parent=None, width=8, height=6, dpi=100):
        self.figure = Figure(figsize=(width, height), dpi=dpi)
        super().__init__(self.figure)


def main():
    """Run the application"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = ControlSimulatorGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
