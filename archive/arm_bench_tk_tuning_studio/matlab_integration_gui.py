"""Comprehensive GUI for control algorithm development, tuning, and visualization."""

import importlib
import sys
import traceback
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
from typing import Dict, Any, List
import numpy as np
import threading
import json
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

from .matlab_integration_enhanced import (
    ControlAlgorithm, MATLABOptimizer, compare_parameter_sets
)


class ControlDevelopmentGUI:
    """Main GUI for control algorithm development."""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ARM Bench - Control Algorithm Development Studio")
        self.root.geometry("1600x900")
        
        # Data
        self.algorithm = ControlAlgorithm("My Controller")
        self.optimizer = MATLABOptimizer()
        self.simulation_results = []
        self.comparison_results = []
        
        # Colors
        self.colors = {
            'bg': '#1e1e1e',
            'panel': '#2d2d2d',
            'accent': '#007acc',
            'success': '#4ec9b0',
            'warning': '#dcdcaa',
            'error': '#f48771',
            'text': '#d4d4d4'
        }
        
        self.root.configure(bg=self.colors['bg'])
        self._setup_ui()
        self._start_matlab_engine_thread()
    
    def _start_matlab_engine_thread(self):
        """Preload MATLAB engine in background for optimization."""
        def preload_engine():
            try:
                if self.optimizer.start_engine():
                    self.root.after(0, lambda: self.update_status("MATLAB engine ready", "success"))
                else:
                    self.root.after(0, lambda: self.update_status("MATLAB engine unavailable; using Python optimizer", "warning"))
            except Exception as e:
                traceback.print_exc()
                self.root.after(0, lambda: self.update_status("MATLAB engine startup failed", "error"))

        thread = threading.Thread(target=preload_engine, daemon=True)
        thread.start()

    def _setup_ui(self):
        """Setup main user interface."""
        # Title bar
        title_frame = tk.Frame(self.root, bg=self.colors['accent'], height=60)
        title_frame.pack(fill='x')
        title_frame.pack_propagate(False)
        
        title = tk.Label(title_frame, text="🎮 Control Algorithm Development Studio", 
                        font=('Segoe UI', 18, 'bold'),
                        bg=self.colors['accent'], fg='white')
        title.pack(side='left', padx=20, pady=15)
        
        # Status indicator
        self.status_label = tk.Label(title_frame, text="● Ready", 
                                     font=('Segoe UI', 10),
                                     bg=self.colors['accent'], fg=self.colors['success'])
        self.status_label.pack(side='right', padx=20)
        
        # Main paned window
        main_paned = ttk.PanedWindow(self.root, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left panel - Configuration (40% width)
        left_frame = tk.Frame(main_paned, bg=self.colors['panel'], width=640)
        main_paned.add(left_frame, weight=2)
        
        # Right panel - Visualization (60% width)
        right_frame = tk.Frame(main_paned, bg=self.colors['panel'])
        main_paned.add(right_frame, weight=3)
        
        self._setup_left_panel(left_frame)
        self._setup_right_panel(right_frame)
    
    def _setup_left_panel(self, parent):
        """Setup left configuration panel."""
        notebook = ttk.Notebook(parent)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Tab 1: Control Logic
        logic_frame = tk.Frame(notebook, bg=self.colors['panel'])
        notebook.add(logic_frame, text="📝 Control Logic")
        self._setup_logic_tab(logic_frame)
        
        # Tab 2: Parameters
        params_frame = tk.Frame(notebook, bg=self.colors['panel'])
        notebook.add(params_frame, text="⚙️ Parameters")
        self._setup_parameters_tab(params_frame)
        
        # Tab 3: Simulation
        sim_frame = tk.Frame(notebook, bg=self.colors['panel'])
        notebook.add(sim_frame, text="🎯 Simulation")
        self._setup_simulation_tab(sim_frame)
        
        # Tab 4: Optimization
        opt_frame = tk.Frame(notebook, bg=self.colors['panel'])
        notebook.add(opt_frame, text="🔧 Optimization")
        self._setup_optimization_tab(opt_frame)
        
        # Tab 5: Comparison
        comp_frame = tk.Frame(notebook, bg=self.colors['panel'])
        notebook.add(comp_frame, text="📊 Compare")
        self._setup_comparison_tab(comp_frame)
    
    def _setup_right_panel(self, parent):
        """Setup right visualization panel."""
        # Top: Control buttons
        btn_frame = tk.Frame(parent, bg=self.colors['panel'], height=60)
        btn_frame.pack(fill='x', padx=10, pady=5)
        btn_frame.pack_propagate(False)
        
        ttk.Button(btn_frame, text="▶️  Run Simulation", 
                  command=self.run_simulation).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="🔧 Optimize Parameters", 
                  command=self.run_optimization).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="📊 Compare Sets", 
                  command=self.run_comparison).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="💾 Save Results", 
                  command=self.save_results).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="📂 Load Config", 
                  command=self.load_config).pack(side='left', padx=5)
        
        # Visualization area
        if MATPLOTLIB_AVAILABLE:
            self._setup_plots(parent)
        else:
            no_plot_label = tk.Label(parent, 
                                     text="⚠️ Matplotlib not installed\nInstall with: pip install matplotlib",
                                     bg=self.colors['panel'], fg=self.colors['warning'],
                                     font=('Segoe UI', 12))
            no_plot_label.pack(expand=True)
    
    def _setup_plots(self, parent):
        """Setup matplotlib plots."""
        self.fig = Figure(figsize=(10, 8), facecolor=self.colors['panel'])
        
        # Create subplots
        self.ax1 = self.fig.add_subplot(3, 1, 1)
        self.ax2 = self.fig.add_subplot(3, 1, 2)
        self.ax3 = self.fig.add_subplot(3, 1, 3)
        
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_facecolor('#2d2d2d')
            ax.tick_params(colors='#d4d4d4')
            ax.spines['bottom'].set_color('#d4d4d4')
            ax.spines['top'].set_color('#d4d4d4')
            ax.spines['left'].set_color('#d4d4d4')
            ax.spines['right'].set_color('#d4d4d4')
            ax.xaxis.label.set_color('#d4d4d4')
            ax.yaxis.label.set_color('#d4d4d4')
            ax.title.set_color('#d4d4d4')
            ax.grid(True, alpha=0.2)
        
        self.ax1.set_title("System Response")
        self.ax1.set_ylabel("Position")
        self.ax2.set_title("Control Signal")
        self.ax2.set_ylabel("Control")
        self.ax3.set_title("Tracking Error")
        self.ax3.set_ylabel("Error")
        self.ax3.set_xlabel("Time (s)")
        
        self.fig.tight_layout()
        
        # Canvas
        canvas_frame = tk.Frame(parent, bg=self.colors['panel'])
        canvas_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.canvas = FigureCanvasTkAgg(self.fig, canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # Toolbar
        toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame)
        toolbar.update()
    
    def _setup_logic_tab(self, parent):
        """Control logic editor tab."""
        # Title
        tk.Label(parent, text="Define Your Control Algorithm", 
                font=('Segoe UI', 12, 'bold'),
                bg=self.colors['panel'], fg=self.colors['text']).pack(pady=10)
        
        # Instructions
        instructions = tk.Label(parent, 
                               text="Write Python code for your controller.\n"
                                    "Available: state (np.array), reference (float), dt (float), parameters\n"
                                    "Set 'control_output' variable with your result.",
                               font=('Segoe UI', 9), justify='left',
                               bg=self.colors['panel'], fg=self.colors['warning'])
        instructions.pack(pady=5)
        
        # Code editor
        editor_frame = tk.Frame(parent, bg=self.colors['panel'])
        editor_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.code_editor = scrolledtext.ScrolledText(
            editor_frame,
            font=('Consolas', 10),
            bg='#1e1e1e',
            fg='#d4d4d4',
            insertbackground='white',
            selectbackground=self.colors['accent'],
            wrap='word'
        )
        self.code_editor.pack(fill='both', expand=True)
        
        # Default template
        default_code = """# PD Controller Example
# Define Kp, Kd in Parameters tab

error = reference - state[0]
error_dot = -state[1]  # Assuming state[1] is velocity

control_output = Kp * error + Kd * error_dot

# Optional: Add saturation
max_control = 10.0
control_output = np.clip(control_output, -max_control, max_control)
"""
        self.code_editor.insert('1.0', default_code)
        
        # Buttons
        btn_frame = tk.Frame(parent, bg=self.colors['panel'])
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="💾 Save Logic", 
                  command=self.save_control_logic).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="📂 Load from File", 
                  command=self.load_control_logic).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="✓ Validate Syntax", 
                  command=self.validate_logic).pack(side='left', padx=5)
    
    def _setup_parameters_tab(self, parent):
        """Parameters configuration tab."""
        tk.Label(parent, text="Control Parameters", 
                font=('Segoe UI', 12, 'bold'),
                bg=self.colors['panel'], fg=self.colors['text']).pack(pady=10)
        
        # Add parameter section
        add_frame = tk.LabelFrame(parent, text="Add New Parameter",
                                 bg=self.colors['panel'], fg=self.colors['text'],
                                 font=('Segoe UI', 10, 'bold'))
        add_frame.pack(fill='x', padx=10, pady=5)
        
        row1 = tk.Frame(add_frame, bg=self.colors['panel'])
        row1.pack(pady=5, padx=10)
        
        tk.Label(row1, text="Name:", bg=self.colors['panel'], 
                fg=self.colors['text'], width=8).pack(side='left')
        self.param_name_entry = ttk.Entry(row1, width=15)
        self.param_name_entry.pack(side='left', padx=5)
        
        tk.Label(row1, text="Value:", bg=self.colors['panel'], 
                fg=self.colors['text'], width=8).pack(side='left')
        self.param_value_entry = ttk.Entry(row1, width=12)
        self.param_value_entry.pack(side='left', padx=5)
        
        row2 = tk.Frame(add_frame, bg=self.colors['panel'])
        row2.pack(pady=5, padx=10)
        
        tk.Label(row2, text="Min:", bg=self.colors['panel'], 
                fg=self.colors['text'], width=8).pack(side='left')
        self.param_min_entry = ttk.Entry(row2, width=12)
        self.param_min_entry.pack(side='left', padx=5)
        
        tk.Label(row2, text="Max:", bg=self.colors['panel'], 
                fg=self.colors['text'], width=8).pack(side='left')
        self.param_max_entry = ttk.Entry(row2, width=12)
        self.param_max_entry.pack(side='left', padx=5)
        
        ttk.Button(add_frame, text="➕ Add", 
                  command=self.add_parameter).pack(pady=5)
        
        # Parameters list
        list_frame = tk.LabelFrame(parent, text="Current Parameters",
                                  bg=self.colors['panel'], fg=self.colors['text'],
                                  font=('Segoe UI', 10, 'bold'))
        list_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Scrollable container
        canvas = tk.Canvas(list_frame, bg=self.colors['panel'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        self.params_container = tk.Frame(canvas, bg=self.colors['panel'])
        
        self.params_container.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=self.params_container, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        self.param_widgets = {}
        
        # Buttons
        btn_frame = tk.Frame(parent, bg=self.colors['panel'])
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="💾 Save All", 
                  command=self.save_all_parameters).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="🔄 Reset", 
                  command=self.reset_parameters).pack(side='left', padx=5)
    
    def _setup_simulation_tab(self, parent):
        """Simulation configuration tab."""
        tk.Label(parent, text="Simulation Configuration", 
                font=('Segoe UI', 12, 'bold'),
                bg=self.colors['panel'], fg=self.colors['text']).pack(pady=10)
        
        config_frame = tk.LabelFrame(parent, text="Settings",
                                    bg=self.colors['panel'], fg=self.colors['text'])
        config_frame.pack(fill='x', padx=10, pady=5)
        
        # Time step
        row1 = tk.Frame(config_frame, bg=self.colors['panel'])
        row1.pack(fill='x', pady=5, padx=10)
        tk.Label(row1, text="Time Step (dt):", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.dt_entry = ttk.Entry(row1, width=15)
        self.dt_entry.insert(0, "0.01")
        self.dt_entry.pack(side='left', padx=5)
        tk.Label(row1, text="seconds", bg=self.colors['panel'], 
                fg=self.colors['warning']).pack(side='left')
        
        # Time steps
        row2 = tk.Frame(config_frame, bg=self.colors['panel'])
        row2.pack(fill='x', pady=5, padx=10)
        tk.Label(row2, text="Number of Steps:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.steps_entry = ttk.Entry(row2, width=15)
        self.steps_entry.insert(0, "1000")
        self.steps_entry.pack(side='left', padx=5)
        
        self.time_label = tk.Label(row2, text="Total: 10.0s",
                                   bg=self.colors['panel'], fg=self.colors['success'])
        self.time_label.pack(side='left', padx=10)
        
        # Initial state
        row3 = tk.Frame(config_frame, bg=self.colors['panel'])
        row3.pack(fill='x', pady=5, padx=10)
        tk.Label(row3, text="Initial State:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.initial_state_entry = ttk.Entry(row3, width=25)
        self.initial_state_entry.insert(0, "[0.0, 0.0]")
        self.initial_state_entry.pack(side='left', padx=5)
        
        # Reference
        row4 = tk.Frame(config_frame, bg=self.colors['panel'])
        row4.pack(fill='x', pady=5, padx=10)
        tk.Label(row4, text="Reference Value:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.reference_entry = ttk.Entry(row4, width=15)
        self.reference_entry.insert(0, "1.0")
        self.reference_entry.pack(side='left', padx=5)
        
        # Plant model
        plant_frame = tk.LabelFrame(parent, text="Plant Model",
                                   bg=self.colors['panel'], fg=self.colors['text'])
        plant_frame.pack(fill='x', padx=10, pady=10)
        
        self.plant_var = tk.StringVar(value="double_integrator")
        
        models = [
            ("Double Integrator (ẍ = u)", "double_integrator"),
            ("First Order (ẋ = -ax + bu)", "first_order"),
            ("Mass-Spring-Damper", "mass_spring_damper"),
        ]
        
        for text, value in models:
            tk.Radiobutton(plant_frame, text=text, variable=self.plant_var,
                          value=value, bg=self.colors['panel'], 
                          fg=self.colors['text'],
                          selectcolor=self.colors['accent']).pack(anchor='w', padx=10, pady=2)
    
    def _setup_optimization_tab(self, parent):
        """Optimization configuration tab."""
        tk.Label(parent, text="Parameter Optimization", 
                font=('Segoe UI', 12, 'bold'),
                bg=self.colors['panel'], fg=self.colors['text']).pack(pady=10)
        
        # Method selection
        method_frame = tk.LabelFrame(parent, text="Optimization Method",
                                    bg=self.colors['panel'], fg=self.colors['text'])
        method_frame.pack(fill='x', padx=10, pady=5)
        
        self.opt_method_var = tk.StringVar(value="auto")
        
        methods = [
            ("Auto (MATLAB if available, else Python)", "auto"),
            ("MATLAB fmincon", "fmincon"),
            ("MATLAB Genetic Algorithm", "ga"),
            ("Python Differential Evolution", "python"),
        ]
        
        for text, value in methods:
            tk.Radiobutton(method_frame, text=text, variable=self.opt_method_var,
                          value=value, bg=self.colors['panel'], 
                          fg=self.colors['text'],
                          selectcolor=self.colors['accent']).pack(anchor='w', padx=10, pady=2)
        
        # Results display
        results_frame = tk.LabelFrame(parent, text="Optimization Results",
                                     bg=self.colors['panel'], fg=self.colors['text'])
        results_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        self.opt_results_text = scrolledtext.ScrolledText(
            results_frame,
            font=('Consolas', 10),
            bg='#1e1e1e',
            fg='#d4d4d4',
            height=20
        )
        self.opt_results_text.pack(fill='both', expand=True, padx=5, pady=5)
    
    def _setup_comparison_tab(self, parent):
        """Parameter comparison tab."""
        tk.Label(parent, text="Compare Parameter Sets", 
                font=('Segoe UI', 12, 'bold'),
                bg=self.colors['panel'], fg=self.colors['text']).pack(pady=10)
        
        # Instructions
        info = tk.Label(parent, 
                       text="Add multiple parameter sets to compare performance.",
                       bg=self.colors['panel'], fg=self.colors['warning'])
        info.pack(pady=5)
        
        # Parameter sets list
        list_frame = tk.LabelFrame(parent, text="Parameter Sets",
                                  bg=self.colors['panel'], fg=self.colors['text'])
        list_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.comparison_tree = ttk.Treeview(list_frame, 
                                           columns=('Set', 'Parameters', 'IAE'),
                                           show='headings', height=10)
        self.comparison_tree.heading('Set', text='Set #')
        self.comparison_tree.heading('Parameters', text='Parameters')
        self.comparison_tree.heading('IAE', text='IAE')
        self.comparison_tree.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Buttons
        btn_frame = tk.Frame(parent, bg=self.colors['panel'])
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="➕ Add Current", 
                  command=self.add_current_to_comparison).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="🗑️ Clear All", 
                  command=self.clear_comparison).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="📊 Run Comparison", 
                  command=self.run_comparison).pack(side='left', padx=5)
    
    # Event handlers
    def save_control_logic(self):
        code = self.code_editor.get('1.0', 'end-1c')
        self.algorithm.set_control_logic(code)
        self.update_status("Control logic saved", "success")
        messagebox.showinfo("Success", "Control logic saved!")
    
    def load_control_logic(self):
        file_path = filedialog.askopenfilename(
            title="Load Control Logic",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        if file_path:
            with open(file_path, 'r') as f:
                code = f.read()
            self.code_editor.delete('1.0', 'end')
            self.code_editor.insert('1.0', code)
            self.save_control_logic()
    
    def validate_logic(self):
        code = self.code_editor.get('1.0', 'end-1c')
        try:
            compile(code, '<string>', 'exec')
            self.update_status("Code syntax valid", "success")
            messagebox.showinfo("Validation", "✓ Code syntax is valid!")
        except SyntaxError as e:
            self.update_status(f"Syntax error at line {e.lineno}", "error")
            messagebox.showerror("Syntax Error", f"Line {e.lineno}: {e.msg}")
    
    def add_parameter(self):
        name = self.param_name_entry.get().strip()
        try:
            value = float(self.param_value_entry.get())
            min_val = float(self.param_min_entry.get())
            max_val = float(self.param_max_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers!")
            return
        
        if not name or name in self.algorithm.parameters:
            messagebox.showerror("Error", "Invalid or duplicate parameter name!")
            return
        
        if min_val >= max_val:
            messagebox.showerror("Error", "Min must be less than Max!")
            return
        
        self.algorithm.parameters[name] = value
        self.algorithm.parameter_bounds[name] = (min_val, max_val)
        
        self._create_parameter_row(name, value, min_val, max_val)
        
        # Clear entries
        self.param_name_entry.delete(0, 'end')
        self.param_value_entry.delete(0, 'end')
        self.param_min_entry.delete(0, 'end')
        self.param_max_entry.delete(0, 'end')
        
        self.update_status(f"Added parameter: {name}", "success")
    
    def _create_parameter_row(self, name: str, value: float, 
                             min_val: float, max_val: float):
        row = tk.Frame(self.params_container, bg=self.colors['panel'])
        row.pack(fill='x', pady=2, padx=5)
        
        tk.Label(row, text=f"{name}:", width=12, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text'],
                font=('Segoe UI', 10, 'bold')).pack(side='left')
        
        value_entry = ttk.Entry(row, width=12)
        value_entry.insert(0, str(value))
        value_entry.pack(side='left', padx=5)
        
        tk.Label(row, text=f"[{min_val:.2f}, {max_val:.2f}]",
                bg=self.colors['panel'], fg=self.colors['warning']).pack(side='left', padx=5)
        
        ttk.Button(row, text="❌", width=3,
                  command=lambda: self.remove_parameter(name, row)).pack(side='right')
        
        self.param_widgets[name] = value_entry
    
    def remove_parameter(self, name: str, row: tk.Frame):
        if name in self.algorithm.parameters:
            del self.algorithm.parameters[name]
            del self.algorithm.parameter_bounds[name]
            del self.param_widgets[name]
        row.destroy()
        self.update_status(f"Removed parameter: {name}", "success")
    
    def save_all_parameters(self):
        for name, entry in self.param_widgets.items():
            try:
                self.algorithm.parameters[name] = float(entry.get())
            except ValueError:
                pass
        self.update_status(f"Saved {len(self.param_widgets)} parameters", "success")
        messagebox.showinfo("Success", f"Saved {len(self.param_widgets)} parameters!")
    
    def reset_parameters(self):
        for widget in self.params_container.winfo_children():
            widget.destroy()
        self.param_widgets.clear()
        self.algorithm.parameters.clear()
        self.algorithm.parameter_bounds.clear()
        self.update_status("Parameters reset", "success")
    
    def get_simulation_config(self) -> Dict:
        """Get current simulation configuration."""
        dt = float(self.dt_entry.get())
        steps = int(self.steps_entry.get())
        initial_state = np.array(eval(self.initial_state_entry.get()))
        reference = float(self.reference_entry.get())
        
        # Update time display
        total_time = dt * steps
        self.time_label.config(text=f"Total: {total_time:.1f}s")
        
        # Get plant model
        plant_type = self.plant_var.get()
        
        if plant_type == "double_integrator":
            def plant(state, control, dt):
                state_dot = np.array([state[1], control])
                return state + state_dot * dt
        elif plant_type == "first_order":
            def plant(state, control, dt):
                a, b = 1.0, 1.0
                state_dot = np.array([-a * state[0] + b * control, 0])
                return state + state_dot * dt
        else:  # mass_spring_damper
            def plant(state, control, dt):
                m, b, k = 1.0, 0.5, 1.0
                state_dot = np.array([
                    state[1],
                    (control - b*state[1] - k*state[0]) / m
                ])
                return state + state_dot * dt
        
        return {
            'dt': dt,
            'time_steps': steps,
            'initial_state': initial_state,
            'reference': reference,
            'plant_model': plant
        }
    
    def run_simulation(self):
        """Run simulation with current parameters."""
        self.save_control_logic()
        self.save_all_parameters()
        
        if not self.algorithm.control_code:
            messagebox.showerror("Error", "Please define control logic first!")
            return
        
        if not self.algorithm.parameters:
            messagebox.showerror("Error", "Please define at least one parameter!")
            return
        
        self.update_status("Running simulation...", "warning")
        
        def simulate():
            try:
                config = self.get_simulation_config()
                result = self.algorithm.simulate(
                    config['initial_state'],
                    config['reference'],
                    config['time_steps'],
                    config['dt'],
                    config['plant_model']
                )
                self.simulation_results = [result]
                self.root.after(0, lambda: self.plot_results([result]))
                self.root.after(0, lambda: self.update_status("Simulation complete", "success"))
            except Exception as e:
                trace = traceback.format_exc()
                print(trace)
                self.root.after(0, lambda: self.update_status(f"Error: {str(e)}", "error"))
                self.root.after(0, lambda: messagebox.showerror("Simulation Error", f"{str(e)}\n\nSee console for traceback."))
        
        threading.Thread(target=simulate, daemon=True).start()
    
    def run_optimization(self):
        """Run parameter optimization."""
        self.save_control_logic()
        self.save_all_parameters()
        
        if not self.algorithm.control_code or not self.algorithm.parameters:
            messagebox.showerror("Error", "Define control logic and parameters first!")
            return
        
        self.update_status("Optimizing parameters...", "warning")
        self.opt_results_text.delete('1.0', 'end')
        self.opt_results_text.insert('end', "Starting optimization...\n\n")
        
        def optimize():
            try:
                config = self.get_simulation_config()
                method = self.opt_method_var.get()
                
                if method == "auto":
                    method = "fmincon" if self.optimizer.matlab_available else "python"
                
                self.opt_results_text.insert('end', f"Method: {method}\n")
                self.opt_results_text.insert('end', f"Initial parameters: {self.algorithm.parameters}\n\n")
                
                result = self.optimizer.optimize_parameters(self.algorithm, config, method)
                
                if result:
                    # Update parameters
                    self.algorithm.parameters.update(result['parameters'])
                    
                    # Display results
                    text = "\n=== OPTIMIZATION RESULTS ===\n\n"
                    text += "Optimized Parameters:\n"
                    for name, value in result['parameters'].items():
                        text += f"  {name} = {value:.6f}\n"
                    text += f"\nCost: {result['cost']:.6f}\n"
                    text += "\nPerformance Metrics:\n"
                    for metric, value in result['metrics'].items():
                        text += f"  {metric}: {value:.4f}\n"
                    
                    self.root.after(0, lambda: self.opt_results_text.insert('end', text))
                    self.root.after(0, lambda: self.update_status("Optimization complete!", "success"))
                    
                    # Update parameter displays
                    for name, entry in self.param_widgets.items():
                        if name in result['parameters']:
                            entry.delete(0, 'end')
                            entry.insert(0, f"{result['parameters'][name]:.6f}")
                    
                    # Run simulation with optimized parameters
                    self.root.after(100, self.run_simulation)
                else:
                    self.root.after(0, lambda: self.update_status("Optimization failed", "error"))
                    
            except Exception as e:
                trace = traceback.format_exc()
                print(trace)
                self.root.after(0, lambda: self.opt_results_text.insert('end', f"\nError: {str(e)}\n\n{trace}\n"))
                self.root.after(0, lambda: self.update_status("Optimization error", "error"))
                self.root.after(0, lambda: messagebox.showerror("Optimization Error", f"{str(e)}\n\nSee console for traceback."))
                self.root.after(0, lambda: self.update_status(f"Error: {str(e)}", "error"))
        
        threading.Thread(target=optimize, daemon=True).start()
    
    def add_current_to_comparison(self):
        """Add current parameters to comparison list."""
        params = self.algorithm.parameters.copy()
        if not params:
            messagebox.showerror("Error", "No parameters to add!")
            return
        
        set_num = len(self.comparison_results) + 1
        param_str = ", ".join([f"{k}={v:.2f}" for k, v in params.items()])
        
        self.comparison_tree.insert('', 'end', values=(set_num, param_str, "N/A"))
        self.comparison_results.append({'parameters': params})
        
        self.update_status(f"Added parameter set #{set_num}", "success")
    
    def clear_comparison(self):
        """Clear comparison list."""
        for item in self.comparison_tree.get_children():
            self.comparison_tree.delete(item)
        self.comparison_results.clear()
        self.update_status("Comparison list cleared", "success")
    
    def run_comparison(self):
        """Run comparison of parameter sets."""
        if len(self.comparison_results) < 2:
            messagebox.showerror("Error", "Add at least 2 parameter sets to compare!")
            return
        
        self.update_status("Running comparison...", "warning")
        
        def compare():
            try:
                config = self.get_simulation_config()
                parameter_sets = [r['parameters'] for r in self.comparison_results]
                
                results = compare_parameter_sets(self.algorithm, parameter_sets, config)
                
                # Update tree with results
                for i, result in enumerate(results):
                    item_id = self.comparison_tree.get_children()[i]
                    iae = result['metrics']['iae']
                    self.comparison_tree.set(item_id, 'IAE', f"{iae:.4f}")
                
                # Plot all results
                sim_results = [r['simulation'] for r in results]
                self.root.after(0, lambda: self.plot_results(sim_results))
                self.root.after(0, lambda: self.update_status(f"Compared {len(results)} sets", "success"))
                
            except Exception as e:
                self.root.after(0, lambda: self.update_status(f"Error: {str(e)}", "error"))
                self.root.after(0, lambda: messagebox.showerror("Comparison Error", str(e)))
        
        threading.Thread(target=compare, daemon=True).start()
    
    def plot_results(self, results: List[Dict]):
        """Plot simulation results."""
        if not MATPLOTLIB_AVAILABLE or not results:
            return
        
        # Clear previous plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        colors_list = ['#007acc', '#4ec9b0', '#dcdcaa', '#f48771', '#c586c0']
        
        for i, result in enumerate(results):
            color = colors_list[i % len(colors_list)]
            label = f"Set {i+1}" if len(results) > 1 else "Response"
            
            # Plot response
            self.ax1.plot(result['time'], result['state'][:, 0], 
                         color=color, label=label, linewidth=2)
            self.ax1.axhline(y=result['reference'], color='white', 
                           linestyle='--', alpha=0.5, label='Reference' if i == 0 else '')
            
            # Plot control
            self.ax2.plot(result['time'][1:], result['control'], 
                         color=color, linewidth=2)
            
            # Plot error
            self.ax3.plot(result['time'][1:], result['error'], 
                         color=color, linewidth=2)
        
        # Styling
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_facecolor('#2d2d2d')
            ax.grid(True, alpha=0.2, color='white')
            ax.tick_params(colors='#d4d4d4')
            for spine in ax.spines.values():
                spine.set_color('#d4d4d4')
        
        self.ax1.set_title("System Response", color='#d4d4d4')
        self.ax1.set_ylabel("Position", color='#d4d4d4')
        self.ax1.legend(facecolor='#2d2d2d', edgecolor='#d4d4d4', 
                       labelcolor='#d4d4d4')
        
        self.ax2.set_title("Control Signal", color='#d4d4d4')
        self.ax2.set_ylabel("Control", color='#d4d4d4')
        
        self.ax3.set_title("Tracking Error", color='#d4d4d4')
        self.ax3.set_ylabel("Error", color='#d4d4d4')
        self.ax3.set_xlabel("Time (s)", color='#d4d4d4')
        
        self.fig.tight_layout()
        self.canvas.draw()
    
    def save_results(self):
        """Save results to file."""
        if not self.simulation_results:
            messagebox.showerror("Error", "No results to save!")
            return
        
        file_path = filedialog.asksaveasfilename(
            title="Save Results",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            data = {
                'algorithm_name': self.algorithm.name,
                'control_code': self.algorithm.control_code,
                'parameters': self.algorithm.parameters,
                'parameter_bounds': self.algorithm.parameter_bounds,
                'results': []
            }
            
            for result in self.simulation_results:
                data['results'].append({
                    'time': result['time'].tolist(),
                    'state': result['state'].tolist(),
                    'control': result['control'].tolist(),
                    'error': result['error'].tolist(),
                    'reference': float(result['reference'])
                })
            
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.update_status(f"Results saved to {Path(file_path).name}", "success")
            messagebox.showinfo("Success", f"Results saved to:\n{file_path}")
    
    def load_config(self):
        """Load configuration from file."""
        file_path = filedialog.askopenfilename(
            title="Load Configuration",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                
                # Load control code
                if 'control_code' in data:
                    self.code_editor.delete('1.0', 'end')
                    self.code_editor.insert('1.0', data['control_code'])
                    self.save_control_logic()
                
                # Load parameters
                if 'parameters' in data and 'parameter_bounds' in data:
                    self.reset_parameters()
                    for name, value in data['parameters'].items():
                        bounds = data['parameter_bounds'].get(name, (value*0.1, value*10))
                        self.algorithm.parameters[name] = value
                        self.algorithm.parameter_bounds[name] = bounds
                        self._create_parameter_row(name, value, bounds[0], bounds[1])
                
                self.update_status(f"Loaded config from {Path(file_path).name}", "success")
                messagebox.showinfo("Success", "Configuration loaded!")
                
            except Exception as e:
                messagebox.showerror("Load Error", f"Failed to load configuration:\n{str(e)}")
    
    def update_status(self, message: str, status: str = "info"):
        """Update status label."""
        color_map = {
            "success": self.colors['success'],
            "warning": self.colors['warning'],
            "error": self.colors['error'],
            "info": self.colors['text']
        }
        self.status_label.config(text=f"● {message}", fg=color_map.get(status, self.colors['text']))


if __name__ == "__main__":
    app = ControlDevelopmentGUI()
    app.root.mainloop()
