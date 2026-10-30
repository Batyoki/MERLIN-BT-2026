"""UI components for control development GUI."""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
from typing import Dict, Any, List
import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class ControlLogicTab:
    """Tab for defining control logic."""
    
    def __init__(self, parent, colors: Dict, algorithm):
        self.parent = parent
        self.colors = colors
        self.algorithm = algorithm
        self.setup()
    
    def setup(self):
        # Title
        title = tk.Label(self.parent, text="Define Control Logic", 
                        font=('Segoe UI', 12, 'bold'),
                        bg=self.colors['panel'], fg=self.colors['text'])
        title.pack(pady=10)
        
        # Instructions
        instructions = tk.Label(self.parent, 
                               text="Write your control algorithm in Python.\n"
                                    "Available variables: state, reference, parameters\n"
                                    "Set control_output variable with your result.",
                               font=('Segoe UI', 9),
                               bg=self.colors['panel'], fg=self.colors['warning'],
                               justify='left')
        instructions.pack(pady=5)
        
        # Code editor
        editor_frame = tk.Frame(self.parent, bg=self.colors['panel'])
        editor_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.code_editor = scrolledtext.ScrolledText(
            editor_frame,
            font=('Consolas', 10),
            bg='#1e1e1e',
            fg='#d4d4d4',
            insertbackground='white',
            selectbackground=self.colors['accent']
        )
        self.code_editor.pack(fill='both', expand=True)
        
        # Default template
        default_code = """# Example: PD Controller
# Parameters: Kp, Kd (define in Parameters tab)

error = reference - state[0]
error_dot = -state[1]  # Assuming state[1] is velocity

control_output = Kp * error + Kd * error_dot

# Ensure output is numpy array
control_output = np.array([control_output])
"""
        self.code_editor.insert('1.0', default_code)
        
        # Buttons
        btn_frame = tk.Frame(self.parent, bg=self.colors['panel'])
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="💾 Save Logic", 
                  command=self.save_logic).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="📂 Load Logic", 
                  command=self.load_logic).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="✓ Validate", 
                  command=self.validate_logic).pack(side='left', padx=5)
    
    def save_logic(self):
        code = self.code_editor.get('1.0', 'end-1c')
        self.algorithm.set_control_logic(code)
        messagebox.showinfo("Success", "Control logic saved!")
    
    def load_logic(self):
        file_path = filedialog.askopenfilename(
            title="Load Control Logic",
            filetypes=[("Python files", "*.py"), ("Text files", "*.txt")]
        )
        if file_path:
            with open(file_path, 'r') as f:
                code = f.read()
            self.code_editor.delete('1.0', 'end')
            self.code_editor.insert('1.0', code)
            self.save_logic()
    
    def validate_logic(self):
        code = self.code_editor.get('1.0', 'end-1c')
        try:
            # Try to compile the code
            compile(code, '<string>', 'exec')
            messagebox.showinfo("Validation", "✓ Code syntax is valid!")
        except SyntaxError as e:
            messagebox.showerror("Syntax Error", f"Line {e.lineno}: {e.msg}")


class ParametersTab:
    """Tab for defining and managing parameters."""
    
    def __init__(self, parent, colors: Dict, algorithm):
        self.parent = parent
        self.colors = colors
        self.algorithm = algorithm
        self.param_entries = {}
        self.bound_entries = {}
        self.setup()
    
    def setup(self):
        # Title
        title = tk.Label(self.parent, text="Control Parameters", 
                        font=('Segoe UI', 12, 'bold'),
                        bg=self.colors['panel'], fg=self.colors['text'])
        title.pack(pady=10)
        
        # Add parameter section
        add_frame = tk.LabelFrame(self.parent, text="Add New Parameter",
                                 bg=self.colors['panel'], fg=self.colors['text'],
                                 font=('Segoe UI', 10, 'bold'))
        add_frame.pack(fill='x', padx=10, pady=5)
        
        row1 = tk.Frame(add_frame, bg=self.colors['panel'])
        row1.pack(pady=5)
        
        tk.Label(row1, text="Name:", bg=self.colors['panel'], 
                fg=self.colors['text']).pack(side='left', padx=5)
        self.param_name_entry = ttk.Entry(row1, width=15)
        self.param_name_entry.pack(side='left', padx=5)
        
        tk.Label(row1, text="Initial Value:", bg=self.colors['panel'], 
                fg=self.colors['text']).pack(side='left', padx=5)
        self.param_value_entry = ttk.Entry(row1, width=15)
        self.param_value_entry.pack(side='left', padx=5)
        
        row2 = tk.Frame(add_frame, bg=self.colors['panel'])
        row2.pack(pady=5)
        
        tk.Label(row2, text="Min:", bg=self.colors['panel'], 
                fg=self.colors['text']).pack(side='left', padx=5)
        self.param_min_entry = ttk.Entry(row2, width=12)
        self.param_min_entry.pack(side='left', padx=5)
        
        tk.Label(row2, text="Max:", bg=self.colors['panel'], 
                fg=self.colors['text']).pack(side='left', padx=5)
        self.param_max_entry = ttk.Entry(row2, width=12)
        self.param_max_entry.pack(side='left', padx=5)
        
        ttk.Button(row2, text="➕ Add Parameter", 
                  command=self.add_parameter).pack(side='left', padx=10)
        
        # Parameters list
        list_frame = tk.LabelFrame(self.parent, text="Current Parameters",
                                  bg=self.colors['panel'], fg=self.colors['text'],
                                  font=('Segoe UI', 10, 'bold'))
        list_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Scrollable frame for parameters
        canvas = tk.Canvas(list_frame, bg=self.colors['panel'], 
                          highlightthickness=0)
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
        
        # Buttons
        btn_frame = tk.Frame(self.parent, bg=self.colors['panel'])
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="💾 Save All", 
                  command=self.save_all_parameters).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="🔄 Reset to Defaults", 
                  command=self.reset_parameters).pack(side='left', padx=5)
    
    def add_parameter(self):
        name = self.param_name_entry.get().strip()
        try:
            value = float(self.param_value_entry.get())
            min_val = float(self.param_min_entry.get())
            max_val = float(self.param_max_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers!")
            return
        
        if not name:
            messagebox.showerror("Error", "Please enter parameter name!")
            return
        
        if min_val >= max_val:
            messagebox.showerror("Error", "Min must be less than Max!")
            return
        
        # Add to algorithm
        self.algorithm.parameters[name] = value
        self.algorithm.parameter_bounds[name] = (min_val, max_val)
        
        # Create UI row
        self._create_parameter_row(name, value, min_val, max_val)
        
        # Clear entries
        self.param_name_entry.delete(0, 'end')
        self.param_value_entry.delete(0, 'end')
        self.param_min_entry.delete(0, 'end')
        self.param_max_entry.delete(0, 'end')
    
    def _create_parameter_row(self, name: str, value: float, 
                             min_val: float, max_val: float):
        row = tk.Frame(self.params_container, bg=self.colors['panel'])
        row.pack(fill='x', pady=2, padx=5)
        
        tk.Label(row, text=f"{name}:", width=15, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text'],
                font=('Segoe UI', 10, 'bold')).pack(side='left')
        
        value_entry = ttk.Entry(row, width=12)
        value_entry.insert(0, str(value))
        value_entry.pack(side='left', padx=5)
        self.param_entries[name] = value_entry
        
        tk.Label(row, text=f"[{min_val:.2f}, {max_val:.2f}]",
                bg=self.colors['panel'], fg=self.colors['warning']).pack(side='left', padx=5)
        
        ttk.Button(row, text="❌", width=3,
                  command=lambda: self.remove_parameter(name, row)).pack(side='right')
    
    def remove_parameter(self, name: str, row: tk.Frame):
        if name in self.algorithm.parameters:
            del self.algorithm.parameters[name]
        if name in self.algorithm.parameter_bounds:
            del self.algorithm.parameter_bounds[name]
        if name in self.param_entries:
            del self.param_entries[name]
        row.destroy()
    
    def save_all_parameters(self):
        for name, entry in self.param_entries.items():
            try:
                value = float(entry.get())
                self.algorithm.parameters[name] = value
            except ValueError:
                pass
        messagebox.showinfo("Success", f"Saved {len(self.param_entries)} parameters!")
    
    def reset_parameters(self):
        for widget in self.params_container.winfo_children():
            widget.destroy()
        self.param_entries.clear()
        self.algorithm.parameters.clear()
        self.algorithm.parameter_bounds.clear()


class SimulationTab:
    """Tab for simulation configuration."""
    
    def __init__(self, parent, colors: Dict):
        self.parent = parent
        self.colors = colors
        self.config = {
            'dt': 0.01,
            'time_steps': 1000,
            'initial_state': np.array([0.0, 0.0]),
            'reference': 1.0
        }
        self.setup()
    
    def setup(self):
        # Title
        title = tk.Label(self.parent, text="Simulation Configuration", 
                        font=('Segoe UI', 12, 'bold'),
                        bg=self.colors['panel'], fg=self.colors['text'])
        title.pack(pady=10)
        
        # Configuration frame
        config_frame = tk.LabelFrame(self.parent, text="Settings",
                                    bg=self.colors['panel'], fg=self.colors['text'],
                                    font=('Segoe UI', 10, 'bold'))
        config_frame.pack(fill='x', padx=10, pady=5)
        
        # Time step
        row1 = tk.Frame(config_frame, bg=self.colors['panel'])
        row1.pack(fill='x', pady=5, padx=10)
        tk.Label(row1, text="Time Step (dt):", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.dt_entry = ttk.Entry(row1, width=20)
        self.dt_entry.insert(0, "0.01")
        self.dt_entry.pack(side='left', padx=5)
        tk.Label(row1, text="seconds", bg=self.colors['panel'], 
                fg=self.colors['warning']).pack(side='left')
        
        # Time steps
        row2 = tk.Frame(config_frame, bg=self.colors['panel'])
        row2.pack(fill='x', pady=5, padx=10)
        tk.Label(row2, text="Number of Steps:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.steps_entry = ttk.Entry(row2, width=20)
        self.steps_entry.insert(0, "1000")
        self.steps_entry.pack(side='left', padx=5)
        
        # Total time display
        self.time_label = tk.Label(row2, text="Total: 10.0s",
                                   bg=self.colors['panel'], fg=self.colors['success'])
        self.time_label.pack(side='left', padx=10)
        
        # Initial state
        row3 = tk.Frame(config_frame, bg=self.colors['panel'])
        row3.pack(fill='x', pady=5, padx=10)
        tk.Label(row3, text="Initial State:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.initial_state_entry = ttk.Entry(row3, width=20)
        self.initial_state_entry.insert(0, "[0.0, 0.0]")
        self.initial_state_entry.pack(side='left', padx=5)
        tk.Label(row3, text="[pos, vel]", bg=self.colors['panel'], 
                fg=self.colors['warning']).pack(side='left')
        
        # Reference
        row4 = tk.Frame(config_frame, bg=self.colors['panel'])
        row4.pack(fill='x', pady=5, padx=10)
        tk.Label(row4, text="Reference Value:", width=20, anchor='w',
                bg=self.colors['panel'], fg=self.colors['text']).pack(side='left')
        self.reference_entry = ttk.Entry(row4, width=20)
        self.reference_entry.insert(0, "1.0")
        self.reference_entry.pack(side='left', padx=5)
        
        # Update button
        ttk.Button(config_frame, text="✓ Update Configuration",
                  command=self.update_config).pack(pady=10)
        
        # Plant model selection
        plant_frame = tk.LabelFrame(self.parent, text="Plant Model",
                                   bg=self.colors['panel'], fg=self.colors['text'],
                                   font=('Segoe UI', 10, 'bold'))
        plant_frame.pack(fill='x', padx=10, pady=5)
        
        self.plant_var = tk.StringVar(value="double_integrator")
        
        models = [
            ("Double Integrator (ẍ = u)", "double_integrator"),
            ("First Order (ẋ = -ax + bu)", "first_order"),
            ("Mass-Spring-Damper", "mass_spring_damper"),
            ("Custom (define in code)", "custom")
        ]
        
        for text, value in models:
            tk.Radiobutton(plant_frame, text=text, variable=self.plant_var,
                          value=value, bg=self.colors['panel'], 
                          fg=self.colors['text'],
                          selectcolor=self.colors['accent']).pack(anchor='w', padx=10, pady=2)
    
    def update_config(self):
        try:
            self.config['dt'] = float(self.dt_entry.get())
            self.config['time_steps'] = int(self.steps_entry.get())
            
            # Parse initial state
            state_str = self.initial_state_entry.get()
            self.config['initial_state'] = np.array(eval(state_str))
            
            self.config['reference'] = float(self.reference_entry.get())
            
            # Update time display
            total_time = self.config['dt'] * self.config['time_steps']
            self.time_label.config(text=f"Total: {total_time:.1f}s")
            
            messagebox.showinfo("Success", "Configuration updated!")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid configuration: {e}")
    
    def get_plant_model(self):
        """Get the plant model function based on selection."""
        model_type = self.plant_var.get()
        
        if model_type == "double_integrator":
            def plant(state, control, dt):
                # ẍ = u
                state_dot = np.array([state[1], control[0]])
                return state + state_dot * dt
            return plant
        
        elif model_type == "first_order":
            def plant(state, control, dt):
                # ẋ = -ax + bu (a=1, b=1)
                state_dot = np.array([-state[0] + control[0], 0])
                return state + state_dot * dt
            return plant
        
        elif model_type == "mass_spring_damper":
            def plant(state, control, dt):
                # mẍ + bẋ + kx = u (m=1, b=0.5, k=1)
                m, b, k = 1.0, 0.5, 1.0
                state_dot = np.array([
                    state[1],
                    (control[0] - b*state[1] - k*state[0]) / m
                ])
                return state + state_dot * dt
            return plant
        
        else:  # custom
            return lambda state, control, dt: state  # Placeholder
