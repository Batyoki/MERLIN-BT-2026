"""
Parameter Tuning Module
Automatically tunes control parameters using optimization algorithms
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution
from typing import Dict, List, Tuple, Callable, Optional, Any
import threading
import time
from dataclasses import dataclass

try:
    from .param_parser import parse_control_script
except ImportError:
    from param_parser import parse_control_script


@dataclass
class ParameterBounds:
    """Bounds for a single parameter"""
    name: str
    min_value: float
    max_value: float
    initial_value: float


def bounds_from_script(control_law_text: str) -> List[ParameterBounds]:
    """ParameterBounds for every '@param' declared in a control script (see param_parser.py)."""
    parsed = parse_control_script(control_law_text)
    return [ParameterBounds(p.name, p.min, p.max, p.default) for p in parsed.params]


class ParameterTuner:
    """
    Automatic parameter tuning using optimization algorithms
    """
    
    def __init__(self, simulator, callback=None):
        """
        Initialize parameter tuner
        
        Args:
            simulator: MATLABControlSimulator instance
            callback: Optional callback for progress updates
        """
        self.simulator = simulator
        self.callback = callback
        self.tuning_history = []
        self.best_params = None
        self.best_score = float('inf')
        self.is_tuning = False
        self.stop_requested = False
        
    def tune_parameters(
        self,
        algorithm_name: str,
        parameter_bounds: List[ParameterBounds],
        objective: str = 'ise',
        method: str = 'differential_evolution',
        max_iterations: int = 50,
        sim_time: float = 10.0,
        reference_signal: str = 'step',
        reference_amplitude: float = 1.0,
        control_law_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Tune control parameters to minimize objective function
        
        Args:
            algorithm_name: Control algorithm name
            parameter_bounds: List of ParameterBounds objects
            objective: Objective to minimize ('ise', 'iae', 'itae', 'custom')
            method: Optimization method ('differential_evolution', 'nelder-mead', 'powell')
            max_iterations: Maximum optimization iterations
            sim_time: Simulation duration
            reference_signal: Reference signal type
            reference_amplitude: Reference amplitude
            
        Returns:
            Dictionary with tuning results
        """
        self.is_tuning = True
        self.stop_requested = False
        self.tuning_history = []
        
        # Extract bounds
        bounds = [(pb.min_value, pb.max_value) for pb in parameter_bounds]
        param_names = [pb.name for pb in parameter_bounds]
        initial_guess = [pb.initial_value for pb in parameter_bounds]
        
        # Define objective function
        def objective_function(param_values):
            if self.stop_requested:
                return float('inf')
            
            # Create parameter dictionary
            params = {name: value for name, value in zip(param_names, param_values)}
            
            try:
                # Run simulation
                results = self.simulator.run_control_simulation(
                    algorithm_name=algorithm_name,
                    parameters=params,
                    sim_time=sim_time,
                    reference_signal=reference_signal,
                    reference_amplitude=reference_amplitude,
                    control_law_text=control_law_text
                )
                
                # Extract objective value
                if objective in results['metrics']:
                    score = results['metrics'][objective]
                else:
                    # Custom weighted objective
                    score = self._compute_custom_objective(results['metrics'])
                
                # Handle NaN/Inf
                if not np.isfinite(score):
                    score = 1e6
                
                # Store in history
                self.tuning_history.append({
                    'iteration': len(self.tuning_history) + 1,
                    'parameters': params.copy(),
                    'score': score,
                    'metrics': results['metrics'].copy()
                })
                
                # Update best
                if score < self.best_score:
                    self.best_score = score
                    self.best_params = params.copy()
                
                # Callback
                if self.callback:
                    self.callback({
                        'iteration': len(self.tuning_history),
                        'parameters': params,
                        'score': score,
                        'best_score': self.best_score,
                        'best_params': self.best_params
                    })
                
                return score
                
            except Exception as e:
                print(f"Error in objective function: {e}")
                return 1e6
        
        # Run optimization
        try:
            if method == 'differential_evolution':
                result = differential_evolution(
                    objective_function,
                    bounds=bounds,
                    maxiter=max_iterations,
                    popsize=10,
                    atol=1e-4,
                    tol=1e-4,
                    disp=True
                )
            elif method == 'nelder-mead':
                result = minimize(
                    objective_function,
                    x0=initial_guess,
                    method='Nelder-Mead',
                    options={'maxiter': max_iterations, 'disp': True}
                )
            elif method == 'powell':
                result = minimize(
                    objective_function,
                    x0=initial_guess,
                    method='Powell',
                    bounds=bounds,
                    options={'maxiter': max_iterations, 'disp': True}
                )
            else:
                raise ValueError(f"Unknown optimization method: {method}")
            
            # Extract optimal parameters
            optimal_params = {
                name: value for name, value in zip(param_names, result.x)
            }
            
            tuning_results = {
                'optimal_parameters': optimal_params,
                'optimal_score': result.fun,
                'history': self.tuning_history,
                'success': result.success,
                'message': result.message if hasattr(result, 'message') else 'Completed',
                'iterations': len(self.tuning_history)
            }
            
        except Exception as e:
            print(f"Optimization error: {e}")
            tuning_results = {
                'optimal_parameters': self.best_params if self.best_params else {},
                'optimal_score': self.best_score,
                'history': self.tuning_history,
                'success': False,
                'message': str(e),
                'iterations': len(self.tuning_history)
            }
        
        finally:
            self.is_tuning = False
        
        return tuning_results
    
    def _compute_custom_objective(self, metrics: Dict[str, float]) -> float:
        """
        Compute custom weighted objective combining multiple metrics
        
        Args:
            metrics: Dictionary of performance metrics
            
        Returns:
            Weighted objective score
        """
        # Weights for different objectives (customize as needed)
        weights = {
            'ise': 1.0,
            'overshoot_percent': 0.5,
            'settling_time': 0.3,
            'control_effort': 0.1
        }
        
        score = 0.0
        for metric_name, weight in weights.items():
            if metric_name in metrics:
                value = metrics[metric_name]
                if np.isfinite(value):
                    score += weight * abs(value)
        
        return score
    
    def grid_search(
        self,
        algorithm_name: str,
        parameter_grids: Dict[str, List[float]],
        objective: str = 'ise',
        sim_time: float = 10.0,
        reference_signal: str = 'step',
        reference_amplitude: float = 1.0,
        control_law_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Grid search over parameter space
        
        Args:
            algorithm_name: Control algorithm name
            parameter_grids: Dictionary mapping parameter names to lists of values to try
            objective: Objective to minimize
            sim_time: Simulation duration
            reference_signal: Reference signal type
            reference_amplitude: Reference amplitude
            
        Returns:
            Dictionary with grid search results
        """
        self.is_tuning = True
        self.stop_requested = False
        self.tuning_history = []
        
        # Generate all combinations
        param_names = list(parameter_grids.keys())
        param_values_lists = [parameter_grids[name] for name in param_names]
        
        # Create grid
        from itertools import product
        grid = list(product(*param_values_lists))
        total_combinations = len(grid)
        
        print(f"Grid search: {total_combinations} combinations")
        
        best_score = float('inf')
        best_params = None
        
        for i, param_tuple in enumerate(grid):
            if self.stop_requested:
                break
            
            params = {name: value for name, value in zip(param_names, param_tuple)}
            
            try:
                # Run simulation
                results = self.simulator.run_control_simulation(
                    algorithm_name=algorithm_name,
                    parameters=params,
                    sim_time=sim_time,
                    reference_signal=reference_signal,
                    reference_amplitude=reference_amplitude,
                    control_law_text=control_law_text
                )
                
                score = results['metrics'].get(objective, 1e6)
                
                if not np.isfinite(score):
                    score = 1e6
                
                self.tuning_history.append({
                    'iteration': i + 1,
                    'parameters': params.copy(),
                    'score': score,
                    'metrics': results['metrics'].copy()
                })
                
                if score < best_score:
                    best_score = score
                    best_params = params.copy()
                
                if self.callback:
                    self.callback({
                        'iteration': i + 1,
                        'total': total_combinations,
                        'parameters': params,
                        'score': score,
                        'best_score': best_score,
                        'best_params': best_params
                    })
                
            except Exception as e:
                print(f"Error in grid search iteration {i}: {e}")
        
        self.is_tuning = False
        
        return {
            'optimal_parameters': best_params,
            'optimal_score': best_score,
            'history': self.tuning_history,
            'success': True,
            'iterations': len(self.tuning_history)
        }
    
    def stop_tuning(self):
        """Request to stop tuning"""
        self.stop_requested = True
