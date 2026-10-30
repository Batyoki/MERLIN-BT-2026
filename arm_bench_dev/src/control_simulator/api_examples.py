#!/usr/bin/env python3
"""
Example: Using Control Simulator API Programmatically
Demonstrates how to use the simulator without the GUI
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from arm_bench_dev.src.control_simulator.matlab_engine_interface import MATLABControlSimulator
from arm_bench_dev.src.control_simulator.parameter_tuner import ParameterTuner, ParameterBounds
import matplotlib.pyplot as plt
import numpy as np


def example_1_simple_simulation():
    """Example 1: Run a simple PID simulation"""
    print("\n" + "="*60)
    print("EXAMPLE 1: Simple PID Simulation")
    print("="*60)
    
    # Initialize simulator
    simulator = MATLABControlSimulator()
    simulator.start_engine()
    
    # Define PID parameters
    pid_params = {
        'Kp': 2.0,
        'Ki': 0.5,
        'Kd': 0.1
    }
    
    # Run simulation
    results = simulator.run_control_simulation(
        algorithm_name='pid',
        parameters=pid_params,
        sim_time=10.0,
        reference_signal='step',
        reference_amplitude=1.0
    )
    
    # Print metrics
    print("\nPerformance Metrics:")
    for metric, value in results['metrics'].items():
        print(f"  {metric}: {value:.4f}")
    
    # Plot
    time = np.array(results['time']).flatten()
    output = np.array(results['states']).flatten()
    reference = np.array(results['reference']).flatten()
    
    plt.figure(figsize=(10, 5))
    plt.plot(time, reference, 'r--', label='Reference', linewidth=2)
    plt.plot(time, output, 'b-', label='Output', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('PID Controller Response')
    plt.legend()
    plt.grid(True)
    plt.savefig('example1_pid_response.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved as: example1_pid_response.png")
    
    simulator.stop_engine()


def example_2_parameter_tuning():
    """Example 2: Automatic parameter tuning"""
    print("\n" + "="*60)
    print("EXAMPLE 2: Automatic Parameter Tuning")
    print("="*60)
    
    # Initialize
    simulator = MATLABControlSimulator()
    simulator.start_engine()
    tuner = ParameterTuner(simulator)
    
    # Define bounds
    param_bounds = [
        ParameterBounds('Kp', min_value=0.5, max_value=5.0, initial_value=1.0),
        ParameterBounds('Ki', min_value=0.01, max_value=1.0, initial_value=0.1),
        ParameterBounds('Kd', min_value=0.01, max_value=0.5, initial_value=0.05)
    ]
    
    # Tune
    print("Tuning PID parameters (this may take 2-3 minutes)...")
    results = tuner.tune_parameters(
        algorithm_name='pid',
        parameter_bounds=param_bounds,
        objective='ise',
        method='nelder-mead',
        max_iterations=30,
        sim_time=5.0
    )
    
    print("\nOptimal Parameters:")
    for param, value in results['optimal_parameters'].items():
        print(f"  {param}: {value:.4f}")
    print(f"\nOptimal ISE: {results['optimal_score']:.4f}")
    
    # Plot tuning history
    history = results['history']
    iterations = [h['iteration'] for h in history]
    scores = [h['score'] for h in history]
    
    plt.figure(figsize=(10, 5))
    plt.plot(iterations, scores, 'bo-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('ISE')
    plt.title('Parameter Tuning Convergence')
    plt.grid(True)
    plt.savefig('example2_tuning_history.png', dpi=150, bbox_inches='tight')
    print("Plot saved as: example2_tuning_history.png")
    
    simulator.stop_engine()


def example_3_controller_comparison():
    """Example 3: Compare PID, LQR, and MPC"""
    print("\n" + "="*60)
    print("EXAMPLE 3: Controller Comparison")
    print("="*60)
    
    # Initialize
    simulator = MATLABControlSimulator()
    simulator.start_engine()
    
    # Define parameters for each controller
    controllers = {
        'PID': {'Kp': 2.0, 'Ki': 0.5, 'Kd': 0.1},
        'LQR': {'Q_pos': 50.0, 'Q_vel': 5.0, 'R': 0.5},
        'MPC': {'Horizon': 10.0, 'Q_weight': 10.0, 'R_weight': 1.0}
    }
    
    results_dict = {}
    
    # Run simulations
    for name, params in controllers.items():
        print(f"\nRunning {name} simulation...")
        results = simulator.run_control_simulation(
            algorithm_name=name.lower(),
            parameters=params,
            sim_time=10.0
        )
        results_dict[name] = results
        
        print(f"  Rise time: {results['metrics']['rise_time']:.3f}s")
        print(f"  Overshoot: {results['metrics']['overshoot_percent']:.1f}%")
        print(f"  ISE: {results['metrics']['ise']:.3f}")
    
    # Plot comparison
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    for name, results in results_dict.items():
        time = np.array(results['time']).flatten()
        output = np.array(results['states']).flatten()
        control = np.array(results['control']).flatten()
        
        ax1.plot(time, output, label=name, linewidth=2)
        ax2.plot(time, control, label=name, linewidth=2)
    
    # Reference on first plot
    time = np.array(results_dict['PID']['time']).flatten()
    reference = np.array(results_dict['PID']['reference']).flatten()
    ax1.plot(time, reference, 'k--', label='Reference', linewidth=2)
    
    ax1.set_ylabel('Output')
    ax1.set_title('Controller Comparison - Output Response')
    ax1.legend()
    ax1.grid(True)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Input')
    ax2.set_title('Controller Comparison - Control Effort')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('example3_comparison.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved as: example3_comparison.png")
    
    simulator.stop_engine()


def example_4_grid_search():
    """Example 4: Grid search over parameter space"""
    print("\n" + "="*60)
    print("EXAMPLE 4: Grid Search")
    print("="*60)
    
    # Initialize
    simulator = MATLABControlSimulator()
    simulator.start_engine()
    tuner = ParameterTuner(simulator)
    
    # Define grid
    parameter_grids = {
        'Kp': [0.5, 1.0, 2.0, 3.0],
        'Ki': [0.1, 0.3, 0.5],
        'Kd': [0.05, 0.1, 0.2]
    }
    
    print(f"Grid search: {4*3*3} = 36 combinations")
    print("This will take 2-3 minutes...")
    
    # Run grid search
    results = tuner.grid_search(
        algorithm_name='pid',
        parameter_grids=parameter_grids,
        objective='ise',
        sim_time=5.0
    )
    
    print("\nOptimal Parameters:")
    for param, value in results['optimal_parameters'].items():
        print(f"  {param}: {value:.4f}")
    print(f"\nOptimal ISE: {results['optimal_score']:.4f}")
    
    # Visualize grid search results
    history = results['history']
    scores = [h['score'] for h in history]
    
    plt.figure(figsize=(10, 6))
    plt.plot(scores, 'o-', linewidth=2)
    plt.xlabel('Parameter Combination')
    plt.ylabel('ISE')
    plt.title('Grid Search Results')
    plt.grid(True)
    
    # Mark best
    best_idx = np.argmin(scores)
    plt.plot(best_idx, scores[best_idx], 'r*', markersize=20, label='Best')
    plt.legend()
    
    plt.savefig('example4_grid_search.png', dpi=150, bbox_inches='tight')
    print("Plot saved as: example4_grid_search.png")
    
    simulator.stop_engine()


def example_5_batch_simulations():
    """Example 5: Batch simulations with different settings"""
    print("\n" + "="*60)
    print("EXAMPLE 5: Batch Simulations")
    print("="*60)
    
    # Initialize
    simulator = MATLABControlSimulator()
    simulator.start_engine()
    
    # Test different reference signals
    reference_signals = ['step', 'ramp', 'sine']
    pid_params = {'Kp': 2.0, 'Ki': 0.5, 'Kd': 0.1}
    
    fig, axes = plt.subplots(len(reference_signals), 1, figsize=(10, 10))
    
    for i, ref_signal in enumerate(reference_signals):
        print(f"\nRunning simulation with {ref_signal} reference...")
        
        results = simulator.run_control_simulation(
            algorithm_name='pid',
            parameters=pid_params,
            sim_time=10.0,
            reference_signal=ref_signal,
            reference_amplitude=1.0
        )
        
        time = np.array(results['time']).flatten()
        output = np.array(results['states']).flatten()
        reference = np.array(results['reference']).flatten()
        
        axes[i].plot(time, reference, 'r--', label='Reference', linewidth=2)
        axes[i].plot(time, output, 'b-', label='Output', linewidth=2)
        axes[i].set_ylabel('Position')
        axes[i].set_title(f'PID Response - {ref_signal.capitalize()} Reference')
        axes[i].legend()
        axes[i].grid(True)
        
        print(f"  RMS Error: {results['metrics']['rms_error']:.4f}")
    
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig('example5_batch_simulations.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved as: example5_batch_simulations.png")
    
    simulator.stop_engine()


def main():
    """Run all examples"""
    print("\n" + "="*70)
    print("CONTROL SIMULATOR API EXAMPLES")
    print("="*70)
    print("\nThis script demonstrates programmatic use of the simulator")
    print("without the GUI. Each example is self-contained.\n")
    
    examples = [
        ("Simple PID Simulation", example_1_simple_simulation),
        ("Automatic Parameter Tuning", example_2_parameter_tuning),
        ("Controller Comparison", example_3_controller_comparison),
        ("Grid Search", example_4_grid_search),
        ("Batch Simulations", example_5_batch_simulations)
    ]
    
    print("Available examples:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    print("  6. Run all examples")
    
    try:
        choice = input("\nSelect example to run (1-6): ").strip()
        
        if choice == '6':
            # Run all
            for name, func in examples:
                print(f"\n{'='*70}")
                print(f"Running: {name}")
                print('='*70)
                func()
        elif choice.isdigit() and 1 <= int(choice) <= 5:
            # Run selected
            name, func = examples[int(choice) - 1]
            func()
        else:
            print("Invalid choice. Exiting.")
            return
        
        print("\n" + "="*70)
        print("EXAMPLES COMPLETED")
        print("="*70)
        print("\nGenerated files:")
        print("  - example1_pid_response.png")
        print("  - example2_tuning_history.png")
        print("  - example3_comparison.png")
        print("  - example4_grid_search.png")
        print("  - example5_batch_simulations.png")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
