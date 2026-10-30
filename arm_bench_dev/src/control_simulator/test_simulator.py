#!/usr/bin/env python3
"""
Test script to verify MATLAB engine integration and control algorithms
Run this before using the full GUI to check if everything works
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if __name__ != '__main__':
    # Under pytest: these are hardware/MATLAB integration checks, not unit tests
    import pytest
    pytest.importorskip("matlab.engine", reason="MATLAB Engine for Python not installed")

from control_simulator.matlab_engine_interface import MATLABControlSimulator
from control_simulator.parameter_tuner import ParameterTuner, ParameterBounds
import matplotlib.pyplot as plt
import numpy as np


def test_matlab_connection():
    """Test 1: Can we start MATLAB engine?"""
    print("\n" + "="*60)
    print("TEST 1: MATLAB Engine Connection")
    print("="*60)
    
    try:
        simulator = MATLABControlSimulator()
        print("Starting MATLAB engine (this may take 10-15 seconds)...")
        success = simulator.start_engine()
        
        if success:
            print("✓ MATLAB engine started successfully")
            return simulator
        else:
            print("✗ MATLAB engine failed to start")
            return None
    except Exception as e:
        print(f"✗ Error: {e}")
        return None


def test_pid_simulation(simulator):
    """Test 2: Can we run a PID simulation?"""
    print("\n" + "="*60)
    print("TEST 2: PID Controller Simulation")
    print("="*60)
    
    try:
        parameters = {'Kp': 2.0, 'Ki': 0.1, 'Kd': 0.05}
        print(f"Running PID simulation with parameters: {parameters}")
        
        results = simulator.run_control_simulation(
            algorithm_name='pid',
            parameters=parameters,
            sim_time=10.0,
            reference_signal='step',
            reference_amplitude=1.0
        )
        
        print("✓ Simulation completed successfully")
        print("\nPerformance Metrics:")
        for metric, value in results['metrics'].items():
            print(f"  {metric}: {value:.4f}")
        
        return results
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_lqr_simulation(simulator):
    """Test 3: Can we run an LQR simulation?"""
    print("\n" + "="*60)
    print("TEST 3: LQR Controller Simulation")
    print("="*60)
    
    try:
        parameters = {'Q_pos': 10.0, 'Q_vel': 1.0, 'R': 0.1}
        print(f"Running LQR simulation with parameters: {parameters}")
        
        results = simulator.run_control_simulation(
            algorithm_name='lqr',
            parameters=parameters,
            sim_time=10.0,
            reference_signal='step',
            reference_amplitude=1.0
        )
        
        print("✓ Simulation completed successfully")
        print("\nPerformance Metrics:")
        for metric, value in results['metrics'].items():
            print(f"  {metric}: {value:.4f}")
        
        return results
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_mpc_simulation(simulator):
    """Test 4: Can we run an MPC simulation?"""
    print("\n" + "="*60)
    print("TEST 4: MPC Controller Simulation")
    print("="*60)
    
    try:
        parameters = {'Horizon': 10.0, 'Q_weight': 10.0, 'R_weight': 1.0}
        print(f"Running MPC simulation with parameters: {parameters}")
        
        results = simulator.run_control_simulation(
            algorithm_name='mpc',
            parameters=parameters,
            sim_time=10.0,
            reference_signal='step',
            reference_amplitude=1.0
        )
        
        print("✓ Simulation completed successfully")
        print("\nPerformance Metrics:")
        for metric, value in results['metrics'].items():
            print(f"  {metric}: {value:.4f}")
        
        return results
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_parameter_tuning(simulator):
    """Test 5: Can we auto-tune parameters?"""
    print("\n" + "="*60)
    print("TEST 5: Parameter Auto-Tuning (Quick Test)")
    print("="*60)
    
    try:
        tuner = ParameterTuner(simulator)
        
        param_bounds = [
            ParameterBounds('Kp', 0.5, 5.0, 1.0),
            ParameterBounds('Ki', 0.01, 1.0, 0.1),
            ParameterBounds('Kd', 0.01, 0.5, 0.05)
        ]
        
        print("Tuning PID parameters (20 iterations, may take 2-3 minutes)...")
        print("Objective: Minimize ISE")
        
        results = tuner.tune_parameters(
            algorithm_name='pid',
            parameter_bounds=param_bounds,
            objective='ise',
            method='nelder-mead',  # Faster than differential evolution
            max_iterations=20,
            sim_time=5.0  # Shorter sim time for faster tuning
        )
        
        print("✓ Tuning completed successfully")
        print(f"\nOptimal parameters: {results['optimal_parameters']}")
        print(f"Optimal score (ISE): {results['optimal_score']:.4f}")
        print(f"Iterations: {results['iterations']}")
        
        return results
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def plot_comparison(pid_results, lqr_results, mpc_results):
    """Test 6: Plot comparison of all three controllers"""
    print("\n" + "="*60)
    print("TEST 6: Plotting Comparison")
    print("="*60)
    
    try:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot outputs
        for name, results in [('PID', pid_results), ('LQR', lqr_results), ('MPC', mpc_results)]:
            if results:
                time = np.array(results['time']).flatten()
                states = np.array(results['states']).flatten()
                ax1.plot(time, states, label=name, linewidth=2)
        
        # Reference
        if pid_results:
            time = np.array(pid_results['time']).flatten()
            reference = np.array(pid_results['reference']).flatten()
            ax1.plot(time, reference, 'k--', label='Reference', linewidth=2)
        
        ax1.set_ylabel('Output')
        ax1.set_title('Controller Comparison - Output Response')
        ax1.legend()
        ax1.grid(True)
        
        # Plot control efforts
        for name, results in [('PID', pid_results), ('LQR', lqr_results), ('MPC', mpc_results)]:
            if results:
                time = np.array(results['time']).flatten()
                control = np.array(results['control']).flatten()
                ax2.plot(time, control, label=name, linewidth=2)
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Input')
        ax2.set_title('Controller Comparison - Control Effort')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig('test_comparison.png', dpi=150, bbox_inches='tight')
        print("✓ Comparison plot saved as 'test_comparison.png'")
        
        plt.show()
    except Exception as e:
        print(f"✗ Error: {e}")


def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("CONTROL SIMULATOR TEST SUITE")
    print("="*60)
    print("This will test MATLAB integration and all controllers")
    print("Estimated time: 3-5 minutes")
    
    input("\nPress Enter to start tests...")
    
    # Test 1: MATLAB connection
    simulator = test_matlab_connection()
    if not simulator:
        print("\n✗ FATAL: Cannot proceed without MATLAB engine")
        return
    
    # Test 2-4: Run all controllers
    pid_results = test_pid_simulation(simulator)
    lqr_results = test_lqr_simulation(simulator)
    mpc_results = test_mpc_simulation(simulator)
    
    # Test 5: Parameter tuning
    tuning_results = test_parameter_tuning(simulator)
    
    # Test 6: Visualization
    if pid_results and lqr_results and mpc_results:
        plot_comparison(pid_results, lqr_results, mpc_results)
    
    # Cleanup
    print("\n" + "="*60)
    print("TESTS COMPLETED")
    print("="*60)
    simulator.stop_engine()
    
    # Summary
    print("\nSummary:")
    print(f"  MATLAB Engine: {'✓' if simulator else '✗'}")
    print(f"  PID Simulation: {'✓' if pid_results else '✗'}")
    print(f"  LQR Simulation: {'✓' if lqr_results else '✗'}")
    print(f"  MPC Simulation: {'✓' if mpc_results else '✗'}")
    print(f"  Parameter Tuning: {'✓' if tuning_results else '✗'}")
    
    all_passed = all([simulator, pid_results, lqr_results, mpc_results, tuning_results])
    
    if all_passed:
        print("\n✓ ALL TESTS PASSED - System ready to use!")
        print("\nYou can now run the full GUI:")
        print("  python control_simulator_gui.py")
    else:
        print("\n✗ Some tests failed - please check errors above")
    
    print("\n")


if __name__ == '__main__':
    main()
