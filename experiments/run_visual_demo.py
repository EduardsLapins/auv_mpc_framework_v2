"""
Visual Demo Mode — Using PythonVehicleSimulator's Own Plotting
================================================================
This runs the simulation and generates output using the EXACT SAME
plotting functions that Fossen's main.py uses:
    - plotVehicleStates() — 6-DOF state time series
    - plotControls() — control input time series
    - plot3D() — animated 3D GIF

This proves the framework uses the real PythonVehicleSimulator,
not a reimplementation.

Usage:
    python -m experiments.run_visual_demo
"""

import os, sys, math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_vehicle_simulator.vehicles.remus100 import remus100
from python_vehicle_simulator.lib.mainLoop import simulate
from python_vehicle_simulator.lib.gnc import attitudeEuler
try:
    from python_vehicle_simulator.lib.plotTimeSeries import (
        plotVehicleStates, plotControls, plot3D
    )
    HAS_PLOT = True
except ImportError:
    from python_vehicle_simulator.lib.mainLoop import simulate
    HAS_PLOT = False


def run_fossen_native():
    """
    Run the PythonVehicleSimulator in its NATIVE mode — exactly as
    Fossen's main.py does — using simulate() + plotVehicleStates().
    
    This is the ultimate proof that we're using the real simulator.
    """
    print("\n" + "="*70)
    print("  VISUAL DEMO 1: Fossen simulator native mode")
    print("  Depth=30m, Heading=50deg, RPM=1525, Current=0.5m/s@170deg")
    print("="*70)
    
    # Create vehicle EXACTLY as main.py does for option 9
    vehicle = remus100('depthHeadingAutopilot', 30, 50, 1525, 0.5, 170)
    
    sampleTime = 0.02
    N = 10000  # 200 seconds
    
    print(f"  Vehicle: {vehicle.name}")
    print(f"  Length: {vehicle.L} m")
    print(f"  Control: {vehicle.controlDescription}")
    print(f"  Duration: {N * sampleTime} s at {1/sampleTime} Hz")
    
    # Run using Fossen's own simulate() function
    print("\n  Running Fossen simulate()...")
    [simTime, simData] = simulate(N, sampleTime, vehicle)
    print(f"  Done. {len(simTime)} samples generated.")
    
    # Plot using Fossen's own plotting functions
    if HAS_PLOT:
        print("\n  Generating Fossen's standard plots...")
        plotVehicleStates(simTime, simData, 1)
        plotControls(simTime, simData, vehicle, 2)
        
        # 3D animation
        print("  Generating 3D animation GIF (this takes a moment)...")
        numDataPoints = 50
        FPS = 10
        filename = '3D_animation_fossen.gif'
        plot3D(simData, numDataPoints, FPS, filename, 3)
        print(f"  Saved: {filename}")
        print("  Open the GIF in a browser to see the 3D trajectory.")
    else:
        print("  (plotTimeSeries not available — check PythonVehicleSimulator version)")
    
    return simTime, simData, vehicle


def run_nmpc_with_fossen_plots():
    """
    Run our NMPC controller, then format the data to use Fossen's
    own plotting functions — proving same vehicle, different controller.
    """
    print("\n" + "="*70)
    print("  VISUAL DEMO 2: NMPC on Fossen simulator with native plots")
    print("="*70)
    
    from adapters.fossen_adapter import FossenVehicleAdapter
    from controllers.nmpc_remus import NMPC_REMUS100
    
    D2R = math.pi / 180
    
    # Create NMPC
    vehicle_for_model = remus100('stepInput', V_current=0.5, beta_current=170)
    nmpc = NMPC_REMUS100(vehicle_for_model, N=10, dt_mpc=0.5, n_rpm=1525)
    nmpc.set_current_estimate(0.5, 170 * D2R)
    
    # Smooth reference
    target_z = 30.0
    target_psi = 50.0 * D2R
    def reference_fn(t):
        tau_rise = 10.0
        alpha = 0 if t < 2.0 else 1.0 - math.exp(-(t - 2.0) / tau_rise)
        eta_d = np.array([0, 0, alpha * target_z, 0, 0, alpha * target_psi], float)
        nu_d = np.array([2.5, 0, 0, 0, 0, 0], float)
        return eta_d, nu_d
    
    # Controller wrapper with rate limiting
    last_t = [-1.0]
    last_u = [np.array([0.0, 0.0, 1525.0])]
    def controller_fn(eta, nu, eta_d, nu_d, t):
        if t - last_t[0] >= 0.5 - 0.001:
            last_u[0] = nmpc.compute(eta, nu, eta_d, nu_d, t)
            last_t[0] = t
        return last_u[0]
    
    # Run simulation
    adapter = FossenVehicleAdapter(n_rpm=1525, V_current=0.5, beta_current=170)
    print("  Running NMPC simulation (200s)...")
    result = adapter.run(
        t_final=200.0, controller_fn=controller_fn,
        reference_fn=reference_fn, sampleTime=0.02
    )
    
    # Convert our result format to Fossen's simData format for plotting
    # Fossen simData columns: [eta(6), nu(6), u_control(3), u_actual(3)]
    N_pts = result.n_steps
    simData = np.zeros((N_pts, 18))
    simData[:, 0:6] = result.eta           # eta
    simData[:, 6:12] = result.nu           # nu
    simData[:, 12:15] = result.u_control   # u_control
    simData[:, 15:18] = result.u_actual    # u_actual
    simTime = result.time.reshape(-1, 1)
    
    if HAS_PLOT:
        print("  Generating Fossen's standard plots for NMPC data...")
        # Temporarily modify vehicle name for plot titles
        adapter.vehicle.name = "REMUS 100 with NMPC Controller"
        plotVehicleStates(simTime, simData, 4)
        plotControls(simTime, simData, adapter.vehicle, 5)
        
        print("  Generating 3D animation GIF...")
        filename = '3D_animation_nmpc.gif'
        plot3D(simData, 50, 10, filename, 6)
        print(f"  Saved: {filename}")
    
    print(f"\n  Final depth: {result.eta[-1, 2]:.2f} m (target: 30 m)")
    print(f"  Final heading: {np.degrees(result.eta[-1, 5]):.1f} deg (target: 50 deg)")
    if nmpc.solve_times:
        print(f"  Avg solve time: {1000*np.mean(nmpc.solve_times):.1f} ms")
    
    return result


def run_pid_with_fossen_plots():
    """
    Run our custom PID, then plot with Fossen's functions.
    """
    print("\n" + "="*70)
    print("  VISUAL DEMO 3: Custom PID on Fossen simulator")
    print("="*70)
    
    from adapters.fossen_adapter import FossenVehicleAdapter
    from controllers.pid_remus import PID_REMUS100
    
    D2R = math.pi / 180
    
    pid = PID_REMUS100(n_rpm=1525)
    
    target_z = 30.0
    target_psi = 50.0 * D2R
    def reference_fn(t):
        tau_rise = 10.0
        alpha = 0 if t < 2.0 else 1.0 - math.exp(-(t - 2.0) / tau_rise)
        eta_d = np.array([0, 0, alpha * target_z, 0, 0, alpha * target_psi], float)
        nu_d = np.array([2.5, 0, 0, 0, 0, 0], float)
        return eta_d, nu_d
    
    adapter = FossenVehicleAdapter(n_rpm=1525, V_current=0.5, beta_current=170)
    print("  Running Custom PID simulation (200s)...")
    
    def controller_fn(eta, nu, eta_d, nu_d, t):
        return pid.compute(eta, nu, eta_d, nu_d, t)
    controller_fn.name = pid.name
    controller_fn.solve_times = pid.solve_times
    
    result = adapter.run(
        t_final=200.0, controller_fn=controller_fn,
        reference_fn=reference_fn, sampleTime=0.02
    )
    
    N_pts = result.n_steps
    simData = np.zeros((N_pts, 18))
    simData[:, 0:6] = result.eta
    simData[:, 6:12] = result.nu
    simData[:, 12:15] = result.u_control
    simData[:, 15:18] = result.u_actual
    simTime = result.time.reshape(-1, 1)
    
    if HAS_PLOT:
        print("  Generating Fossen's standard plots for Custom PID...")
        adapter.vehicle.name = "REMUS 100 with Custom PID Controller"
        plotVehicleStates(simTime, simData, 7)
        plotControls(simTime, simData, adapter.vehicle, 8)
        
        filename = '3D_animation_pid.gif'
        plot3D(simData, 50, 10, filename, 9)
        print(f"  Saved: {filename}")
    
    print(f"\n  Final depth: {result.eta[-1, 2]:.2f} m (target: 30 m)")
    print(f"  Final heading: {np.degrees(result.eta[-1, 5]):.1f} deg (target: 50 deg)")
    
    return result


def main():
    import matplotlib
    matplotlib.use('TkAgg')  # interactive display
    import matplotlib.pyplot as plt
    
    print("\n" + "#"*70)
    print("#  VISUAL DEMO — PythonVehicleSimulator Native Plots")
    print("#  Proves we're using the real Fossen dynamics + plotting")
    print("#"*70)
    
    # Demo 1: Fossen native (exactly as main.py)
    run_fossen_native()
    
    # Demo 2: NMPC with Fossen plots
    try:
        run_nmpc_with_fossen_plots()
    except ImportError as e:
        print(f"  NMPC skipped: {e}")
    
    # Demo 3: Custom PID with Fossen plots
    run_pid_with_fossen_plots()
    
    print("\n" + "="*70)
    print("  VISUAL DEMO COMPLETE")
    print("  Check the matplotlib windows and GIF files.")
    print("  The Fossen plots (figures 1-3) should look identical to")
    print("  running 'python3 main.py' and selecting option 9.")
    print("="*70)
    
    plt.show()  # Keep all figures open


if __name__ == '__main__':
    main()
