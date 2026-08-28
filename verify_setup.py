"""Verify setup."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

def main():
    print("\n" + "="*60)
    print("  AUV MPC Framework v2 — Setup Verification")
    print("="*60 + "\n")

    print("1. PythonVehicleSimulator...", end=" ")
    try:
        from python_vehicle_simulator.vehicles.remus100 import remus100
        from python_vehicle_simulator.lib.gnc import ssa, m2c, gvect
        print("OK")
    except ImportError as e:
        print(f"MISSING\n   {e}")
        print("   pip install -e ./PythonVehicleSimulator")
        return

    print("2. Creating REMUS 100...", end=" ")
    v = remus100('stepInput', V_current=0.5, beta_current=170)
    print(f"OK (M shape={v.M.shape})")

    print("3. Adapter test (2s sim)...", end=" ")
    from adapters.fossen_adapter import FossenVehicleAdapter
    a = FossenVehicleAdapter(V_current=0.5, beta_current=170)
    r = a.run_builtin_autopilot(t_final=2.0, z_d=10, psi_d=30, V_c=0.5, beta_c=170)
    print(f"OK ({r.n_steps} steps, depth={r.eta[-1,2]:.3f}m)")

    print("4. CasADi...", end=" ")
    try:
        import casadi
        print(f"OK (v{casadi.__version__})")

        print("5. Reduced CasADi model...", end=" ")
        from adapters.casadi_model import build_casadi_dynamics
        f = build_casadi_dynamics(v)
        # Test: vehicle at surface, slight stern plane deflection, 1000 RPM
        x0 = np.array([0, 0, 0, 1.0, 0, 0], float)  # [z, theta, psi, u, q, r]
        u0 = np.array([-0.05, 0.0, 1000], float)      # [delta_s, delta_r, n]
        xd = np.array(f(x0, u0, 0, 0)).flatten()
        print(f"OK (u_dot={xd[3]:.3f}, q_dot={xd[4]:.4f}, z_dot={xd[0]:.4f})")
        print(f"   Negative delta_s should pitch down (positive q): q_dot={'+ (correct)' if xd[4] < 0 else '- CHECK SIGNS!'}")
        has_casadi = True
    except ImportError:
        print("MISSING — pip install casadi")
        has_casadi = False

    print("\n" + "="*60)
    print("  READY" if has_casadi else "  Partial (install casadi for NMPC)")
    print("="*60)
    if has_casadi:
        print("  Run: python -m experiments.run_remus100_comparison\n")

if __name__ == '__main__':
    main()
