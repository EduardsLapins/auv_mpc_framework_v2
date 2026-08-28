# AUV MPC Research Framework v2

**Built on the official [PythonVehicleSimulator](https://github.com/cybergalactic/PythonVehicleSimulator) by Thor I. Fossen.**

Master's Thesis: MPC Algorithm Development for Autonomous Underwater Drone Control  
Riga Technical University — Eduards Lapiņš, 2026

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│  Experiment Runner (experiments/)                        │
│  Configures scenarios, runs comparisons, generates plots │
├──────────────────────────────────────────────────────────┤
│  Controllers                                             │
│  ┌────────────────────┐  ┌─────────────────────────────┐ │
│  │ Fossen built-in    │  │ NMPC (CasADi/IPOPT)        │ │
│  │ PID/SMC autopilot  │  │ Optimizes [δ_r, δ_s, n]   │ │
│  │ (baseline)         │  │ with Fossen dynamics model  │ │
│  └────────────────────┘  └─────────────────────────────┘ │
├──────────────────────────────────────────────────────────┤
│  Adapter Layer (adapters/)                               │
│  Wraps PythonVehicleSimulator for MPC research           │
│  • FossenVehicleAdapter — simulation loop                │
│  • CasADi model — symbolic dynamics for NMPC optimizer   │
├──────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────┐    │
│  │  PythonVehicleSimulator (Fossen, 2021)           │    │
│  │  remus100.py — REMUS 100 AUV dynamics            │    │
│  │  gnc.py — GNC utilities (Rzyx, m2c, gvect, etc.) │    │
│  │  OFFICIAL, peer-reviewed, published parameters    │    │
│  └──────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

## Setup

```bash
# 1. Clone and install the Fossen simulator (REQUIRED)
git clone https://github.com/cybergalactic/PythonVehicleSimulator.git
pip install -e ./PythonVehicleSimulator

# 2. Install additional dependencies
pip install casadi matplotlib numpy

# 3. Run the experiment
cd auv_mpc_framework_v2
python -m experiments.run_remus100_comparison
```

## What this compares

| Controller | Type | Inputs | Source |
|-----------|------|--------|--------|
| Fossen PID/SMC | Cascaded depth + heading autopilot | δ_r, δ_s, n | Fossen (2021), built-in |
| **NMPC** | Nonlinear receding-horizon optimization | δ_r, δ_s, n | **This thesis** |

Both controllers drive the **same** Fossen remus100 dynamics — the only difference is the control law.

## Key advantage of this approach

The REMUS 100 is **underactuated** (thesis §1.3): 3 inputs controlling 6 DOF.
There is NO sway thruster — lateral motion requires yaw maneuvers.
This is exactly where NMPC shines: it can predict the future trajectory
and plan coordinated rudder/stern-plane/RPM commands that achieve the
desired 3D path despite underactuation.
