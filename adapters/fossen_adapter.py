"""
Fossen Vehicle Adapter
========================
Wraps the official PythonVehicleSimulator remus100 class for use
with our MPC research framework.

The remus100 is an UNDERACTUATED vehicle (thesis §1.3):
    3 control inputs:  u = [delta_r, delta_s, n]
        delta_r : tail rudder angle [rad]     → controls yaw
        delta_s : stern plane angle [rad]     → controls pitch/depth
        n       : propeller revolution [rpm]  → controls surge speed
    6 DOF state:       η = [x, y, z, φ, θ, ψ], ν = [u, v, w, p, q, r]

This adapter:
    - Creates and configures the remus100 vehicle object
    - Provides a clean run() interface for closed-loop simulation
    - Injects disturbances by modifying current (V_c, beta_c)
    - Extracts M, Minv, and other matrices for NMPC model building
"""

import numpy as np
import math
from dataclasses import dataclass, field
from typing import Callable, Optional, List

# Import from the official PythonVehicleSimulator
from python_vehicle_simulator.vehicles.remus100 import remus100
from python_vehicle_simulator.lib.gnc import attitudeEuler, ssa


@dataclass
class SimulationResult:
    """Container for simulation time-series data."""
    time: np.ndarray             # (T,) time vector [s]
    eta: np.ndarray              # (T, 6) NED pose [x,y,z,φ,θ,ψ]
    nu: np.ndarray               # (T, 6) body velocities [u,v,w,p,q,r]
    u_control: np.ndarray        # (T, 3) commanded controls [δ_r, δ_s, n]
    u_actual: np.ndarray         # (T, 3) actual controls (after actuator dynamics)
    eta_d: np.ndarray            # (T, 6) desired poses
    controller_name: str = ""
    solve_times: List[float] = field(default_factory=list)

    @property
    def n_steps(self):
        return len(self.time)


class FossenVehicleAdapter:
    """
    Adapter wrapping the Fossen PythonVehicleSimulator for MPC research.
    
    Usage:
        adapter = FossenVehicleAdapter()
        vehicle = adapter.vehicle  # access the raw remus100 object
        result = adapter.run(t_final, controller_fn, reference_fn)
    
    Parameters
    ----------
    n_rpm     : cruise propeller RPM (default 1525)
    V_current : ocean current speed [m/s] (default 0)
    beta_current : current direction [deg] (default 0)
    """
    
    def __init__(self, n_rpm: float = 1525, 
                 V_current: float = 0.0, beta_current: float = 0.0):
        
        # Create the official Fossen remus100 vehicle
        self.vehicle = remus100(
            'stepInput',     # we override the controller anyway
            r_z=0, r_psi=0, r_rpm=n_rpm,
            V_current=V_current,
            beta_current=beta_current
        )
        
        self.n_rpm = n_rpm
        self.sampleTime = 0.02   # Fossen default: 50 Hz
        
    @property
    def M(self):
        """Total mass matrix (rigid body + added mass) 6×6."""
        return self.vehicle.M
    
    @property
    def Minv(self):
        """Inverse of total mass matrix 6×6."""
        return self.vehicle.Minv
    
    @property
    def MRB(self):
        """Rigid body mass matrix 6×6."""
        return self.vehicle.MRB
    
    @property
    def MA(self):
        """Added mass matrix 6×6."""
        return self.vehicle.MA
    
    @property
    def W(self):
        """Vehicle weight [N]."""
        return self.vehicle.W
    
    @property
    def B(self):
        """Vehicle buoyancy [N]."""
        return self.vehicle.B
    
    @property
    def actuator_limits(self):
        """Dict of actuator saturation limits."""
        return {
            'delta_r_max': self.vehicle.deltaMax_r,   # 15 deg in rad
            'delta_s_max': self.vehicle.deltaMax_s,   # 15 deg in rad
            'n_max': self.vehicle.nMax,                # 1525 RPM
        }
    
    def set_current(self, V_c: float, beta_c_deg: float):
        """
        Set ocean current (the primary environmental disturbance).
        
        Parameters
        ----------
        V_c       : current speed [m/s]
        beta_c_deg: current direction [deg]
        """
        self.vehicle.V_c = V_c
        self.vehicle.beta_c = beta_c_deg * math.pi / 180
    
    def run(self, t_final: float,
            controller_fn: Callable,
            reference_fn: Callable = None,
            eta0: np.ndarray = None,
            nu0: np.ndarray = None,
            sampleTime: float = None,
            disturbance_fn: Callable = None) -> SimulationResult:
        """
        Run closed-loop simulation with a custom controller.
        
        This replaces Fossen's simulate() function, keeping the
        exact same dynamics but swapping in our controller.
        
        Parameters
        ----------
        t_final       : simulation duration [s]
        controller_fn : callable(eta, nu, eta_d, nu_d, t) → u_control [3,]
                        where u_control = [delta_r, delta_s, n]
        reference_fn  : callable(t) → (eta_d [6,], nu_d [6,])
                        Default: hold current position
        eta0          : initial NED pose [6,] (default: zeros)
        nu0           : initial body velocities [6,] (default: zeros)
        sampleTime    : simulation time step [s] (default: 0.02)
        disturbance_fn: callable(t) → (V_c, beta_c_deg)
                        Time-varying current. Default: use vehicle's current
        
        Returns
        -------
        SimulationResult with full time-series data
        """
        dt = sampleTime or self.sampleTime
        N = int(t_final / dt)
        
        # Initial conditions
        eta = eta0.copy() if eta0 is not None else np.zeros(6, float)
        nu = nu0.copy() if nu0 is not None else np.zeros(6, float)
        u_actual = np.array([0, 0, 0], float)
        
        if reference_fn is None:
            reference_fn = lambda t: (eta.copy(), np.zeros(6))
        
        # Pre-allocate
        time_log = np.zeros(N)
        eta_log = np.zeros((N, 6))
        nu_log = np.zeros((N, 6))
        uc_log = np.zeros((N, 3))
        ua_log = np.zeros((N, 3))
        etad_log = np.zeros((N, 6))
        
        for i in range(N):
            t = i * dt
            
            # Update current if time-varying disturbance
            if disturbance_fn is not None:
                V_c, beta_c_deg = disturbance_fn(t)
                self.vehicle.V_c = V_c
                self.vehicle.beta_c = beta_c_deg * math.pi / 180
            
            # Get reference
            eta_d, nu_d = reference_fn(t)
            
            # Call our custom controller (NOT the vehicle's built-in autopilot)
            u_control = controller_fn(eta, nu, eta_d, nu_d, t)
            
            # Ensure correct shape
            u_control = np.array(u_control[:3], float)
            
            # Log BEFORE dynamics update
            time_log[i] = t
            eta_log[i] = eta.copy()
            nu_log[i] = nu.copy()
            uc_log[i] = u_control.copy()
            ua_log[i] = u_actual.copy()
            etad_log[i] = eta_d.copy()
            
            # Propagate dynamics using the OFFICIAL Fossen model
            [nu, u_actual] = self.vehicle.dynamics(
                eta, nu, u_actual, u_control, dt
            )
            eta = attitudeEuler(eta, nu, dt)
        
        # Collect solve times if controller tracks them
        solve_times = []
        if hasattr(controller_fn, 'solve_times'):
            solve_times = controller_fn.solve_times
        elif hasattr(controller_fn, '__self__') and hasattr(controller_fn.__self__, 'solve_times'):
            solve_times = controller_fn.__self__.solve_times
        
        return SimulationResult(
            time=time_log, eta=eta_log, nu=nu_log,
            u_control=uc_log, u_actual=ua_log,
            eta_d=etad_log,
            controller_name=getattr(controller_fn, 'name', 
                                     getattr(controller_fn, '__name__', 'custom')),
            solve_times=solve_times
        )
    
    def run_builtin_autopilot(self, t_final: float, z_d: float, psi_d: float,
                               n_d: float = 1525, V_c: float = 0.5,
                               beta_c: float = 170,
                               disturbance_fn: Callable = None,
                               reference_fn: Callable = None,
                               eta0: np.ndarray = None,
                               nu0: np.ndarray = None) -> SimulationResult:
        """
        Run with Fossen's built-in depth+heading autopilot for baseline comparison.

        This is useful to verify our controllers against the reference implementation.
        ``disturbance_fn(t) -> (V_c, beta_c_deg)`` optionally applies the same
        time-varying current as run(), so the baseline faces identical conditions.
        ``reference_fn(t) -> (eta_d [6,], nu_d [6,])`` optionally feeds the
        autopilot the same time-varying setpoints the other controllers track
        (the vehicle reads ref_z [m] and ref_psi [deg] each step); z_d/psi_d
        then only seed the initial setpoint.
        ``eta0``/``nu0`` set the initial pose / body velocities (default: zeros,
        i.e. surfaced and at rest).  When ``eta0`` is given, the autopilot's own
        depth LP filter and heading reference model are seeded from it as well,
        so the baseline starts trimmed at that state instead of ramping its
        internal references up from zero.
        """
        # Create a fresh vehicle with the autopilot configured
        vehicle = remus100('depthHeadingAutopilot', z_d, psi_d, n_d, V_c, beta_c)

        dt = self.sampleTime
        N = int(t_final / dt)

        eta = eta0.copy() if eta0 is not None else np.zeros(6, float)
        nu = nu0.copy() if nu0 is not None else vehicle.nu.copy()
        u_actual = vehicle.u_actual.copy()

        if eta0 is not None:
            # Seed the autopilot's internal reference states so it does not fight
            # a spurious 0 -> eta0 transient over the first steps.
            vehicle.z_d = float(eta[2])
            vehicle.psi_d = float(eta[5])

        time_log = np.zeros(N)
        eta_log = np.zeros((N, 6))
        nu_log = np.zeros((N, 6))
        uc_log = np.zeros((N, 3))
        ua_log = np.zeros((N, 3))

        D2R = math.pi / 180
        etad_log = np.tile(np.array([0, 0, z_d, 0, 0, psi_d * D2R]), (N, 1))

        for i in range(N):
            t = i * dt
            if disturbance_fn is not None:
                V_now, beta_now_deg = disturbance_fn(t)
                vehicle.V_c = V_now
                vehicle.beta_c = beta_now_deg * math.pi / 180
            if reference_fn is not None:
                eta_d_t, _ = reference_fn(t)
                vehicle.ref_z = float(eta_d_t[2])
                # The autopilot's 3rd-order reference model tracks the raw
                # difference to ref_psi without angle wrapping, so a wrapped
                # reference (+180 deg emitted as -180) would send it the long
                # way around.  Re-represent the command in the winding nearest
                # the autopilot's internal state psi_d.
                psi_raw = float(eta_d_t[5])
                psi_near = vehicle.psi_d + (
                    (psi_raw - vehicle.psi_d + math.pi) % (2 * math.pi) - math.pi)
                vehicle.ref_psi = psi_near / D2R
                etad_log[i] = eta_d_t
            u_control = vehicle.depthHeadingAutopilot(eta, nu, dt)

            time_log[i] = t
            eta_log[i] = eta.copy()
            nu_log[i] = nu.copy()
            uc_log[i] = u_control.copy()
            ua_log[i] = u_actual.copy()

            [nu, u_actual] = vehicle.dynamics(eta, nu, u_actual, u_control, dt)
            eta = attitudeEuler(eta, nu, dt)
        
        return SimulationResult(
            time=time_log, eta=eta_log, nu=nu_log,
            u_control=uc_log, u_actual=ua_log,
            eta_d=etad_log,
            controller_name="Fossen built-in autopilot"
        )
