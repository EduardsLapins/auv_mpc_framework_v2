"""
Cascaded PID Controller for REMUS 100
========================================
Thesis §1.6.1 — Own PID implementation as baseline.

Architecture (matching standard AUV practice):
    Depth loop:  PI on depth error → desired pitch angle θ_d
                 PID on pitch error → stern plane δ_s
    Heading loop: PID on heading error → rudder δ_r
    Speed: fixed RPM (not controlled)

This is OUR implementation, separate from Fossen's built-in autopilot.
Fossen uses SMC for heading; we use standard PID for a cleaner comparison.
"""

import time
import numpy as np
import math


class PID_REMUS100:
    """
    Cascaded PID for depth + heading control of REMUS 100.
    
    Tuning based on Fossen's vehicle parameters:
        - Yaw time constant T_yaw = 1s
        - Pitch natural frequency w_pitch from restoring moment
        - Depth response dominated by pitch-to-depth coupling
    """
    
    def __init__(self, n_rpm=1525):
        self.n_rpm = n_rpm
        self.name = "PID (custom)"
        self.solve_times = []  # compatibility with NMPC interface
        
        D2R = math.pi / 180
        
        # Actuator limits
        self.delta_r_max = 15 * D2R
        self.delta_s_max = 15 * D2R
        
        # ─── Depth outer loop (PI → θ_d) ────────────────────────
        self.Kp_z = 0.1         # depth proportional gain
        self.Ki_z = 0.005       # depth integral gain
        self.z_int = 0.0        # depth error integral
        self.z_int_max = 20.0   # anti-windup limit
        
        # ─── Pitch inner loop (PID → δ_s) ───────────────────────
        self.Kp_theta = 5.0     # pitch proportional gain
        self.Kd_theta = 2.0     # pitch derivative gain
        self.Ki_theta = 0.3     # pitch integral gain
        self.theta_int = 0.0    # pitch error integral
        self.theta_int_max = 5.0
        self.Kw = 5.0           # heave velocity feedback
        
        # ─── Heading loop (PID → δ_r) ───────────────────────────
        self.Kp_psi = 3.0       # heading proportional gain
        self.Kd_psi = 1.5       # heading derivative gain (yaw rate)
        self.Ki_psi = 0.02      # heading integral gain
        self.psi_int = 0.0      # heading error integral
        self.psi_int_max = 10.0
        
        # Reference smoothing (rate limiter)
        self.psi_d_smooth = 0.0
        self.z_d_smooth = 0.0
        self.r_max = 5.0 * D2R  # max yaw rate for reference [rad/s]
        self.z_rate_max = 2.0   # max depth rate for reference [m/s]
        
        self._initialized = False
    
    def compute(self, eta, nu, eta_d, nu_d, t, dt: float = 0.02):
        """
        Compute [delta_r, delta_s, n] from current state and reference.

        Parameters
        ----------
        eta   : [x, y, z, phi, theta, psi]
        nu    : [u, v, w, p, q, r]
        eta_d : [_, _, z_d, _, _, psi_d]
        dt    : integration step [s] — must match call rate (0.02 at 50 Hz, 0.2 at 5 Hz)

        Returns
        -------
        u_control : [delta_r, delta_s, n_rpm]
        """
        _t0 = time.perf_counter()
        
        z = eta[2]
        theta = eta[4]
        psi = eta[5]
        w = nu[2]      # heave velocity
        q = nu[4]      # pitch rate
        r = nu[5]      # yaw rate
        
        z_ref = eta_d[2]
        psi_ref = eta_d[5]
        
        # ─── Reference smoothing (prevents aggressive transients) ────
        if not self._initialized:
            self.z_d_smooth = z
            self.psi_d_smooth = psi
            self._initialized = True
        
        # Rate-limited depth reference
        z_err_ref = z_ref - self.z_d_smooth
        z_rate = np.clip(z_err_ref, -self.z_rate_max * dt, self.z_rate_max * dt)
        self.z_d_smooth += z_rate
        
        # Rate-limited heading reference
        psi_err_ref = self._ssa(psi_ref - self.psi_d_smooth)
        psi_rate = np.clip(psi_err_ref, -self.r_max * dt, self.r_max * dt)
        self.psi_d_smooth += psi_rate
        
        # ─── DEPTH CONTROL (outer PI → θ_d, inner PID → δ_s) ────
        
        # Outer loop: depth error → desired pitch
        e_z = z - self.z_d_smooth
        self.z_int += dt * e_z
        self.z_int = np.clip(self.z_int, -self.z_int_max, self.z_int_max)
        
        theta_d = self.Kp_z * (e_z + self.Ki_z / self.Kp_z * self.z_int)
        theta_d = np.clip(theta_d, -math.radians(20), math.radians(20))
        
        # Inner loop: pitch error → stern plane
        e_theta = self._ssa(theta - theta_d)
        self.theta_int += dt * e_theta
        self.theta_int = np.clip(self.theta_int, -self.theta_int_max, self.theta_int_max)
        
        delta_s = -(self.Kp_theta * e_theta + 
                     self.Kd_theta * q + 
                     self.Ki_theta * self.theta_int +
                     self.Kw * w)
        
        delta_s = np.clip(delta_s, -self.delta_s_max, self.delta_s_max)
        
        # ─── HEADING CONTROL (PID → δ_r) ────────────────────────
        e_psi = self._ssa(psi - self.psi_d_smooth)
        self.psi_int += dt * e_psi
        self.psi_int = np.clip(self.psi_int, -self.psi_int_max, self.psi_int_max)
        
        delta_r = -(self.Kp_psi * e_psi + 
                     self.Kd_psi * r + 
                     self.Ki_psi * self.psi_int)
        
        delta_r = np.clip(delta_r, -self.delta_r_max, self.delta_r_max)
        
        self.solve_times.append(time.perf_counter() - _t0)
        # Note: Fossen expects [delta_r, delta_s, n]
        return np.array([delta_r, -delta_s, self.n_rpm], float)
    
    def reset(self):
        self.z_int = 0.0
        self.theta_int = 0.0
        self.psi_int = 0.0
        self.psi_d_smooth = 0.0
        self.z_d_smooth = 0.0
        self._initialized = False
    
    @staticmethod
    def _ssa(angle):
        """Smallest signed angle."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
