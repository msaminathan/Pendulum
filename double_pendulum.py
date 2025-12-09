"""
Double Pendulum Physics Simulation
Implements Lagrangian mechanics for double pendulum system
"""

import numpy as np
from scipy.integrate import odeint


class DoublePendulum:
    """
    Double pendulum system with point masses at the ends of massless rods.
    """
    
    def __init__(self, L1=1.0, L2=1.0, m1=1.0, m2=1.0, g=9.81):
        """
        Initialize double pendulum parameters.
        
        Parameters:
        -----------
        L1 : float
            Length of top pendulum arm (m)
        L2 : float
            Length of bottom pendulum arm (m)
        m1 : float
            Mass of top pendulum bob (kg)
        m2 : float
            Mass of bottom pendulum bob (kg)
        g : float
            Acceleration due to gravity (m/s²)
        """
        self.L1 = L1
        self.L2 = L2
        self.m1 = m1
        self.m2 = m2
        self.g = g
    
    def equations(self, state, t):
        """
        System of differential equations for double pendulum.
        
        Parameters:
        -----------
        state : array
            [theta1, theta2, omega1, omega2]
        t : float
            Time
        
        Returns:
        --------
        dstate_dt : array
            [dtheta1/dt, dtheta2/dt, domega1/dt, domega2/dt]
        """
        theta1, theta2, omega1, omega2 = state
        
        # Mass matrix elements
        M11 = (self.m1 + self.m2) * self.L1**2
        M12 = self.m2 * self.L1 * self.L2 * np.cos(theta1 - theta2)
        M21 = M12
        M22 = self.m2 * self.L2**2
        
        # Right-hand side vector
        C1 = -self.m2 * self.L1 * self.L2 * omega2**2 * np.sin(theta1 - theta2) - (self.m1 + self.m2) * self.g * self.L1 * np.sin(theta1)
        C2 = self.m2 * self.L1 * self.L2 * omega1**2 * np.sin(theta1 - theta2) - self.m2 * self.g * self.L2 * np.sin(theta2)
        
        # Invert mass matrix
        det = M11 * M22 - M12 * M21
        if abs(det) < 1e-10:
            # Handle singular case
            alpha1 = 0
            alpha2 = 0
        else:
            alpha1 = (M22 * C1 - M12 * C2) / det
            alpha2 = (-M21 * C1 + M11 * C2) / det
        
        return [omega1, omega2, alpha1, alpha2]
    
    def solve(self, theta1_0, theta2_0, omega1_0=0.0, omega2_0=0.0, t_span=None, dt=0.01):
        """
        Solve the double pendulum equations of motion.
        
        Parameters:
        -----------
        theta1_0 : float
            Initial angle of top pendulum (radians)
        theta2_0 : float
            Initial angle of bottom pendulum (radians)
        omega1_0 : float
            Initial angular velocity of top pendulum (rad/s)
        omega2_0 : float
            Initial angular velocity of bottom pendulum (rad/s)
        t_span : tuple or None
            (t_start, t_end) time span. If None, uses (0, 20)
        dt : float
            Time step for solution
        
        Returns:
        --------
        t : array
            Time points
        solution : array
            Solution array with shape (n_time, 4)
            Columns: [theta1, theta2, omega1, omega2]
        """
        if t_span is None:
            t_span = (0, 20)
        
        t = np.arange(t_span[0], t_span[1], dt)
        initial_state = [theta1_0, theta2_0, omega1_0, omega2_0]
        
        solution = odeint(self.equations, initial_state, t)
        
        return t, solution
    
    def get_positions(self, theta1, theta2):
        """
        Convert angular positions to Cartesian coordinates.
        
        Parameters:
        -----------
        theta1 : array or float
            Angle of top pendulum
        theta2 : array or float
            Angle of bottom pendulum
        
        Returns:
        --------
        x1, y1 : arrays or floats
            Position of top mass
        x2, y2 : arrays or floats
            Position of bottom mass
        """
        x1 = self.L1 * np.sin(theta1)
        y1 = -self.L1 * np.cos(theta1)
        x2 = x1 + self.L2 * np.sin(theta2)
        y2 = y1 - self.L2 * np.cos(theta2)
        
        return x1, y1, x2, y2
    
    def get_energy(self, theta1, theta2, omega1, omega2):
        """
        Calculate total energy (kinetic + potential).
        
        Parameters:
        -----------
        theta1, theta2 : arrays or floats
            Angular positions
        omega1, omega2 : arrays or floats
            Angular velocities
        
        Returns:
        --------
        T : array or float
            Kinetic energy
        V : array or float
            Potential energy
        E : array or float
            Total energy
        """
        # Kinetic energy
        T = 0.5 * (self.m1 + self.m2) * self.L1**2 * omega1**2 + \
            0.5 * self.m2 * self.L2**2 * omega2**2 + \
            self.m2 * self.L1 * self.L2 * omega1 * omega2 * np.cos(theta1 - theta2)
        
        # Potential energy
        V = -(self.m1 + self.m2) * self.g * self.L1 * np.cos(theta1) - \
            self.m2 * self.g * self.L2 * np.cos(theta2)
        
        E = T + V
        
        return T, V, E

