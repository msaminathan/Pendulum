import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

st.set_page_config(page_title="Double Pendulum - Mathematical Details", layout="wide")

st.title("Mathematical Formulation of Double Pendulum")
st.markdown("---")

st.header("1. Coordinate System and Variables")

st.markdown("""
The system is described using two generalized coordinates:
- **θ₁(t)**: Angular position of the top pendulum (measured from vertical)
- **θ₂(t)**: Angular position of the bottom pendulum (measured relative to the top pendulum arm)

The positions of the masses in Cartesian coordinates are:
""")

st.latex(r"""
\begin{align}
x_1 &= L_1 \sin(\theta_1) \\
y_1 &= -L_1 \cos(\theta_1) \\
x_2 &= L_1 \sin(\theta_1) + L_2 \sin(\theta_2) \\
y_2 &= -L_1 \cos(\theta_1) - L_2 \cos(\theta_2)
\end{align}
""")

st.header("2. Lagrangian Formulation")

st.subheader("2.1 Kinetic Energy")

st.markdown("""
The kinetic energy of the system is the sum of kinetic energies of both masses:
""")

st.latex(r"""
T = \frac{1}{2} m_1 (\dot{x}_1^2 + \dot{y}_1^2) + \frac{1}{2} m_2 (\dot{x}_2^2 + \dot{y}_2^2)
""")

st.markdown("""
Taking time derivatives and simplifying:
""")

st.latex(r"""
\begin{align}
\dot{x}_1 &= L_1 \dot{\theta}_1 \cos(\theta_1) \\
\dot{y}_1 &= L_1 \dot{\theta}_1 \sin(\theta_1) \\
\dot{x}_2 &= L_1 \dot{\theta}_1 \cos(\theta_1) + L_2 \dot{\theta}_2 \cos(\theta_2) \\
\dot{y}_2 &= L_1 \dot{\theta}_1 \sin(\theta_1) + L_2 \dot{\theta}_2 \sin(\theta_2)
\end{align}
""")

st.markdown("""
Substituting into the kinetic energy expression:
""")

st.latex(r"""
T = \frac{1}{2} m_1 L_1^2 \dot{\theta}_1^2 + \frac{1}{2} m_2 \left[ L_1^2 \dot{\theta}_1^2 + L_2^2 \dot{\theta}_2^2 + 2 L_1 L_2 \dot{\theta}_1 \dot{\theta}_2 \cos(\theta_1 - \theta_2) \right]
""")

st.latex(r"""
T = \frac{1}{2} (m_1 + m_2) L_1^2 \dot{\theta}_1^2 + \frac{1}{2} m_2 L_2^2 \dot{\theta}_2^2 + m_2 L_1 L_2 \dot{\theta}_1 \dot{\theta}_2 \cos(\theta_1 - \theta_2)
""")

st.subheader("2.2 Potential Energy")

st.markdown("""
The potential energy (taking the pivot point as reference):
""")

st.latex(r"""
V = -m_1 g L_1 \cos(\theta_1) - m_2 g \left[ L_1 \cos(\theta_1) + L_2 \cos(\theta_2) \right]
""")

st.latex(r"""
V = -(m_1 + m_2) g L_1 \cos(\theta_1) - m_2 g L_2 \cos(\theta_2)
""")

st.subheader("2.3 Lagrangian")

st.markdown("""
The Lagrangian is defined as:
""")

st.latex(r"""
\mathcal{L} = T - V
""")

st.latex(r"""
\begin{align}
\mathcal{L} &= \frac{1}{2} (m_1 + m_2) L_1^2 \dot{\theta}_1^2 + \frac{1}{2} m_2 L_2^2 \dot{\theta}_2^2 \\
&\quad + m_2 L_1 L_2 \dot{\theta}_1 \dot{\theta}_2 \cos(\theta_1 - \theta_2) \\
&\quad + (m_1 + m_2) g L_1 \cos(\theta_1) + m_2 g L_2 \cos(\theta_2)
\end{align}
""")

st.header("3. Equations of Motion (Euler-Lagrange Equations)")

st.markdown("""
Applying the Euler-Lagrange equations:
""")

st.latex(r"""
\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}_i} \right) - \frac{\partial \mathcal{L}}{\partial \theta_i} = 0, \quad i = 1, 2
""")

st.markdown("""
This yields the coupled nonlinear differential equations:
""")

st.latex(r"""
\begin{align}
(m_1 + m_2) L_1^2 \ddot{\theta}_1 + m_2 L_1 L_2 \ddot{\theta}_2 \cos(\theta_1 - \theta_2) \\
+ m_2 L_1 L_2 \dot{\theta}_2^2 \sin(\theta_1 - \theta_2) + (m_1 + m_2) g L_1 \sin(\theta_1) &= 0
\end{align}
""")

st.latex(r"""
\begin{align}
m_2 L_2^2 \ddot{\theta}_2 + m_2 L_1 L_2 \ddot{\theta}_1 \cos(\theta_1 - \theta_2) \\
- m_2 L_1 L_2 \dot{\theta}_1^2 \sin(\theta_1 - \theta_2) + m_2 g L_2 \sin(\theta_2) &= 0
\end{align}
""")

st.header("4. Small Angle Approximation")

st.markdown("""
For **small angular displacements**, we can linearize the equations using:
- $\\sin(\\theta) \\approx \\theta$
- $\\cos(\\theta) \\approx 1$
- $\\cos(\\theta_1 - \\theta_2) \\approx 1$
- $\\sin(\\theta_1 - \\theta_2) \\approx 0$
- Terms involving $\\dot{\\theta}^2$ are neglected

The linearized equations become:
""")

st.latex("""
\\begin{aligned}
(m_1 + m_2) L_1^2 \\ddot{\\theta}_1 + m_2 L_1 L_2 \\ddot{\\theta}_2 + (m_1 + m_2) g L_1 \\theta_1 &= 0 \\\\
m_2 L_2^2 \\ddot{\\theta}_2 + m_2 L_1 L_2 \\ddot{\\theta}_1 + m_2 g L_2 \\theta_2 &= 0
\\end{aligned}
""")

st.markdown("""
In matrix form:
""")

st.markdown("""
Matrix form of the linearized system:
""")

st.latex("""
M \\begin{bmatrix} \\ddot{\\theta}_1 \\\\ \\ddot{\\theta}_2 \\end{bmatrix} + K \\begin{bmatrix} \\theta_1 \\\\ \\theta_2 \\end{bmatrix} = \\begin{bmatrix} 0 \\\\ 0 \\end{bmatrix}
""")

st.markdown("""
where the mass matrix $M$ is:
""")

st.latex(r"""
M = \begin{bmatrix}
(m_1 + m_2) L_1^2 & m_2 L_1 L_2 \\
m_2 L_1 L_2 & m_2 L_2^2
\end{bmatrix}
""")

st.markdown("""
and the stiffness matrix $K$ is:
""")

st.latex(r"""
K = \begin{bmatrix}
(m_1 + m_2) g L_1 & 0 \\
0 & m_2 g L_2
\end{bmatrix}
""")

st.header("5. Numerical Solution")

st.markdown("""
For the general (nonlinear) case, we solve the system numerically using:
- **4th order Runge-Kutta method** (RK4) for time integration
- Convert second-order ODEs to first-order system:
  - State vector: $[\\theta_1, \\theta_2, \\dot{\\theta}_1, \\dot{\\theta}_2]^T$
  - Solve for $[\\dot{\\theta}_1, \\dot{\\theta}_2, \\ddot{\\theta}_1, \\ddot{\\theta}_2]^T$

The system is solved by inverting the mass matrix:
""")

st.markdown("""
The accelerations are obtained by inverting the mass matrix:
""")

st.latex("""
\\begin{bmatrix} \\ddot{\\theta}_1 \\\\ \\ddot{\\theta}_2 \\end{bmatrix} = M^{-1} \\begin{bmatrix} C_1 \\\\ C_2 \\end{bmatrix}
""")

st.markdown("""
where the force vector components are:
""")

st.latex("""
C_1 = -m_2 L_1 L_2 \\dot{\\theta}_2^2 \\sin(\\theta_1 - \\theta_2) - (m_1 + m_2) g L_1 \\sin(\\theta_1)
""")

st.latex("""
C_2 = m_2 L_1 L_2 \\dot{\\theta}_1^2 \\sin(\\theta_1 - \\theta_2) - m_2 g L_2 \\sin(\\theta_2)
""")

st.markdown("""
where $M$ is the mass matrix:
""")

st.latex("""
M = \\begin{bmatrix}
(m_1 + m_2) L_1^2 & m_2 L_1 L_2 \\cos(\\theta_1 - \\theta_2) \\\\
m_2 L_1 L_2 \\cos(\\theta_1 - \\theta_2) & m_2 L_2^2
\\end{bmatrix}
""")

st.markdown("---")
st.info("💡 The simulation page uses these equations to compute the motion numerically!")

