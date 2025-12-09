import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import io
import sys
import os

# Add parent directory to path to import double_pendulum
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from double_pendulum import DoublePendulum

st.set_page_config(page_title="Double Pendulum - Simulation", layout="wide")

st.title("Interactive Double Pendulum Simulation")
st.markdown("---")

# Sidebar for parameters
st.sidebar.header("Simulation Parameters")

# Physical parameters
st.sidebar.subheader("Physical Parameters")
L1 = st.sidebar.slider("Top Arm Length (L₁, m)", 0.5, 2.0, 1.0, 0.1)
L2 = st.sidebar.slider("Bottom Arm Length (L₂, m)", 0.5, 2.0, 1.0, 0.1)
m1 = st.sidebar.slider("Top Mass (m₁, kg)", 0.1, 5.0, 1.0, 0.1)
m2 = st.sidebar.slider("Bottom Mass (m₂, kg)", 0.1, 5.0, 1.0, 0.1)
g = st.sidebar.slider("Gravity (g, m/s²)", 1.0, 15.0, 9.81, 0.1)

# Initial conditions
st.sidebar.subheader("Initial Conditions")
theta1_0_deg = st.sidebar.slider("Initial Top Angle (θ₁, degrees)", -90, 90, 0, 1)
theta2_0_deg = st.sidebar.slider("Initial Bottom Angle (θ₂, degrees)", -90, 90, 10, 1)
omega1_0 = st.sidebar.slider("Initial Top Angular Velocity (ω₁, rad/s)", -5.0, 5.0, 0.0, 0.1)
omega2_0 = st.sidebar.slider("Initial Bottom Angular Velocity (ω₂, rad/s)", -5.0, 5.0, 0.0, 0.1)

# Simulation parameters
st.sidebar.subheader("Simulation Settings")
t_max = st.sidebar.slider("Simulation Time (s)", 5, 50, 20, 1)
dt = st.sidebar.slider("Time Step (s)", 0.001, 0.05, 0.01, 0.001)

# Convert degrees to radians
theta1_0 = np.deg2rad(theta1_0_deg)
theta2_0 = np.deg2rad(theta2_0_deg)

# Initialize pendulum
pendulum = DoublePendulum(L1=L1, L2=L2, m1=m1, m2=m2, g=g)

# Solve equations
t, solution = pendulum.solve(
    theta1_0=theta1_0,
    theta2_0=theta2_0,
    omega1_0=omega1_0,
    omega2_0=omega2_0,
    t_span=(0, t_max),
    dt=dt
)

theta1 = solution[:, 0]
theta2 = solution[:, 1]
omega1 = solution[:, 2]
omega2 = solution[:, 3]

# Get positions
x1, y1, x2, y2 = pendulum.get_positions(theta1, theta2)

# Get energy
T, V, E = pendulum.get_energy(theta1, theta2, omega1, omega2)

# Main content area
col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("Angular Displacement vs Time")
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(t, np.rad2deg(theta1), 'b-', linewidth=2, label='θ₁ (Top)')
    ax1.plot(t, np.rad2deg(theta2), 'r-', linewidth=2, label='θ₂ (Bottom)')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Angle (degrees)', fontsize=12)
    ax1.set_title('Angular Displacement', fontsize=14, weight='bold')
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)
    st.pyplot(fig1)
    
    st.subheader("Angular Velocity vs Time")
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(t, omega1, 'b-', linewidth=2, label='ω₁ (Top)')
    ax2.plot(t, omega2, 'r-', linewidth=2, label='ω₂ (Bottom)')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax2.set_title('Angular Velocity', fontsize=14, weight='bold')
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)
    st.pyplot(fig2)

with col2:
    st.subheader("Pendulum Trajectory (Top View)")
    fig3, ax3 = plt.subplots(figsize=(10, 10))
    
    # Plot traces
    ax3.plot(x1, y1, 'b-', linewidth=1.5, alpha=0.6, label='Top Mass Trace')
    ax3.plot(x2, y2, 'r-', linewidth=1.5, alpha=0.6, label='Bottom Mass Trace')
    
    # Plot current positions
    ax3.plot(x1[-1], y1[-1], 'bo', markersize=10, label='Top Mass')
    ax3.plot(x2[-1], y2[-1], 'ro', markersize=10, label='Bottom Mass')
    
    # Draw arms
    ax3.plot([0, x1[-1]], [0, y1[-1]], 'b-', linewidth=2, alpha=0.5)
    ax3.plot([x1[-1], x2[-1]], [y1[-1], y2[-1]], 'r-', linewidth=2, alpha=0.5)
    
    # Pivot point
    ax3.plot(0, 0, 'ko', markersize=12, label='Pivot')
    
    ax3.set_xlabel('X Position (m)', fontsize=12)
    ax3.set_ylabel('Y Position (m)', fontsize=12)
    ax3.set_title('Pendulum Trajectory', fontsize=14, weight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect('equal')
    
    # Set reasonable limits
    max_range = max(L1 + L2, np.max(np.abs(x2)), np.max(np.abs(y2))) * 1.2
    ax3.set_xlim(-max_range, max_range)
    ax3.set_ylim(-max_range, max_range)
    
    st.pyplot(fig3)
    
    st.subheader("Energy Conservation")
    fig4, ax4 = plt.subplots(figsize=(10, 6))
    ax4.plot(t, T, 'g-', linewidth=2, label='Kinetic Energy')
    ax4.plot(t, V, 'b-', linewidth=2, label='Potential Energy')
    ax4.plot(t, E, 'r--', linewidth=2, label='Total Energy')
    ax4.set_xlabel('Time (s)', fontsize=12)
    ax4.set_ylabel('Energy (J)', fontsize=12)
    ax4.set_title('Energy vs Time', fontsize=14, weight='bold')
    ax4.legend(fontsize=11)
    ax4.grid(True, alpha=0.3)
    st.pyplot(fig4)

# Phase space plots
st.markdown("---")
st.subheader("Phase Space Diagrams")

col3, col4 = st.columns(2)

with col3:
    fig5, ax5 = plt.subplots(figsize=(10, 6))
    ax5.plot(theta1, omega1, 'b-', linewidth=1.5, alpha=0.7)
    ax5.set_xlabel('θ₁ (rad)', fontsize=12)
    ax5.set_ylabel('ω₁ (rad/s)', fontsize=12)
    ax5.set_title('Top Pendulum Phase Space', fontsize=14, weight='bold')
    ax5.grid(True, alpha=0.3)
    st.pyplot(fig5)

with col4:
    fig6, ax6 = plt.subplots(figsize=(10, 6))
    ax6.plot(theta2, omega2, 'r-', linewidth=1.5, alpha=0.7)
    ax6.set_xlabel('θ₂ (rad)', fontsize=12)
    ax6.set_ylabel('ω₂ (rad/s)', fontsize=12)
    ax6.set_title('Bottom Pendulum Phase Space', fontsize=14, weight='bold')
    ax6.grid(True, alpha=0.3)
    st.pyplot(fig6)

# Information box
st.markdown("---")
st.info(f"""
**Simulation Summary:**
- Initial conditions: θ₁ = {theta1_0_deg}°, θ₂ = {theta2_0_deg}°
- Total simulation time: {t_max} s
- Time step: {dt} s
- Total energy: {E[0]:.4f} J (initial), {E[-1]:.4f} J (final)
- Energy conservation error: {abs(E[-1] - E[0]):.6f} J
""")

