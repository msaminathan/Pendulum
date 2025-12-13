import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import sys
import os
import tempfile
import hashlib

# Add parent directory to path to import double_pendulum
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from double_pendulum import DoublePendulum

st.set_page_config(page_title="Double Pendulum - Nonlinear Dynamics", layout="wide")

st.title("Full Nonlinear Dynamics (No Small Angle Approximation)")
st.markdown("---")

st.markdown("""
This page demonstrates the **complete nonlinear dynamics** of the double pendulum system 
without any approximations. The equations include all nonlinear terms, making the system 
exhibit chaotic behavior for large initial angles.
""")

st.header("1. Complete Nonlinear Equations of Motion")

st.markdown("""
The full nonlinear equations of motion derived from Lagrangian mechanics are:
""")

st.latex("""
\\begin{align}
(m_1 + m_2) L_1^2 \\ddot{\\theta}_1 + m_2 L_1 L_2 \\ddot{\\theta}_2 \\cos(\\theta_1 - \\theta_2) \\\\
+ m_2 L_1 L_2 \\dot{\\theta}_2^2 \\sin(\\theta_1 - \\theta_2) + (m_1 + m_2) g L_1 \\sin(\\theta_1) &= 0
\\end{align}
""")

st.latex("""
\\begin{align}
m_2 L_2^2 \\ddot{\\theta}_2 + m_2 L_1 L_2 \\ddot{\\theta}_1 \\cos(\\theta_1 - \\theta_2) \\\\
- m_2 L_1 L_2 \\dot{\\theta}_1^2 \\sin(\\theta_1 - \\theta_2) + m_2 g L_2 \\sin(\\theta_2) &= 0
\\end{align}
""")

st.markdown("""
**Key differences from linearized equations:**
- All trigonometric functions are kept: $\\sin(\\theta)$ and $\\cos(\\theta)$ (not approximated)
- Coupling terms: $\\cos(\\theta_1 - \\theta_2)$ and $\\sin(\\theta_1 - \\theta_2)$ are fully nonlinear
- Centrifugal terms: $\\dot{\\theta}_1^2$ and $\\dot{\\theta}_2^2$ are included
- No small angle assumptions: $\\sin(\\theta) \\neq \\theta$ for large angles
""")

st.header("2. Matrix Form of Nonlinear Equations")

st.markdown("""
The system can be written in matrix form as:
""")

st.latex("""
M(\\theta_1, \\theta_2) \\begin{bmatrix} \\ddot{\\theta}_1 \\\\ \\ddot{\\theta}_2 \\end{bmatrix} = 
\\begin{bmatrix} C_1(\\theta_1, \\theta_2, \\dot{\\theta}_1, \\dot{\\theta}_2) \\\\ C_2(\\theta_1, \\theta_2, \\dot{\\theta}_1, \\dot{\\theta}_2) \\end{bmatrix}
""")

st.markdown("""
where the **mass matrix** $M$ depends on the angles:
""")

st.latex("""
M(\\theta_1, \\theta_2) = \\begin{bmatrix}
(m_1 + m_2) L_1^2 & m_2 L_1 L_2 \\cos(\\theta_1 - \\theta_2) \\\\
m_2 L_1 L_2 \\cos(\\theta_1 - \\theta_2) & m_2 L_2^2
\\end{bmatrix}
""")

st.markdown("""
and the **force vector** $C$ contains all nonlinear terms:
""")

st.latex("""
\\begin{align}
C_1 &= -m_2 L_1 L_2 \\dot{\\theta}_2^2 \\sin(\\theta_1 - \\theta_2) - (m_1 + m_2) g L_1 \\sin(\\theta_1) \\\\
C_2 &= m_2 L_1 L_2 \\dot{\\theta}_1^2 \\sin(\\theta_1 - \\theta_2) - m_2 g L_2 \\sin(\\theta_2)
\\end{align}
""")

st.markdown("""
The accelerations are obtained by inverting the mass matrix:
""")

st.latex("""
\\begin{bmatrix} \\ddot{\\theta}_1 \\\\ \\ddot{\\theta}_2 \\end{bmatrix} = M^{-1}(\\theta_1, \\theta_2) 
\\begin{bmatrix} C_1 \\\\ C_2 \\end{bmatrix}
""")

st.markdown("---")

st.header("3. Comparison: Small vs Large Angle Motion")

col1, col2 = st.columns(2)

with col1:
    st.subheader("Small Angle Approximation")
    st.markdown("""
    For small angles ($|\\theta| < 15°$):
    - Motion is **periodic** and **predictable**
    - System exhibits **normal modes**
    - Energy is approximately conserved
    - Trajectories are smooth ellipses in phase space
    """)
    
    # Small angle simulation
    st.markdown("**Example: Small Initial Angles**")
    pendulum_small = DoublePendulum(L1=1.0, L2=1.0, m1=1.0, m2=1.0, g=9.81)
    t_small, sol_small = pendulum_small.solve(
        theta1_0=np.deg2rad(5),
        theta2_0=np.deg2rad(10),
        t_span=(0, 10),
        dt=0.01
    )
    
    fig_small, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Angular displacement
    ax1.plot(t_small, np.rad2deg(sol_small[:, 0]), 'b-', linewidth=2, label='θ₁')
    ax1.plot(t_small, np.rad2deg(sol_small[:, 1]), 'r-', linewidth=2, label='θ₂')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (degrees)')
    ax1.set_title('Small Angle Motion (θ₁₀=5°, θ₂₀=10°)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Phase space
    ax2.plot(sol_small[:, 0], sol_small[:, 2], 'b-', linewidth=1.5, alpha=0.7, label='Top')
    ax2.plot(sol_small[:, 1], sol_small[:, 3], 'r-', linewidth=1.5, alpha=0.7, label='Bottom')
    ax2.set_xlabel('Angle (rad)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Phase Space Trajectory')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    st.pyplot(fig_small)

with col2:
    st.subheader("Large Angle (Nonlinear)")
    st.markdown("""
    For large angles ($|\\theta| > 30°$):
    - Motion can be **chaotic** and **unpredictable**
    - Sensitive dependence on initial conditions
    - Complex phase space trajectories
    - Energy still conserved (numerically)
    """)
    
    # Large angle simulation
    st.markdown("**Example: Large Initial Angles**")
    pendulum_large = DoublePendulum(L1=1.0, L2=1.0, m1=1.0, m2=1.0, g=9.81)
    t_large, sol_large = pendulum_large.solve(
        theta1_0=np.deg2rad(45),
        theta2_0=np.deg2rad(60),
        t_span=(0, 10),
        dt=0.01
    )
    
    fig_large, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Angular displacement
    ax1.plot(t_large, np.rad2deg(sol_large[:, 0]), 'b-', linewidth=2, label='θ₁')
    ax1.plot(t_large, np.rad2deg(sol_large[:, 1]), 'r-', linewidth=2, label='θ₂')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (degrees)')
    ax1.set_title('Large Angle Motion (θ₁₀=45°, θ₂₀=60°)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Phase space
    ax2.plot(sol_large[:, 0], sol_large[:, 2], 'b-', linewidth=1.5, alpha=0.7, label='Top')
    ax2.plot(sol_large[:, 1], sol_large[:, 3], 'r-', linewidth=1.5, alpha=0.7, label='Bottom')
    ax2.set_xlabel('Angle (rad)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Phase Space Trajectory (Chaotic)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    st.pyplot(fig_large)

st.markdown("---")

st.header("4. Nonlinear Terms Analysis")

st.subheader("4.1 Centrifugal Forces")
st.markdown("""
The terms $\\dot{\\theta}_1^2$ and $\\dot{\\theta}_2^2$ represent **centrifugal forces**:
- These arise from the rotating reference frame
- They become significant when angular velocities are large
- They couple the two pendulums nonlinearly
""")

st.latex("""
\\text{Centrifugal coupling: } m_2 L_1 L_2 \\dot{\\theta}_2^2 \\sin(\\theta_1 - \\theta_2) \\text{ and } 
m_2 L_1 L_2 \\dot{\\theta}_1^2 \\sin(\\theta_1 - \\theta_2)
""")

st.subheader("4.2 Coriolis Effect")
st.markdown("""
The coupling term $\\cos(\\theta_1 - \\theta_2)$ in the mass matrix creates **Coriolis-like effects**:
- The effective inertia depends on the relative angle between pendulums
- When $\\theta_1 - \\theta_2 = \\pm \\pi/2$, the coupling is zero
- When $\\theta_1 - \\theta_2 = 0$ or $\\pi$, coupling is maximum
""")

st.subheader("4.3 Gravitational Nonlinearity")
st.markdown("""
The gravitational terms $\\sin(\\theta_1)$ and $\\sin(\\theta_2)$ are fully nonlinear:
- For small angles: $\\sin(\\theta) \\approx \\theta$ (linear restoring force)
- For large angles: $\\sin(\\theta) < \\theta$ (weaker restoring force)
- This leads to longer periods for larger amplitudes
""")

st.markdown("---")

st.header("5. Interactive Nonlinear Dynamics Explorer")

st.sidebar.header("Nonlinear Dynamics Parameters")

# Parameters
L1_nl = st.sidebar.slider("Top Arm Length (L₁, m)", 0.5, 2.0, 1.0, 0.1, key="nl_L1")
L2_nl = st.sidebar.slider("Bottom Arm Length (L₂, m)", 0.5, 2.0, 1.0, 0.1, key="nl_L2")
m1_nl = st.sidebar.slider("Top Mass (m₁, kg)", 0.1, 5.0, 1.0, 0.1, key="nl_m1")
m2_nl = st.sidebar.slider("Bottom Mass (m₂, kg)", 0.1, 5.0, 1.0, 0.1, key="nl_m2")

# Initial conditions - allow large angles
st.sidebar.subheader("Initial Conditions (Large Angles)")
theta1_0_nl = st.sidebar.slider("Initial Top Angle (θ₁, degrees)", -180, 180, 45, 1, key="nl_theta1")
theta2_0_nl = st.sidebar.slider("Initial Bottom Angle (θ₂, degrees)", -180, 180, 60, 1, key="nl_theta2")
omega1_0_nl = st.sidebar.slider("Initial Top Angular Velocity (ω₁, rad/s)", -10.0, 10.0, 0.0, 0.1, key="nl_omega1")
omega2_0_nl = st.sidebar.slider("Initial Bottom Angular Velocity (ω₂, rad/s)", -10.0, 10.0, 0.0, 0.1, key="nl_omega2")

t_max_nl = st.sidebar.slider("Simulation Time (s)", 5, 50, 20, 1, key="nl_tmax")
dt_nl = st.sidebar.slider("Time Step (s)", 0.001, 0.05, 0.01, 0.001, key="nl_dt")

# Solve
pendulum_nl = DoublePendulum(L1=L1_nl, L2=L2_nl, m1=m1_nl, m2=m2_nl, g=9.81)
t_nl, sol_nl = pendulum_nl.solve(
    theta1_0=np.deg2rad(theta1_0_nl),
    theta2_0=np.deg2rad(theta2_0_nl),
    omega1_0=omega1_0_nl,
    omega2_0=omega2_0_nl,
    t_span=(0, t_max_nl),
    dt=dt_nl
)

theta1_nl = sol_nl[:, 0]
theta2_nl = sol_nl[:, 1]
omega1_nl = sol_nl[:, 2]
omega2_nl = sol_nl[:, 3]

# Get positions and energy
x1_nl, y1_nl, x2_nl, y2_nl = pendulum_nl.get_positions(theta1_nl, theta2_nl)
T_nl, V_nl, E_nl = pendulum_nl.get_energy(theta1_nl, theta2_nl, omega1_nl, omega2_nl)

# Plots
col3, col4 = st.columns(2)

with col3:
    st.subheader("Angular Motion")
    fig_ang, ax_ang = plt.subplots(figsize=(10, 6))
    ax_ang.plot(t_nl, np.rad2deg(theta1_nl), 'b-', linewidth=2, label='θ₁ (Top)')
    ax_ang.plot(t_nl, np.rad2deg(theta2_nl), 'r-', linewidth=2, label='θ₂ (Bottom)')
    ax_ang.set_xlabel('Time (s)', fontsize=12)
    ax_ang.set_ylabel('Angle (degrees)', fontsize=12)
    ax_ang.set_title('Nonlinear Angular Displacement', fontsize=14, weight='bold')
    ax_ang.legend(fontsize=11)
    ax_ang.grid(True, alpha=0.3)
    st.pyplot(fig_ang)
    
    st.subheader("Trajectory Traces")
    fig_traj, ax_traj = plt.subplots(figsize=(10, 10))
    
    # Traces
    ax_traj.plot(x1_nl, y1_nl, 'b-', linewidth=1.5, alpha=0.6, label='Top Mass Trace')
    ax_traj.plot(x2_nl, y2_nl, 'r-', linewidth=1.5, alpha=0.6, label='Bottom Mass Trace')
    
    # Current positions
    ax_traj.plot(x1_nl[-1], y1_nl[-1], 'bo', markersize=10, label='Top Mass')
    ax_traj.plot(x2_nl[-1], y2_nl[-1], 'ro', markersize=10, label='Bottom Mass')
    
    # Arms
    ax_traj.plot([0, x1_nl[-1]], [0, y1_nl[-1]], 'b-', linewidth=2, alpha=0.5)
    ax_traj.plot([x1_nl[-1], x2_nl[-1]], [y1_nl[-1], y2_nl[-1]], 'r-', linewidth=2, alpha=0.5)
    ax_traj.plot(0, 0, 'ko', markersize=12, label='Pivot')
    
    ax_traj.set_xlabel('X Position (m)', fontsize=12)
    ax_traj.set_ylabel('Y Position (m)', fontsize=12)
    ax_traj.set_title('Nonlinear Trajectory Traces', fontsize=14, weight='bold')
    ax_traj.legend(fontsize=10)
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_aspect('equal')
    
    max_range = max(L1_nl + L2_nl, np.max(np.abs(x2_nl)), np.max(np.abs(y2_nl))) * 1.2
    ax_traj.set_xlim(-max_range, max_range)
    ax_traj.set_ylim(-max_range, max_range)
    
    st.pyplot(fig_traj)
    
    # Create animated trajectory trace
    st.subheader("Animated Trajectory Traces")
    
    # Create animation function
    def create_trajectory_animation(x1, y1, x2, y2, L1, L2, fps=30, duration=10):
        """Create MP4 animation of trajectory traces"""
        # Calculate frame step to match desired duration
        total_frames = int(duration * fps)
        frame_step = max(1, len(x1) // total_frames)
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Set up axes
        max_range = max(L1 + L2, np.max(np.abs(x2)), np.max(np.abs(y2))) * 1.2
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_aspect('equal')
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Nonlinear Trajectory Traces Animation', fontsize=14, weight='bold')
        ax.grid(True, alpha=0.3)
        
        # Initialize empty lines
        line1, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.6, label='Top Mass Trace')
        line2, = ax.plot([], [], 'r-', linewidth=1.5, alpha=0.6, label='Bottom Mass Trace')
        
        # Initialize markers for current positions
        marker1, = ax.plot([], [], 'bo', markersize=10, label='Top Mass')
        marker2, = ax.plot([], [], 'ro', markersize=10, label='Bottom Mass')
        
        # Initialize arm lines
        arm1, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5)
        arm2, = ax.plot([], [], 'r-', linewidth=2, alpha=0.5)
        
        # Pivot point
        ax.plot(0, 0, 'ko', markersize=12, label='Pivot')
        
        ax.legend(fontsize=10, loc='upper right')
        
        def animate(frame):
            idx = min(frame * frame_step, len(x1) - 1)
            
            # Update traces
            line1.set_data(x1[:idx+1], y1[:idx+1])
            line2.set_data(x2[:idx+1], y2[:idx+1])
            
            # Update markers
            marker1.set_data([x1[idx]], [y1[idx]])
            marker2.set_data([x2[idx]], [y2[idx]])
            
            # Update arms
            arm1.set_data([0, x1[idx]], [0, y1[idx]])
            arm2.set_data([x1[idx], x2[idx]], [y1[idx], y2[idx]])
            
            return line1, line2, marker1, marker2, arm1, arm2
        
        # Create animation
        anim = FuncAnimation(fig, animate, frames=total_frames, 
                            interval=1000/fps, blit=True, repeat=True)
        
        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.mp4')
        temp_file.close()
        
        writer = FFMpegWriter(fps=fps, bitrate=1800)
        anim.save(temp_file.name, writer=writer)
        plt.close(fig)
        
        return temp_file.name
    
    # Create a unique key based on parameters for caching
    param_hash = hashlib.md5(
        f"{L1_nl}_{L2_nl}_{m1_nl}_{m2_nl}_{theta1_0_nl}_{theta2_0_nl}_{omega1_0_nl}_{omega2_0_nl}_{t_max_nl}_{dt_nl}".encode()
    ).hexdigest()
    cache_key = f"anim_{param_hash}"
    
    # Button to generate animation
    if st.button("🎬 Generate Animation", key="generate_anim_btn", type="primary"):
        with st.spinner("Creating animation... This may take a moment."):
            try:
                anim_file = create_trajectory_animation(
                    x1_nl, y1_nl, x2_nl, y2_nl, L1_nl, L2_nl, fps=30, duration=10
                )
                st.session_state[cache_key] = anim_file
                st.success("Animation created successfully!")
            except Exception as e:
                st.error(f"Error creating animation: {str(e)}")
                st.info("Make sure ffmpeg is installed. On Linux: `sudo apt-get install ffmpeg`")
    
    # Display animation if available
    if cache_key in st.session_state and os.path.exists(st.session_state[cache_key]):
        anim_file = st.session_state[cache_key]
        st.markdown("**Animation Preview:**")
        with open(anim_file, 'rb') as video_file:
            video_bytes = video_file.read()
            st.video(video_bytes)
        
        # Download button
        st.download_button(
            label="📥 Download Animation (MP4)",
            data=video_bytes,
            file_name="nonlinear_trajectory_traces.mp4",
            mime="video/mp4",
            key="download_anim"
        )
    else:
        st.info("👆 Click the button above to generate an animated MP4 of the trajectory traces.")

with col4:
    st.subheader("Phase Space")
    fig_phase, (ax_phase1, ax_phase2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Top pendulum phase space
    ax_phase1.plot(theta1_nl, omega1_nl, 'b-', linewidth=1.5, alpha=0.7)
    ax_phase1.set_xlabel('θ₁ (rad)', fontsize=12)
    ax_phase1.set_ylabel('ω₁ (rad/s)', fontsize=12)
    ax_phase1.set_title('Top Pendulum Phase Space', fontsize=13, weight='bold')
    ax_phase1.grid(True, alpha=0.3)
    
    # Bottom pendulum phase space
    ax_phase2.plot(theta2_nl, omega2_nl, 'r-', linewidth=1.5, alpha=0.7)
    ax_phase2.set_xlabel('θ₂ (rad)', fontsize=12)
    ax_phase2.set_ylabel('ω₂ (rad/s)', fontsize=12)
    ax_phase2.set_title('Bottom Pendulum Phase Space', fontsize=13, weight='bold')
    ax_phase2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    st.pyplot(fig_phase)
    
    st.subheader("Energy Analysis")
    fig_energy, ax_energy = plt.subplots(figsize=(10, 6))
    ax_energy.plot(t_nl, T_nl, 'g-', linewidth=2, label='Kinetic Energy')
    ax_energy.plot(t_nl, V_nl, 'b-', linewidth=2, label='Potential Energy')
    ax_energy.plot(t_nl, E_nl, 'r--', linewidth=2, label='Total Energy')
    ax_energy.set_xlabel('Time (s)', fontsize=12)
    ax_energy.set_ylabel('Energy (J)', fontsize=12)
    ax_energy.set_title('Energy Conservation (Nonlinear)', fontsize=14, weight='bold')
    ax_energy.legend(fontsize=11)
    ax_energy.grid(True, alpha=0.3)
    st.pyplot(fig_energy)

st.markdown("---")

st.header("6. Chaotic Behavior Indicators")

col5, col6 = st.columns(2)

with col5:
    st.subheader("Sensitivity to Initial Conditions")
    st.markdown("""
    **Test:** Compare trajectories with slightly different initial conditions.
    
    For chaotic systems, small differences in initial conditions lead to 
    exponentially diverging trajectories.
    """)
    
    # Compare two nearby initial conditions
    theta1_pert = np.deg2rad(theta1_0_nl) + 0.001
    theta2_pert = np.deg2rad(theta2_0_nl) + 0.001
    
    t_pert, sol_pert = pendulum_nl.solve(
        theta1_0=theta1_pert,
        theta2_0=theta2_pert,
        omega1_0=omega1_0_nl,
        omega2_0=omega2_0_nl,
        t_span=(0, t_max_nl),
        dt=dt_nl
    )
    
    # Calculate separation
    separation = np.sqrt((sol_nl[:, 0] - sol_pert[:, 0])**2 + 
                        (sol_nl[:, 1] - sol_pert[:, 1])**2)
    
    fig_sep, ax_sep = plt.subplots(figsize=(10, 6))
    ax_sep.semilogy(t_nl, separation, 'r-', linewidth=2)
    ax_sep.set_xlabel('Time (s)', fontsize=12)
    ax_sep.set_ylabel('Separation (rad)', fontsize=12)
    ax_sep.set_title('Trajectory Separation (Chaos Indicator)', fontsize=14, weight='bold')
    ax_sep.grid(True, alpha=0.3)
    st.pyplot(fig_sep)

with col6:
    st.subheader("Poincaré Section")
    st.markdown("""
    **Poincaré sections** reveal the structure of chaotic attractors.
    
    We plot the system state whenever θ₁ crosses zero with positive velocity.
    """)
    
    # Find zero crossings
    zero_crossings = []
    for i in range(1, len(theta1_nl)):
        if theta1_nl[i-1] < 0 and theta1_nl[i] >= 0 and omega1_nl[i] > 0:
            zero_crossings.append(i)
    
    if len(zero_crossings) > 0:
        indices = np.array(zero_crossings)
        fig_poincare, ax_poincare = plt.subplots(figsize=(10, 6))
        ax_poincare.scatter(theta2_nl[indices], omega2_nl[indices], 
                           s=20, alpha=0.6, c='red')
        ax_poincare.set_xlabel('θ₂ (rad)', fontsize=12)
        ax_poincare.set_ylabel('ω₂ (rad/s)', fontsize=12)
        ax_poincare.set_title('Poincaré Section (θ₁ = 0, ω₁ > 0)', fontsize=14, weight='bold')
        ax_poincare.grid(True, alpha=0.3)
        st.pyplot(fig_poincare)
    else:
        st.info("No zero crossings found. Try adjusting initial conditions or simulation time.")

st.markdown("---")
st.info("""
**Key Takeaways:**
- Nonlinear equations capture the full dynamics including chaotic behavior
- Large initial angles lead to complex, unpredictable motion
- Energy is still conserved (within numerical precision)
- The system exhibits sensitive dependence on initial conditions for large angles
""")

