import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
import matplotlib.patches as mpatches

st.set_page_config(page_title="Double Pendulum - Introduction", layout="wide")

st.title("Double Pendulum Oscillation")
st.markdown("---")

col1, col2 = st.columns([1, 1])

with col1:
    st.header("System Description")
    st.markdown("""
    The double pendulum consists of two pendulums connected in series:
    
    - **Top Pendulum**: Attached to a fixed ceiling point, free to swing without friction
    - **Bottom Pendulum**: Attached to the end of the top pendulum, free to rotate
    
    Both pendulums are modeled with:
    - Point masses at their ends
    - Massless, rigid rods
    - No friction or air resistance
    - Motion constrained to a plane
    """)
    
    st.header("Parameters")
    st.markdown("""
    - **L₁**: Length of top pendulum arm
    - **L₂**: Length of bottom pendulum arm
    - **m₁**: Mass of top pendulum bob
    - **m₂**: Mass of bottom pendulum bob
    - **θ₁**: Angular displacement of top pendulum (from vertical)
    - **θ₂**: Angular displacement of bottom pendulum (from vertical)
    - **g**: Acceleration due to gravity
    """)

with col2:
    st.header("System Diagram")
    
    # Create figure for diagram
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Ceiling
    ax.plot([-1.5, 1.5], [1.5, 1.5], 'k-', linewidth=3, label='Ceiling')
    
    # Pivot point
    pivot_x, pivot_y = 0, 1.5
    ax.plot(pivot_x, pivot_y, 'ko', markersize=10)
    
    # Top pendulum (at angle)
    L1 = 1.0
    theta1 = np.pi / 6  # 30 degrees
    top_end_x = pivot_x + L1 * np.sin(theta1)
    top_end_y = pivot_y - L1 * np.cos(theta1)
    
    # Draw top pendulum arm
    ax.plot([pivot_x, top_end_x], [pivot_y, top_end_y], 'b-', linewidth=3, label='Top Arm (L₁)')
    
    # Top pendulum mass
    circle1 = Circle((top_end_x, top_end_y), 0.1, color='red', zorder=5)
    ax.add_patch(circle1)
    ax.text(top_end_x + 0.15, top_end_y, 'm₁', fontsize=12, weight='bold')
    
    # Bottom pendulum
    L2 = 0.8
    theta2 = np.pi / 4  # 45 degrees
    bottom_end_x = top_end_x + L2 * np.sin(theta2)
    bottom_end_y = top_end_y - L2 * np.cos(theta2)
    
    # Draw bottom pendulum arm
    ax.plot([top_end_x, bottom_end_x], [top_end_y, bottom_end_y], 'g-', linewidth=3, label='Bottom Arm (L₂)')
    
    # Bottom pendulum mass
    circle2 = Circle((bottom_end_x, bottom_end_y), 0.1, color='red', zorder=5)
    ax.add_patch(circle2)
    ax.text(bottom_end_x + 0.15, bottom_end_y, 'm₂', fontsize=12, weight='bold')
    
    # Draw angles
    # Angle 1 arc
    arc1 = mpatches.Arc((pivot_x, pivot_y), 0.4, 0.4, angle=0, 
                       theta1=0, theta2=np.degrees(theta1), color='blue', linewidth=2)
    ax.add_patch(arc1)
    ax.text(0.3, 1.3, 'θ₁', fontsize=14, color='blue', weight='bold')
    
    # Angle 2 arc (relative to top arm)
    arc2 = mpatches.Arc((top_end_x, top_end_y), 0.3, 0.3, angle=np.degrees(theta1), 
                       theta1=0, theta2=np.degrees(theta2), color='green', linewidth=2)
    ax.add_patch(arc2)
    ax.text(top_end_x + 0.25, top_end_y - 0.2, 'θ₂', fontsize=14, color='green', weight='bold')
    
    # Vertical reference line
    ax.plot([pivot_x, pivot_x], [pivot_y, pivot_y - 1.5], 'k--', linewidth=1, alpha=0.3)
    
    # Labels
    ax.text(pivot_x, pivot_y + 0.15, 'Pivot Point', fontsize=10, ha='center')
    ax.text(-1.8, 1.5, 'Fixed Ceiling', fontsize=10, va='center')
    
    ax.set_title('Double Pendulum System', fontsize=16, weight='bold', pad=20)
    
    plt.tight_layout()
    st.pyplot(fig)
    
    # Legend
    st.markdown("""
    **Legend:**
    - Blue line: Top pendulum arm (L₁)
    - Green line: Bottom pendulum arm (L₂)
    - Red circles: Point masses (m₁, m₂)
    - θ₁: Angular displacement of top pendulum
    - θ₂: Angular displacement of bottom pendulum
    """)

st.markdown("---")
st.info("💡 Navigate to the next pages to explore the mathematical details and interactive simulation!")

