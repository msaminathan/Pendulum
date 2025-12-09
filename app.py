import streamlit as st

st.set_page_config(
    page_title="Double Pendulum Simulation",
    page_icon="🔬",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🔬 Double Pendulum Oscillation Simulator")
st.markdown("""
Welcome to the Double Pendulum Simulation App! This interactive application demonstrates 
the fascinating dynamics of a double pendulum system using Lagrangian mechanics.

Navigate through the pages using the sidebar to explore:
- **Introduction**: System description and diagram
- **Mathematical Details**: Complete derivation of equations of motion
- **Simulation**: Interactive simulation with customizable parameters
- **Nonlinear Dynamics**: Full nonlinear equations and chaotic behavior analysis
- **References**: Academic references and code download
""")

st.markdown("---")

col1, col2, col3 = st.columns(3)

with col1:
    st.markdown("""
    ### 📐 System Overview
    Two pendulums connected in series, demonstrating complex nonlinear dynamics and chaos.
    """)

with col2:
    st.markdown("""
    ### 📊 Interactive Simulation
    Adjust parameters in real-time and observe the motion, trajectories, and energy conservation.
    """)

with col3:
    st.markdown("""
    ### 📚 Educational Resource
    Complete mathematical derivations and references for learning classical mechanics.
    """)

st.markdown("---")
st.info("💡 **Tip**: Use the sidebar to navigate between pages and explore different aspects of the double pendulum system!")

