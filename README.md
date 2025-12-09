# Double Pendulum Oscillation Simulator

An interactive multipage Streamlit application that simulates and visualizes the motion of a double pendulum system using Lagrangian mechanics.

## Features

- **Interactive Simulation**: Real-time visualization with adjustable parameters
- **Mathematical Details**: Complete derivation of equations of motion using Lagrangian mechanics
- **Multiple Visualizations**: 
  - Angular displacement and velocity plots
  - Trajectory traces of both pendulum masses
  - Phase space diagrams
  - Energy conservation plots
- **Educational Content**: Comprehensive mathematical background and references

## Installation

1. Clone or download this repository
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the application:
   ```bash
   streamlit run app.py
   ```

## Application Structure

The app consists of five main pages:

1. **Introduction**: System description and visual diagram
2. **Mathematical Details**: Complete Lagrangian formulation and equations of motion
3. **Simulation**: Interactive simulation with customizable parameters
4. **Nonlinear Dynamics**: Full nonlinear equations without approximations, chaotic behavior analysis, and comparison plots
5. **References**: Academic references and code download

## System Description

The double pendulum consists of:
- **Top Pendulum**: Attached to a fixed ceiling point, free to swing without friction
- **Bottom Pendulum**: Attached to the end of the top pendulum, free to rotate

Both pendulums are modeled with:
- Point masses at their ends
- Massless, rigid rods
- No friction or air resistance
- Motion constrained to a plane

## Physics

The system is described using Lagrangian mechanics:
- Two generalized coordinates: θ₁ (top angle) and θ₂ (bottom angle)
- Coupled nonlinear differential equations derived from Euler-Lagrange equations
- Numerical solution using scipy's ODE integrator

## Parameters

- **L₁, L₂**: Lengths of top and bottom pendulum arms
- **m₁, m₂**: Masses of top and bottom pendulum bobs
- **g**: Acceleration due to gravity
- **Initial angles**: User-selectable via sliders

## References

See the References page in the application for complete academic citations.

## License

This code is provided for educational purposes. Feel free to use, modify, and share with attribution.

