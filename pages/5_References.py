import streamlit as st
import os
import zipfile
import io

st.set_page_config(page_title="Double Pendulum - References", layout="wide")

st.title("References and Code Download")
st.markdown("---")

col1, col2 = st.columns([1, 1])

with col1:
    st.header("📚 References")
    
    st.subheader("Classical Mechanics")
    st.markdown("""
    1. **Goldstein, H., Poole, C., & Safko, J.** (2002). *Classical Mechanics* (3rd ed.). 
       Addison-Wesley. Chapter 1: Survey of the Elementary Principles.
    
    2. **Taylor, J. R.** (2005). *Classical Mechanics*. University Science Books.
       Chapter 7: Lagrange's Equations.
    
    3. **Marion, J. B., & Thornton, S. T.** (1995). *Classical Dynamics of Particles and Systems* 
       (4th ed.). Harcourt Brace & Company. Chapter 7: Hamilton's Principle.
    """)
    
    st.subheader("Double Pendulum Specific")
    st.markdown("""
    4. **Shinbrot, T., Grebogi, C., Wisdom, J., & Yorke, J. A.** (1992). 
       "Chaos in a double pendulum." *American Journal of Physics*, 60(6), 491-499.
       DOI: 10.1119/1.16860
    
    5. **Levien, R., & Tan, S. M.** (1993). "Double pendulum: An experiment in chaos." 
       *American Journal of Physics*, 61(11), 1038-1044.
       DOI: 10.1119/1.17335
    
    6. **Acheson, D.** (1997). *From Calculus to Chaos: An Introduction to Dynamics*. 
       Oxford University Press. Chapter 5: The Double Pendulum.
    """)
    
    st.subheader("Numerical Methods")
    st.markdown("""
    7. **Press, W. H., Teukolsky, S. A., Vetterling, W. T., & Flannery, B. P.** (2007). 
       *Numerical Recipes: The Art of Scientific Computing* (3rd ed.). 
       Cambridge University Press. Chapter 17: Integration of Ordinary Differential Equations.
    
    8. **Butcher, J. C.** (2016). *Numerical Methods for Ordinary Differential Equations* (3rd ed.). 
       Wiley. Chapter 2: Runge-Kutta Methods.
    """)

with col2:
    st.header("💻 Code Download")
    
    st.markdown("""
    Download the complete source code for this double pendulum simulation.
    The package includes:
    - Main Streamlit application
    - Physics simulation module
    - All pages and visualizations
    - Requirements file
    - README documentation
    """)
    
    # Create zip file with all code
    def create_code_zip():
        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
            # Add main files
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            
            files_to_include = [
                'app.py',
                'double_pendulum.py',
                'requirements.txt',
                'README.md',
                'pages/1_Introduction.py',
                'pages/2_Mathematical_Details.py',
                'pages/3_Simulation.py',
                'pages/4_Nonlinear_Dynamics.py',
                'pages/5_References.py'
            ]
            
            for file_path in files_to_include:
                full_path = os.path.join(base_dir, file_path)
                if os.path.exists(full_path):
                    zip_file.write(full_path, file_path)
        
        zip_buffer.seek(0)
        return zip_buffer
    
    zip_buffer = create_code_zip()
    
    st.download_button(
        label="📥 Download Complete Code (ZIP)",
        data=zip_buffer,
        file_name="double_pendulum_app.zip",
        mime="application/zip"
    )
    
    st.markdown("---")
    st.subheader("Installation Instructions")
    st.markdown("""
    1. Extract the downloaded ZIP file
    2. Install dependencies:
       ```bash
       pip install -r requirements.txt
       ```
    3. Run the application:
       ```bash
       streamlit run app.py
       ```
    """)
    
    st.subheader("Code Structure")
    st.markdown("""
    ```
    double_pendulum_app/
    ├── app.py                    # Main Streamlit entry point
    ├── double_pendulum.py        # Physics simulation module
    ├── requirements.txt          # Python dependencies
    ├── README.md                 # Documentation
    └── pages/
        ├── 1_Introduction.py           # Introduction page
        ├── 2_Mathematical_Details.py   # Math details page
        ├── 3_Simulation.py             # Interactive simulation
        ├── 4_Nonlinear_Dynamics.py     # Full nonlinear dynamics
        └── 5_References.py             # References and download
    ```
    """)

st.markdown("---")
st.header("🔬 Key Concepts Implemented")

st.markdown("""
This application demonstrates:

1. **Lagrangian Mechanics**: Using the Euler-Lagrange equations to derive equations of motion
2. **Coupled Nonlinear ODEs**: Solving a system of second-order differential equations
3. **Numerical Integration**: Using scipy's odeint (LSODA) for time-stepping
4. **Energy Conservation**: Monitoring total energy to verify numerical accuracy
5. **Chaotic Dynamics**: The double pendulum exhibits chaotic behavior for large initial angles
6. **Phase Space Analysis**: Visualizing the system's trajectory in phase space
7. **Interactive Visualization**: Real-time parameter adjustment and visualization
""")

st.markdown("---")
st.header("📝 License and Usage")

st.markdown("""
This code is provided for educational purposes. Feel free to:
- Use it for learning and teaching
- Modify and extend it
- Share it with attribution

For questions or improvements, please refer to the references above or consult classical mechanics textbooks.
""")

