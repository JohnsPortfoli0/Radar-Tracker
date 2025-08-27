# Radar Tracker
This project implements a radar signal filtering and tracking system designed to estimate the true state of a moving target in the presence of noise. The tracker is built in MATLAB and Simulink and utilizes a Kalman filter to smooth noisy radar measurements, predict future positions, and improve accuracy compared to raw sensor data.

# Steps for Running the Script
Please download the `RadarTracker.m` file under "Project Items"

1. Open `RadarTracker.m` in MATLAB
   
2. Run the script and observe the plots
   
   `Figure 1`: Plots the true position of the target as well as the noisy measurements  
   `Figure 2`: Shows a manual Kalman filter vs. MATLABs Kalman filter command  
   `Figure 3`: Shows combination Figure 1 and 2

# Steps for Running the Simulink Demo
Please download the `RadarSimulink.slx` file under "Project Items"

1. Open `RadarSimulink.slx` (either open MATLAB and invoke simulink in the command window then open the file, or double click the file to open it).
2. Run the `RadarTracker.m` in MATLAB (this should resolve the errors on the `Noise Generation` block and the `Control Input` block).
3. Run `RadarSimulink.slx` in the Simulink window.
4. After it has run, double click the `scope` to see the plots that were generated.
5. Results shown in the `scope` should be identical to `Figure 3` in MATLAB.

# Script Explanation and Flow
**1. Data Initialization**
- Defines the sampling interval, simulation time vector, and target parameters (initial position, constant velocity).
- Adds measurement noise to simulate radar readings (radar_measurements)
  
**2. Motion and Measurements**
- Generates the true position of the target using a simple linear motion equation
- Produces noisy measurements by adding Gaussian noise to the true trajectory.
- Plots true vs. measured positions to visualize sensor error.
  
**3. State-Space Model**
- Defines a state transition model for position and velocity:
   - Matrix `A` updates position and velocity over time.
   - Matrix `C` indicates only position is measured.
- Defines noise covariance matrices:
   - `Q` for process noise (uncertainty in dynamics).
   - `R` for measurement noise (uncertainty in sensor).
  
**4. Kalman Filter Setup**
- Creates a state-space system in MATLAB with `ss()`.
- Calls the built-in `kalman()` function to design a Kalman filter and obtain:
- `kalmf`: the Kalman filter system.
- `L`: Kalman gain.
- `P`: steady-state error covariance.
  
**5. Manual Kalman Filter Implementation**
- Initializes state estimates `X_est` with position and velocity and covariance estimate `P_est`.
- Runs a prediction-correction loop:
   - Correction step: Updates the estimate using measurement and Kalman gain.
   - Prediction step: Projects the state and covariance forward in time.
- Stores results for comparison with MATLAB’s filter.
  
**6. MATLAB Kalman Filter Simulation** 
- Uses `lsim()` to simulate the built-in Kalman filter on radar data.
- Collects state estimates from MATLAB’s implementation.
  
**7. Results and Visualization**
- Plots and compares:
   - Raw radar measurements.
   - Manual Kalman filter estimates.
   - MATLAB Kalman filter estimates.
- Shows how the Kalman Filter smooths noisy measurements and accurately reconstructs the target’s trajectory.
  
# Key Features
- Models a target moving at a constant velocity of 20 m/s with noisy radar measurements.
- Adds Gaussian noise to simulate realistic sensor errors.
- Defines system dynamics with transition, measurement, and covariance matrices.
- Manual filter loop (prediction + correction).
- MATLAB’s built-in kalman() function.
- Plots true position, noisy radar measurements, and estimates from both Kalman filters.
- Provides a fundamental demonstration of how Kalman Filters extract useful state estimates from noisy data.
 
# Requirments
- MATLAB R2024a (This is what I used)
- Toolboxes
  - Control System Toolbox (e.g., `ss`, `kalman`, `lsim`)

# License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

