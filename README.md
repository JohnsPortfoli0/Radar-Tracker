# Radar Tracker
This project implements a radar signal filtering and tracking system designed to estimate the true state of a moving target in the presence of noise. The tracker is built in MATLAB and Simulink and utilizes a Kalman filter to smooth noisy radar measurements, predict future positions, and improve accuracy compared to raw sensor data.

Note: Original instructions wanted you to download 'RadarTracker.m'. This version is still valid, but is less clean than the one recommended below!

# Steps for Running the Script
Please download the `RadarTracker_Pos.mlx` file under "Project Items"

1. Open `RadarTracker_Pos.mlx` in MATLAB
   
2. Run the script and observe the plots
   
   `Figure 1`: Plots the true position of the target as well as the noisy measurements  
   `Figure 2`: Shows a recursive Kalman filter vs. MATLABs Kalman filter command vs. true position
   `Figure 3`: Shows combination Figure 1 and 2
   `Figure 4`: Shows true position vs. MATLABs Kalman filter command vs. the output of the MATLAB Kalman filter command (sensor reconstruction reading)

   NOTE: The sensor reconstruction reading is the predicted measurement computed from the estimated states.

# Steps for Running the Simulink Demo
Please download the `RadarSimulink.slx` file under "Project Items"

1. Open `RadarSimulink.slx` (either open MATLAB and invoke simulink in the command window then open the file, or double click the .slx file to open it).
2. Run the `RadarTracker.m` in MATLAB (this should resolve the errors on the `Noise Generation` block and the `Control Input` block).
3. Run `RadarSimulink.slx` in the Simulink window.
4. After it has run, double click the `scope` to see the plots that were generated.
5. Results shown in the `scope` should be identical to `Figure 3` in MATLAB.

# Script Explanation and Flow
**1. Data Section**
- Defines the sampling interval, simulation time vector, and target parameters (initial position, constant velocity).
- Adds measurement noise to simulate radar readings (radar_measurements)
  
**2. Motion Model**
- Generates the true position of the target using a simple linear motion equation
- Produces noisy measurements by adding Gaussian noise to the true trajectory.
  
**3. State-Space Model Definition**
- Defines a state transition model for position and velocity:
   - Matrix `A` updates position and velocity over time. (Derived from Kinematics for this project x = xi + v*t).
   - Matrix `B` Control Input, which we do not have since we are not accounting for accelerations and decelerations. However this must be set to eye(2) so we can independently inject process noise into each state and calculate Q.
   - Matrix `C` indicates only position is measured.
   - Matrix `D` no input so there is no instantaneous affect on the measurement, D = 0, but matrix size must match number of rows in Matrix C so it is set to zeros(1, 2).

** 4. Noise Covariance
- Defines noise covariance matrices:
   - `Q` for process noise (uncertainty in dynamics).       
   - `R` for measurement noise (uncertainty in sensor).
  
**5. Kalman Filter Setup**
- Creates a state-space system in MATLAB with `ss()`.
- Calls the built-in `kalman()` function to design a Kalman filter and obtain:
- `kalmf`: the Kalman filter system.
- `L`: Kalman gain.
- `P`: steady-state error covariance.
- Simulates the filter given noisy readings
- `y`: sensor reconstruction reading (reconstructed estimate based on internal state estimates)
- `time`: same timing vector as 't'
- `x_kalmf_estimates`: internal state estimates at each time step
  
**6. Recursive Kalman Filter Implementation**
- Initializes state estimates `X_est` with position and velocity and covariance estimate `P_est`.
- Runs a prediction-correction loop:
   - Correction step: Updates the estimate using measurement and Kalman gain.
   - Prediction step: Projects the state and covariance forward in time.
- Stores results for comparison with MATLAB’s filter.
  
**7. MATLAB Kalman Filter Simulation** 
- Uses `lsim()` to simulate the built-in Kalman filter on radar data.
- Collects state estimates from MATLAB’s implementation.
  
**8. Results and Visualization**
- Plots and compares:
   - Raw radar measurements.
   - Manual Kalman filter estimates.
   - MATLAB Kalman filter estimates.
- Shows how the Kalman Filter smooths noisy measurements and accurately reconstructs the target’s trajectory.
- Calculates the MAPE from the position estimates and gives back both filters accuracy as a percentage.
  
# Key Features

**Simulated Radar Noise**  
- Models a moving target (20 m/s) under radar measurement noise to mimic real-world tracking scenarios.

**Dual Kalman Filter Implementations**  
- Includes both a **recursive filter loop** (prediction + correction) and **MATLAB’s `kalman()` function**, showcasing hands-on and built-in approaches.

**State-Space Modeling**  
- Defines essential state-transition and measurement matrices (`A`, `C`, etc.), plus process and measurement noise covariances (`Q`, `R`).

**Visual Results Comparison**  
- Generates three key plots:  
   - True vs. noisy radar measurements  
   - Manual vs. MATLAB Kalman filter estimates  
   - Combined comparison for in-depth analysis

 ** Accuracy Check**
 - Checks the accuracy of both Kalman Filter implementations and outputs the results in the Command Window.

**Educational and Insightful**  
  - Offers a clear and practical demonstration of Kalman filtering—great for radar tracking and navigation systems for beginners.
 
# Requirments
- MATLAB R2024a (This is what I used)
- Toolboxes
  - Control System Toolbox (e.g., `ss`, `kalman`, `lsim`)

# License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

