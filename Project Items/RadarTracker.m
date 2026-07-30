%% Title: Graduate Project 2 (Radar Signal Filtering)
%
% * Objective: The objective of this project is to generate a simple linear
%              motion model and have a kalman filter estimate the true 
%              position based on noisy measurements. A target is being
%              tracked traveling at a constant speed of 20 m/s and is
%              attempting to scramble its position. Acceleration is not 
%              accounted for in the is model, however the acceleration 
%              and deceleration by default contribute to the error 
%              calculated in the Process Noise (Q matrix). Through the use 
%              of a kalman filter, the targets position can located based 
%              on estimation. This serves as a fundamental demonstration of
%              how a kalman filter works to extract the true positon of the
%              target represented by the linear motion model.
%              
% * Name: John Schatzel
% 
% * Date: 4/30/2025
% * Course: ECEN 5830

close all
clc
clear all

%% Data Section
dt = 0.1;         % sampling interval, smaller dt = better filter tracking
t = 0:dt:20;       % timing vector
xi = 0;            % initial position
v = 20;            % 20 m/s constant velocity
sigma = 10;        % standard deviation of measurement noise (refer to R for its relation)

%% Target motion equation and noise addition
% linear motion model w/ const. velocity (represents true trajectory of
% target)
x = xi + v*t;

% simulate radar measurements
z = x + sigma*randn(size(t)); % radar measurement vector

%% State Space Model Definition
% (2 states, 1 output, no inputs)

% State Transition Matrix (n x n)
% position updated by velocity, velocity constant
A = [1 dt; 0 1]; % derived from Kinematics                  

% Control Input Matrix (n x m)
% B = [0; 0]; % no external control input -> [0; 0];
% although we do not have any control input, we still want to inject 
% process noise into the position and velocity so that Q may be calculated!
B = eye(2); % indpendently inject process noise into each state

% State-to-Measurement Matrix (called 'H' for Kalman filtering) (p x n)
C = [1 0]; % only measuring position                        

% Feedthrough Matrix (p x m)
% D = 0; % input has no instantaneous affect on the measurement
D = zeros(1,2); % needed for ss model

% Process Noise Covariance (uncertainty in model dynamics)
% - smaller Q: motion model is accurate
% - larger Q: motion model is uncertain (increase predicted uncertainty)
Q = [1 0; 0 1];

% Measurement Noise Covariance (uncertainty in sensor measurements)
% - smaller R: filter trusts measurements more (believes sensor is accurate)
% - larger R: filter trusts model more (believes that the sensor is noisy)
R = sigma^2;

% measuring position and velocity
% pos = (xi + v*t) + sigma*randn(size(t));
% vel = v*ones(size(t));      % or noisy velocity if available
% z = [pos(:), vel(:)];       % 201-by-2

sys = ss(A, B, C, D, dt);

% Creating Kalman Filter from the Built-In 'kalman' Command
[kalmf, L, P] = kalman(sys, Q, R); % MATLAB generated kalman filter,
% kalmf = Kalman filter system
% L = Kalman Gain (NOT DIRECTLY USED)
% P = Steady-state error covariance

% Simulate kalmf on radar measurements
[y, time, x_kalmf_estimates] = lsim(kalmf, z, t);
% y = sensor reconstruction reading
% time = same timing vector as 't'
% x_kalmf_estimates = internal states at each time step

%% Manual Kalman Filter Implementation
% Initialize state and covariance
X_est = [0; 20];       % Initial position and velocity (STATE VECTOR)
P_est = P;             % Initial covariance estimate, used results from built-in filter to make the comparison fair (STATE COVARIANCE)

% Preallocate for storage
X_estimates = zeros(2, length(t));

% Portion followed example on MathWorks website
% kalman filter loop
for k = 1:length(t)
    % Measurement Update (Correction step)
    K = P_est * C' / (C * P_est * C' + R);    % Kalman Gain
    X_est = X_est + K * (z(k) - C * X_est);   % Corrected state
    P_est = (eye(2) - K * C) * P_est;         % Corrected covariance

    % Store corrected estimate
    X_estimates(:, k) = X_est;

    % Time Update (Prediction step)
    X_est = A * X_est;             % Predict next state
    P_est = A * P_est * A' + Q;    % Predict next covariance
end


%% Plots

% True Position vs. Noisy Measurements
figure
plot(t, x, 'g-', ...
     t, z, 'rx')

legend('True Position', 'Radar Measurements', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('True Position vs. Radar Measurements');
grid on;

% Kalman Filter Plots Against True Position
figure;
plot(t, x, 'g-', ...
     t, X_estimates(1,:), 'b--', ...
     t, x_kalmf_estimates(:,1), 'm-.', 'LineWidth', 1.5);
legend('True Position', 'Manual Kalman Filter Estimate', 'MATLAB Estimate', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('Manual vs. MATLAB Kalman Filter');
grid on;

% Kalman Filters vs. True Position vs. Noisy Measurements
figure;
plot(t, x, 'g-', ...
     t, z, 'rx', ...
     t, X_estimates(1,:), 'b--', ...
     t, x_kalmf_estimates(:,1), 'm-.', 'LineWidth', 1.5);
legend('True Position', 'Radar Measurements', 'Manual Kalman Filter Estimate', 'MATLAB kalmf Estimate', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('Radar Target Tracking using Kalman Filter');
grid on;

%% Kalman Filter Accuracy Checks
% Extract position estimates
man_est   = X_estimates(1,:).';        % Manual KF position (Nx1)
kalmf_est = x_kalmf_estimates(:,1);    % MATLAB kalmf position (Nx1)
truth     = x(:);          % True position (Nx1)

% MAPE: Mean Absolute Percentage Error
% ACC: Accuracy

% Mask out zero values in truth to avoid divide-by-zero in MAPE
mask = truth ~= 0;

% Manual KF metrics
MAPE_manual = mean(abs((man_est(mask) - truth(mask)) ./ truth(mask))) * 100;
ACC_manual  = 100 - MAPE_manual;

% MATLAB kalmf metrics
MAPE_kalmf = mean(abs((kalmf_est(mask) - truth(mask)) ./ truth(mask))) * 100;
ACC_kalmf  = 100 - MAPE_kalmf;

% Print results
fprintf('Manual Kalman Filter: \nMAPE: %.2f%% \nAccuracy: %.2f%%\n', MAPE_manual, ACC_manual);
fprintf('\nMATLAB kalmf Command: \nMAPE: %.2f%% \nAccuracy: %.2f%%\n', MAPE_kalmf, ACC_kalmf);

