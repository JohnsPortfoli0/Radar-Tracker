%% Title: Graduate Project 2 (Radar Signal Filtering)
%
% * Objective: The objective of this project is to generate a simple linear
%              equation of motion and have a kalman filter estimate its 
%              position based on noise measurements. A target is being
%              tracked traveling at a constant speed of 20 m/s and is
%              attempting to scramble its position. Through the use of a
%              kalman filter, the targets position can located based on
%              estimation. This serves as a fundamental demonstration of
%              how a kalman filter works to extract the location of its
%              picked target. 
%              
% * Name: John Schatzel
% 
% * Date: 4/30/2025
% * Course: ECEN 5830

close all
clear all
clc

%% Data Section
dt = 0.25;                        % sampling interval, smaller dt = better filter tracking
t = 0:dt:20;                      % timing vector
initial_position = 0;             % initial position
velocity = 20;                    % 20 m/s constant velocity
mn = 50;           % standard deviation of measurement noise

%% Target motion equation and noise addition
% true state (constant velocity motion)
true_position = initial_position + velocity*t;

% simulate radar measurements
radar_measurements = true_position + mn*randn(size(t));

figure
plot(t, true_position, 'g-', ...
     t, radar_measurements, 'rx')

legend('True Position', 'Radar Measurements', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('True Position vs. Radar Measurements');
grid on;

%% State Space Model Definition
% define state-space model

% value of '1' indicates that the new position/velocity depends 100% on the old position
% dt is the position over time
A = [1 dt; 0 1];                  % state transition matrix (position updated by velocity, velocity constant)

% value of '0' indicates there is nothing else controlling the position of
% the target over time
B = [0; 0];                       % control input matrix (no external control input)

% this indicates we are only measuring the position over time, not the
% velocity 
C = [1 0];                        % measurement matrix

% value set to '0' because there is no control input directly being passed
% to the output
D = 0;                            % direct transmission matrix

% value of '1' denotes there is slight disturbance in pos. and velo.
% higher variance than '1' = more disturbance
% value of '0' denotes there is no direct from pos. to velo. and vice versa
% noise covariance matrices
Q = [1 0; 0 1];               % process noise covariance (uncertainty in model dynamics)
R = mn^2;                     % measurement noise covariance (uncertainty in sensor measurements)

% create state-space system and Kalman filter
% NOTE: In the simulated system, B = [0; 0] and D = 0 because there is 
% no true control input.

% However, for Kalman filter design, we use B = eye(2) and D = zeros(1,2) 
% to model process noise entering the position and velocity states 
% independently.
sys = ss(A, eye(2), C, zeros(1,2), dt); % define system with separate noise inputs

% MATLAB Kalman Filter Command
[kalmf, L, P] = kalman(sys, Q, R); % MATLAB generated kalman filter, 
% kalmf = Kalman filter system
% L = Kalman Gain (NOT DIRECTLY USED)
% P = Steady-state error covariance

%% Manual Kalman Filter Implementation

% Initialize state and covariance
X_est = [0; 20];        % Initial position and velocity (or [0; 20] if you prefer)
P_est = P;             % Initial covariance estimate

% Preallocate for storage
X_estimates = zeros(2, length(t));

% Portion followed example on MathWorks website
% kalman filter loop
for k = 1:length(t)
    % Measurement Update (Correction step)
    K = P_est * C' / (C * P_est * C' + R);    % Kalman Gain
    X_est = X_est + K * (radar_measurements(k) - C * X_est); % Corrected state
    P_est = (eye(2) - K * C) * P_est;          % Corrected covariance

    % Store corrected estimate
    X_estimates(:, k) = X_est;

    % Time Update (Prediction step)
    X_est = A * X_est;             % Predict next state
    P_est = A * P_est * A' + Q;    % Predict next covariance
end


% MATLAB Kalman Filter (kalmf) Simulation
% Simulate kalmf on radar measurements
[y, time, x_kalmf_estimates] = lsim(kalmf, radar_measurements, t);
% y = output (NOT DIRECTLY USED)
% time = same timing vector as 't'
% x_kalmf_estimates = internal states at each time step

figure;
plot(t, true_position, 'g-', ...
     t, X_estimates(1,:), 'b--', ...
     t, x_kalmf_estimates(:,1), 'm-.', 'LineWidth', 1.5);
legend('True Position', 'Manual Kalman Filter Estimate', 'MATLAB Estimate', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('Manaual vs. MATLAB Kalman Filter');
grid on;


%% Plots
figure;
plot(t, true_position, 'g-', ...
     t, radar_measurements, 'rx', ...
     t, X_estimates(1,:), 'b--', ...
     t, x_kalmf_estimates(:,1), 'm-.', 'LineWidth', 1.5);
legend('True Position', 'Radar Measurements', 'Manual Kalman Filter Estimate', 'MATLAB kalmf Estimate', 'location', 'northwest');
xlabel('Time (s)');
ylabel('Position (m)');
title('Radar Target Tracking using Kalman Filter');
grid on;
