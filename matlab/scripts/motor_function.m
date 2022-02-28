%% motor_function.m - Real system PI tuning
% Sript to read in CSV data from controls experiment, and determine system
% first order step response. The accuracy of the controller is shown, and a
% simulink model is used to create the PI tuned system response.
%
% Author: Josiah Smith (jsmith2@mines.edu)
% required files: real_motor_model.slx, control_revised.csv
%
%% Parse Experimental Data
% Read input CSV to two vectors, calculate time vector
%
array = csvread('control_revised.csv');
voltage = array(:,1);
velocity = array(:,2);

disp(size(velocity))
r_time = (1:size(velocity,1))/100;

%% Define motor parameters
Ra=1; % armaature resistance [Ohms]
Kt=.5; % motor torque constant [Nm/A]
Ke=.5; % back emf constant [Vs/rad]
J=.05; % Load inertia [Nm^2]
b=.5; % damping [Nm/s]

% Find transfer function approximation parameters
% Find K from motor's averaged final value
K = 13.45;
% Find sigma from motor model rise time, determined manually from CSV
% plots.
rise_time = 1.17 - 1.01;  
sigma = 2.2 / rise_time;

%% Simulate Model
%
% Simulate the selected model, showing the block diagram to display in the
% documentation. This model is used to compare our approximated step
% response with the real one, as well as to create a PI controller model
%
model = 'real_motor_model';
info = 'Simulating model: ';
disp(append(info, model))

open_system(model)
%
% run the simulation
%
out=sim(model);

%% A Plot of the results
% Plot measured step response (blue), and approximated transfer function's
% step response.
%
figure
subplot(2,1,1);
plot(r_time,velocity);
hold on;
plot(out.Open_Loop_Velocity)
title('Real Motor Open Loop Velocity Step Response vs Approximation');

xlabel('Time (seconds)')
ylabel('Velocity (rad/s)')
hold off;

subplot(2,1,2);
hold on;
plot(out.Closed_Loop_Position);
title('Motor Model PI Controlled Response');

xlabel('Time (seconds)')
ylabel('Position (rad)')
hold off;
