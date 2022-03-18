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

% Rho system step response
array = csvread('robot_crawl_v4.csv');
rho_dot = array(:,2)./2;

disp(size(rho_dot))
real_time = (1:size(rho_dot,1))/100;

% Phi system step response
array = csvread('robot_turn_v4.csv')./2;
phi_dot = array(:,3);

%% Define motor parameters
% Find transfer function approximation parameters
% Find K from motor's averaged final value
%
rho_K = 0.0039724/2;
% Find sigma from motor model rise time, determined manually from CSV
% plots.
rho_rise_time = 1.15 - 1; 
rho_sigma = 2.2 / rho_rise_time;

% Find phi response
phi_K = 0.0124314/2;
phi_rise_time = 1.15 - 1;
phi_sigma = 2.2 / phi_rise_time;

%% Simulate Model
%
% Simulate the selected model, showing the block diagram to display in the
% documentation. This model is used to compare our approximated step
% response with the real one, as well as to create a PI controller model
%
model = 'robot_tuning';
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
plot(real_time, smooth(rho_dot));
hold on;
plot(out.Open_Vel_Forward);
title('Motor Same Voltage Step Response (Forward)');

xlabel('Time (seconds)')
ylabel('Rho (v1+v2)/2')
hold off;

subplot(2,1,2);
plot(real_time, -1.*smooth(phi_dot));
hold on;
plot(out.Open_Vel_Turn);
title('Motor Different Voltage Step Response (Turning) ');
hold off;
