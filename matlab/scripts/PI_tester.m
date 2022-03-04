%% PI_tester.m - Verfiy Real PI Controller Results
% This script runs a simulation of a motor with bearing friction and plots the
% results
%
% Author: Josiah Smith (jsmith2@mines.edu)
% required file: PI_verification.slx
%
%% Initialize simulation parameters
% Find transfer function approximation parameters from motor_function.m
% Find K from motor's averaged final value
K = 13.45;
% Find sigma from motor model rise time, determined manually from CSV
% plots.
rise_time = 1.17 - 1.01;  
sigma = 2.2 / rise_time;

%% Run a Simulation
%
% Simulate ideal PID controller results vs discrete controller designed for
% Arduino
%
model = 'PI_verification';

info = 'Simulating model: ';
disp(append(info, model))

open_system(model)
%
% run the simulation
%
out=sim(model);

%% A Plot of the results
%
% Plot controller step response of both ideal and discrete controllers.
%
figure
plot(out.Position);
title('Ideal vs Arduino Controller Position Step Response');
xlabel('Time (seconds)');
ylabel('Position (rad)');
