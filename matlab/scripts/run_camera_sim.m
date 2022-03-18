%% run_camera_sim.m
% Initialize and run camera controls simulation to show robot response.
%
model = 'camera';
info = 'Simulating model: ';

%% Initilize constants
% Set up simulation parameters for robot
%
rhodot_s = 1;
x_s = [0 3 6 10];
y_s = [0 0 3 3];

disp(append(info, model))

%% Run the Simulation
% Open and run the model, collecting response to plot it
%
open_system(model)
%
% run the simulation
%
out = sim(model);

%% Plot the Results
% Show the robot's drive path, as well as X and Y responses. 
%
figure

subplot(3,1,1)
hold on;
plot(out.X.Data, out.Y.Data)
plot(x_s,y_s,'*-')
legend('Real', 'Goal')
hold off;


subplot(3,1,2)
hold on;
plot(out.X)
plot(out.Y)
legend('X', 'Y')
hold off;

subplot(3,1,3)
hold on;
plot(out.Phi)
plot(out.Phi_s)
legend('Real', 'Goal')
hold off;

