%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: PositionController.slx
%
%% Define motor parameters
K=0.15; % DC gain [rad/Vs]
sigma=9; % time constant reciprocal [1/s]
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('PositionController')
%
% run the simulation
%
out=sim('PositionController');
%% A Plot of the results
%
figure
%set(gcf, 'Position', [20,20,550,400])
plot(out.DesiredPosition, '--', 'linewidth',2)
hold on
plot(out.Position,'linewidth',2)
xlabel('Time (s)')
ylabel('Position (m)')


