clear
close all
clc
clear mex

addpath('stuff')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Main script for homework 3 - Vehicle Dynamics and Control               %
%                                                                         %
%   Instructions:                                                         %
%       1. Run this script                                                %
%                                                                         %
%   ! For the Simulink impletmenation refer to 'HW03_kinematic.slx' (for  %
%     the kinematic model) and to 'HW03_dynamic.slx' (for the             %
%     dynamic model) in the 'stuff' folder                                %
%   ! Initialization script is 'HW03_initialization.m' in 'stuff' folder, %
%     but it is automatically called by this script                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Choose simulation

% Choose vehicle model for the MPC

sim_model=questdlg('Choose vehicle model for the MPC.','Model choice','kinematic','dynamic','kinematic');

% Set MPC weights

switch sim_model
    case 'kinematic'
        Qy=inputdlg('Choose penalization on the lateral position y (default: 0.005).','Weights choice',[1 35],{'0.005'});
        R=inputdlg('Choose penalization on the steering control delta (default: 0.1).','Weights choice',[1 35],{'0.1'});
    case 'dynamic'
        Qy=inputdlg('Choose penalization on the lateral position y (default: 0.005).','Weights choice',[1 35],{'0.005'});
        R=inputdlg('Choose penalization on the steering rate control delta dot (default: 0.0001).','Weights choice',[1 35],{'0.0001'});
end
Qy=str2double(Qy);
R=str2double(R);

%% Run simulation

% Initialize the model/ACADO/variables
cd stuff
HW03_initialization
cd ..
close_system('untitled', 0);

% Run the correct Simulink file
switch sim_model
    case 'kinematic'
        sim('stuff\HW03_kinematic.slx');
    case 'dynamic'
        sim('stuff\HW03_dynamic.slx');
    otherwise
        sim('stuff\HW03_kinematic.slx')
end
clc

%% Results and plots

% Extract results
delta_sim=squeeze(delta_sim);
delta_d_sim=squeeze(delta_d_sim);
y_ref_sim=squeeze(y_ref_sim);
y_sim=squeeze(y_sim);
t=tout(1):Ts:tout(end);  % scale time: MPC runs at 100 Hz, simulation runs at 1000 Hz, sim data are at 100 Hz

% Plot steering input and lateral position
figure

subplot(311)
hold on
plot(t,y_sim,'k')
plot(t,y_ref_sim,'r')
grid on; box on
xlabel('t [s]', Interpreter='latex')
ylabel('$y$ [m]', Interpreter='latex')
title('Lateral position')
legend('$y$','$y_{ref}$','interpreter','latex','Location','best')
hold off

subplot(312)
plot(t,delta_sim,'k')
grid on; box on
xlabel('t [s]', Interpreter='latex')
ylabel('$\delta$ [deg]', Interpreter='latex')
title('Steering angle')

subplot(313)
plot(t,delta_d_sim,'k')
grid on; box on
xlabel('t [s]', Interpreter='latex')
ylabel('$\dot{\delta}$ [deg/s]', Interpreter='latex')
title('Steering angle rate')

% RMSE on the lateral position tracking
RMSE=sqrt(mean((y_ref_sim-y_sim).^2));
fprintf('RMSE on lateral position tracking: %.4f m\n',RMSE)