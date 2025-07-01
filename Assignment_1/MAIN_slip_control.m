clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Main script for homework 1 - Vehicle Dynamics and Control               %
%                                                                         %
%   Wheel slip control (some parameters, in particular noise, ABS on/off, %
%   and filtering, need to be adjusted in the Simulink file)              %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Data and parameters definition

% System parameters

par.mass=450;          % quarter car mass [kg]
par.Iw=1.2;            % inertia of the wheel [kg*m^2]
par.Reff=0.305;        % wheel effective radius [m]
par.g=9.81;           
par.Pres2Moment=11.25; % convertion from brake pressure to brake torque
par.max_pressure=160;  % max. brake pressure [bar]
k_ref=0.12;            % reference slip [-]

k_thresh=0.001;        % ABS activation threshold [-]

% Maneuver settings

par.V0=120/3.6;        % Initial speed [m/s]
par.Vmin=10/3.6;       % Minimal speed to stop simulation [m/s]
par.friction=0.6;      % Friction coefficient [-]

% PID parameters

Kp_high=500;       % proportional gain (for high speeds)
Kp_low=100;        % proportional gain (for low speeds)
%Kp_thresh=70/3.6;  % speed threshold for gain scheduling [m/s]

Ki_high=30;        % integral gain (for high speeds) 
Ki_low=70;         % integral gain (for low speeds)
Ki_thresh=50/3.6;  % speed threshold for gain scheduling [m/s]
Kb=0.8;            % anti-wind up parameter (back-calculation gain)

Kd_high=5;        % derivative gain (for high speeds)
Kd_low=5;         % derivative gain (for low speeds)
Kd_thresh=0/3.6;  % speed threshold for gain scheduling [m/s]
N=200;            % derivative filtering

%% Model run

sim('slip_control')

%% Postprocessing

% Outputs

t=time;                       % time [s]
v=chassis_speed;              % chassis speed [m/s]
w=wheel_speed;                % wheel rotational speed [rad/s]
p=brake_pressure;             % brake pressure effectively applied [bar]
delta_p=pressure_correction;  % controller's output [bar]
k=slip;                       % slip [-]
x=chassis_position;           % vehicle position [m]
jerk=jerk;                    % jerk [m/s^3]

v_wheel=w*par.Reff;           % wheel linear speed [m/s]
indx=find(t>=2,1);
ITAE_jerk=trapz(t(indx:end),t(indx:end).*abs(jerk(indx:end)));  % ITAE on the longitudinal jerk
clc
fprintf('ITAE_jerk = %.01f m/s^2\n\n',ITAE_jerk)

% Plots - velocities, pressures, slip

figure

subplot(311)
hold on
plot(t,v,'k')
plot(t,v_wheel,'b')
grid on; box on
xlabel('time [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Velocities')
legend('chassis','wheel','interpreter','latex','Location','best')
axis([0 10 0 40])
hold off

subplot(312)
hold on
plot(t,p,'b')
plot(t,delta_p,'r')
grid on; box on
xlabel('time [s]','Interpreter','latex')
ylabel('p [bar]','Interpreter','latex')
title('Brake pressure')
legend('applied pressure','pressure correction','interpreter','latex','Location','best')
xlim([0 10])
ylim([-100 100])
hold off

subplot(313)
hold on
plot(t,slip,'b')
yline(k_ref,'b--')
grid on; box on
xlabel('time [s]','Interpreter','latex')
ylabel('$\kappa$ [-]','Interpreter','latex')
title('Longitudinal slip')
legend('$\kappa$','$\kappa_{ref}$','interpreter','latex','Location','best')
xlim([0 10])
ylim([0 1])
hold off

% Braking distance plot

figure
plot(t,x-2*par.V0,'b')
yline(0)
yline(x(end)-2*par.V0,'b--')
grid on; box on
xlabel('time [s]','Interpreter','latex')
ylabel('x [m]','Interpreter','latex')
title('Vehicle position')
legend('vehicle position','',sprintf('braking distance=%.2f m',x(end)-2*par.V0),'interpreter','latex','Location','best')
xlim([0 10])