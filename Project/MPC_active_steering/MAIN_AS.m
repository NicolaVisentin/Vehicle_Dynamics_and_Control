clear
%close all
clc

addpath('stuff')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Main script for the project - Vehicle Dynamics and Control              %
%                                                                         %
%   Vehicle Stability Control. Instructions: just run the code            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Data and parameters

g = 9.81;        % gravitational acceleration [m/s^2]
Vinit = 50/3.6;  % initialization velocity [m/s]
mu = 1;          % friction coefficient [-]

% Vehicle

m = 1380;      % vehicle mass [Kg]    
Iz = 2634.5;   % body inertia around z-axis [Kg m^2]
L = 2.79;      % wheelbase [m]
lf = 1.384;    % distance from front axle to CoG [m]
lr = L-lf;     % distance from rear axle to CoG [m]

i_steer = 15.4;   % steering ratio [-]

m_f = m*lr/L;   % front sprung mass [Kg]
m_r = m*lf/L;   % rear sprung mass [Kg]

Caf = 120000;           % front axle cornering stiffness
Car = 190000;           % rear axle cornering stiffness
Kus = m_f/Caf-m_r/Car;  % understeer gradient

% Second order TF identified from Sine Swept Test

wn = 11;      % yaw rate frequency
xi = 0.7;     % yaw rate damping
tau = 0.09;   % yaw rate time constant

max_steering_angle = 10*pi/180;  % maximum active steering angle [rad]
steering_rate_limit = 20*pi/180; % maximum steering rate [rad/s]

% Collect everything in a struct

par.m=m;     
par.Iz=Iz;
par.L=L;  
par.lf=lf;
par.lr=lr;   
par.g=g;
par.Vinit=Vinit;
par.i_steer=i_steer;
par.m_f=m_f;    
par.m_r=m_r;    
par.mu=mu;      
par.Caf=Caf;    
par.Car=Car;
par.Kus =Kus;
par.wn=wn; 
par.xi=xi;
par.tau=tau;
par.max_steering_angle=max_steering_angle;
par.steering_rate_limit=steering_rate_limit;

%% Choose type of simulation

% Choose controller

contr=3;

% Choose initial speed

answ=questdlg('Choose initial speed for the simulation.','Initial speed','60 Km/h','100 Km/h','60 Km/h');
switch answ
    case '60 Km/h'
        V_ref=60/3.6;
    case '100 Km/h'
        V_ref=100/3.6;
    otherwise
        V_ref=60/3.6;
end

% Choose noise

answ=questdlg('Include noise in the simulation?','Noise','Yes','No','No');
switch answ
    case 'No'
        noise=0;
    case 'Yes'
        noise=1;
    otherwise
        noise=0;
end

fprintf('SIMULATION:\n\tController: MPCAS\n\tInitial speed: %.0f Km/h\n\tNoise? %s\n\n',V_ref*3.6,answ)

%% MPC with AS

% Parameters

Dt=0.01;  % MPC sampling time
N=20;     % prediction horizon

% Weight matrices

Q=diag([0 3000000/1^2]);
R=diag([1/10000^2, 10/deg2rad(10)^2]);
P=diag([0 0]);

% Input constraints

DMz_min=-10000;
DMz_max=10000;
delta_as_min=-max_steering_angle;  % minimum active steering angle
delta_as_max=max_steering_angle;   % maximum active steering angle

U_min = [DMz_min*ones(N,1), delta_as_min*ones(N,1)];  % 20x2
U_max = [DMz_max*ones(N,1), delta_as_max*ones(N,1)];  % 20x2

MPC_constraints.U_min = U_min;
MPC_constraints.U_max = U_max;

% State constraints

x_min=[-100 -100]';
x_max=[100 100]';

% Send everything in structs

MPC_weights.Q=Q;
MPC_weights.R=R;
MPC_weights.P=P;

MPC_constraints.x_min=x_min;
MPC_constraints.x_max=x_max;

fprintf('DMz constraints: min=%.0f, max=%.0f\n', DMz_min, DMz_max);
fprintf('U_min size: %dx%d\n', size(U_min));
fprintf('U_max size: %dx%d\n', size(U_max));

%% Model run

% Create bus for simulink (vehicle parameters)

par_elems(1)=Simulink.BusElement; par_elems(1).Name='m';
par_elems(2)=Simulink.BusElement; par_elems(2).Name='Iz';
par_elems(3)=Simulink.BusElement; par_elems(3).Name='L';
par_elems(4)=Simulink.BusElement; par_elems(4).Name='lf';
par_elems(5)=Simulink.BusElement; par_elems(5).Name='lr';
par_elems(6)=Simulink.BusElement; par_elems(6).Name='g';
par_elems(7)=Simulink.BusElement; par_elems(7).Name='Vinit';
par_elems(8)=Simulink.BusElement; par_elems(8).Name='i_steer';
par_elems(9)=Simulink.BusElement; par_elems(9).Name='m_f';
par_elems(10)=Simulink.BusElement; par_elems(10).Name='m_r';
par_elems(11)=Simulink.BusElement; par_elems(11).Name='mu';
par_elems(12)=Simulink.BusElement; par_elems(12).Name='Caf';
par_elems(13)=Simulink.BusElement; par_elems(13).Name='Car';
par_elems(14)=Simulink.BusElement; par_elems(14).Name='Kus';
par_elems(15)=Simulink.BusElement; par_elems(15).Name='wn';
par_elems(16)=Simulink.BusElement; par_elems(16).Name='xi';
par_elems(17)=Simulink.BusElement; par_elems(17).Name='tau';
par_elems(18)=Simulink.BusElement; par_elems(18).Name='max_steering_angle';
par_elems(19)=Simulink.BusElement; par_elems(19).Name='steering_rate_limit';
parBus=Simulink.Bus;
parBus.Elements=par_elems;

% Create bus for simulink (MPC parameters)

weight_elems(1)=Simulink.BusElement; weight_elems(1).Name='Q'; weight_elems(1).Dimensions=[2 2];
weight_elems(2)=Simulink.BusElement; weight_elems(2).Name='R'; weight_elems(2).Dimensions=[2 2];
weight_elems(3)=Simulink.BusElement; weight_elems(3).Name='P'; weight_elems(3).Dimensions=[2 2];
weightsBus=Simulink.Bus;
weightsBus.Elements=weight_elems;

% Create bus for simulink (MPC constraints) 
constr_elems(1)=Simulink.BusElement; constr_elems(1).Name='U_min'; constr_elems(1).Dimensions=[N 2];
constr_elems(2)=Simulink.BusElement; constr_elems(2).Name='U_max'; constr_elems(2).Dimensions=[N 2];
constr_elems(3)=Simulink.BusElement; constr_elems(3).Name='x_min'; constr_elems(3).Dimensions=[2 1];
constr_elems(4)=Simulink.BusElement; constr_elems(4).Name='x_max'; constr_elems(4).Dimensions=[2 1];
constraintsBus=Simulink.Bus;
constraintsBus.Elements=constr_elems;

% Run simulation

sim('MPC_VSC_AS.slx')

%% Postprocessing

% Simulation data

t=time;
a_y=lat_acceleration;
v=lat_velocity;
u=long_velocity;
x=x_distance;
y=y_distance;
r=yaw_rate;
delta=steering_wheel_angle/i_steer;
dM=delta_yaw_moment;
delta_as=active_steering; 
r_ref=reference_yaw_rate;

t0=10;          % start time of SwD maneuver [s]
tf=11.9;        % end time of SwD maneuver [s]

start_plot=9;   % start time for the plots [s]
end_plot=14;    % end time for the plots [s]
plot_range=t>start_plot & t<end_plot;

% Performance evaluation

idx_y=find(t>=t0+1.07,1);
y_asses=y(idx_y);           % lateral disp at t=t0+1.07

idx_r1=find(t>=tf+1,1);
idx_r2=find(t>=tf+1.75,1);
r1_asses=r(idx_r1);         % yaw rate at t=tf+1
r2_asses=r(idx_r2);         % yaw rate at t=tf+1.75
r_peak=min(r);              % peak yaw rate

idx_t0=find(t>t0,1);
t_temp=t(idx_t0:end);
r_temp=r(idx_t0:end);
rref_temp=r_ref(idx_t0:end);
t_start_rmetric=t_temp(find(r_temp<=0,1));
idx_t_start_rmetric=find(t_temp>=t_start_rmetric,1);
t_integration=t_temp(idx_t_start_rmetric:end);
r_integration=r_temp(idx_t_start_rmetric:end);
rref_integration=rref_temp(idx_t_start_rmetric:end);
r_metric=trapz(t_integration,abs(r_integration-rref_integration))/trapz(t_integration,abs(rref_integration));  % yaw rate metric [-]
fprintf('RESUTLS:\n\tYaw rate metric = %.02f\n\n',r_metric)

% Plot steering angle, yaw rate and lateral displacement

figure

subplot(311)
plot(t(plot_range), delta(plot_range),'k','LineWidth',0.5)
grid on; box on
xlabel('t [s]', 'Interpreter','latex')
ylabel('$\delta$ [rad]','Interpreter','latex')
title('Steering angle')

subplot(312)
hold on
plot(t(plot_range), r(plot_range),'k','LineWidth',0.5)
plot(t(plot_range),r_ref(plot_range),'k--','LineWidth',0.5)
plot([tf+1 tf+1],[0.35*r_peak r_peak-0.2],'linewidth',2,color=[1 0 0 0.5])
plot([tf+1 tf+1],[0 0.35*r_peak],'linewidth',2,color=[0 1 0 0.5])
plot([tf+1.75 tf+1.75],[0.2*r_peak r_peak-0.2],'linewidth',2,color=[1 0 0 0.5])
plot([tf+1.75 tf+1.75],[0 0.2*r_peak],'linewidth',2,color=[0 1 0 0.5])
grid on; box on
xlabel('t [s]', 'Interpreter','latex')
ylabel('$\dot{\psi}$ [rad/s]','Interpreter','latex')
title('Yaw rate')
legend('$\dot{\psi}$','$\dot{\psi}_{ref}$','interpreter','latex','Location','best')

subplot(313)
hold on
plot(t(plot_range), y(plot_range),'k','LineWidth',0.5)
plot([t0+1.07 t0+1.07],[0 1.83],'linewidth',2,color=[1 0 0 0.5])
plot([t0+1.07 t0+1.07],[1.83 max(y)+2],'linewidth',2,color=[0 1 0 0.5])
grid on; box on
xlabel('t [s]', 'Interpreter','latex')
ylabel('y [m]','Interpreter','latex')
title('Lateral displacement')

% Plot yaw moment

figure
plot(t(plot_range), dM(plot_range),'k','LineWidth',0.5)
grid on; box on
xlabel('t [s]', 'Interpreter','latex')
ylabel('$\Delta M$ [Nm]','Interpreter','latex')
title('Yaw moment (added)')