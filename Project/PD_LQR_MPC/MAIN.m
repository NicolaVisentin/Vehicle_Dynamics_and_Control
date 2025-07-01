clear
close all
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
Kus = m*g*(lr/Caf-lf/Car)/L;  % understeer gradient

% Second order TF identified from Sine Swept Test

wn = 11;      % yaw rate frequency
xi = 0.7;     % yaw rate damping
tau = 0.09;   % yaw rate time constant

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

%% Choose type of simulation

% Choose controller

contr=listdlg( ...
    'PromptString', 'Choose controller for the simulation:', ...
    'SelectionMode', 'single', ...
    'ListString', {'PD','LQR','MPC','off'}, ...
    'InitialValue', 1, ...
    'Name', 'Controller Choice', ...
    'ListSize', [300 70]);
choice_list={'PD','LQR','MPC','off'};

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

fprintf('SIMULATION:\n\tController: %s\n\tInitial speed: %.0f Km/h\n\tNoise? %s\n\n',choice_list{contr},V_ref*3.6,answ)

%% PD

if V_ref-60/3.6 < eps  % 60 Km/h
    Kp=30000;
    Kd=20;
else                   % 100 Km/h
    Kp=20000;
    Kd=100;
end

%% LQR

N=21; 
u_sched=linspace(60,100,N)/3.6;
K_sched=zeros(N,2);

B=[0; 1/Iz];
for ii=1:length(u_sched)
    u=u_sched(ii);
    A=ComputeA(u,par);
    R=1*u/10000^2;
    Q=diag([0 700/1^2]);
    [K,~,~]=lqr(A,B,Q,R);
    K_sched(ii,:)=K;
end
Kv_sched=K_sched(:,1)';
Kr_sched=K_sched(:,2)';

%% MPC

% Parameters

Dt=0.01;  % MPC sampling time
N=20;     % prediction horizon

% Weight matrices

Q=diag([0 700/1^2]);
R=10/10000^2;
P=diag([0 0]);

% Input constraints

DMz_min=-10000;  % minimum added moment
DMz_max=10000;   % maximum added moment
U_min=DMz_min*ones(N,1);
U_max=DMz_max*ones(N,1);

% State constraints

x_min=[-101 -0.5]';
x_max=[101 0.5]';

% Send everything in structs

MPC_weights.Q=Q;
MPC_weights.R=R;
MPC_weights.P=P;

MPC_constraints.U_min=U_min;
MPC_constraints.U_max=U_max;
MPC_constraints.x_min=x_min;
MPC_constraints.x_max=x_max;

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
parBus=Simulink.Bus;
parBus.Elements=par_elems;

% Create bus for simulink (MPC parameters)

weight_elems(1)=Simulink.BusElement; weight_elems(1).Name='Q'; weight_elems(1).Dimensions=size(Q);
weight_elems(2)=Simulink.BusElement; weight_elems(2).Name='R'; weight_elems(2).Dimensions=size(R);
weight_elems(3)=Simulink.BusElement; weight_elems(3).Name='P'; weight_elems(3).Dimensions=size(P);
weightsBus=Simulink.Bus;
weightsBus.Elements=weight_elems;

constr_elems(1)=Simulink.BusElement; constr_elems(1).Name='U_min'; constr_elems(1).Dimensions=size(U_min);
constr_elems(2)=Simulink.BusElement; constr_elems(2).Name='U_max'; constr_elems(2).Dimensions=size(U_max);
constr_elems(3)=Simulink.BusElement; constr_elems(3).Name='x_min'; constr_elems(3).Dimensions=size(x_min);
constr_elems(4)=Simulink.BusElement; constr_elems(4).Name='x_max'; constr_elems(4).Dimensions=size(x_max);
constraintsBus=Simulink.Bus;
constraintsBus.Elements=constr_elems;

% Run simulation

switch contr
    case {1,2,4}
        sim('PD_LQR_VSC.slx')
    case 3
        sim('MPC_VSC.slx')
end

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
r_ref=reference_yaw_rate;

t0=10;          % start time of SwD maneuver [s]
tf=11.9;        % end time of SwD maneuver [s]

start_plot=9;   % start time for the plots [s]
end_plot=14;    % end time for the plots [s]
plot_range=t>start_plot & t<end_plot;

% Performance evaluation

% pass/fail on y
idx_y=find(t>=t0+1.07,1);
y_asses=y(idx_y);           % lateral disp at t=t0+1.07

% pass/fail on r
idx_r1=find(t>=tf+1,1);
idx_r2=find(t>=tf+1.75,1);
r1_asses=r(idx_r1);         % yaw rate at t=tf+1
r2_asses=r(idx_r2);         % yaw rate at t=tf+1.75
r_peak=min(r);              % peak yaw rate

% yaw velocity metric
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

% Plot control input

figure
plot(t(plot_range), dM(plot_range),'k','LineWidth',0.5)
grid on; box on
xlabel('t [s]', 'Interpreter','latex')
ylabel('$\Delta M_z$ [Nm]','Interpreter','latex')
title('Yaw moment (added)')