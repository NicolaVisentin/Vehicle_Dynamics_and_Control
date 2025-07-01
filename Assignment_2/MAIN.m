clear
close all
clc

addpath('stuff')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Main script for homework 2 - Vehicle Dynamics and Control               %
%                                                                         %
%   Vehicle Stability Control. Instructions: just run the code            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Choose type of simulation (run code: dialog box will appear)

% Choose controller
answ=questdlg('Choose controller for the simulation.','Controller choice','off','PD','LQR','off');
switch answ
    case 'LQR'
        contr='LQR';
        VSC_on=1;
    case 'PD'
        contr='PD';
        VSC_on=1;
    case 'off'
        contr='off';
        VSC_on=0;
    otherwise
        contr='off';
        VSC_on=0;
end

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

fprintf('SIMULATION:\n\tController: %s\n\tInitial speed: %.0f Km/h\n\tNoise? %d\n\n',contr,V_ref*3.6,noise)

%% Non-tunable parameters

par.g = 9.81;
par.Vinit   = 50 /3.6;              % initialization velocity, Don't TUNE

% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.l_r      = par.L - par.l_f;     % distance from rear axle to CoG, m

% Steering
par.i_steer  = 15.4;                % steering ratio, -

% Additional
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
par.mu       = 1;                               % friction coefficient, -

%% Tunable parameters

% Reference Generator
par.Calpha_front = 120000;          % front axle cornering stiffness
par.Calpha_rear  = 190000;          % rear axle cornering stiffness
par.Kus = par.m_f/par.Calpha_front - par.m_r/par.Calpha_rear; % understeer gradient

% Second order TF identified from Sine Swept Test
par.wn      = 11;                   % yaw rate frequency
par.kseta   = 0.7;                  % yaw rate damping
par.tau     = 0.09;                 % yaw rate time constant

% Add/ Change after this line

%% Controllers

% PD

if V_ref-60/3.6 < eps  % 60 Km/h
    Kp=30000;
    Kd=20;
else                   % 100 Km/h
    Kp=20000;
    Kd=100;
end

% LQR

N=20; 
u_sched=linspace(60,100,N)/3.6;
K_sched=zeros(N,2);

alpha=1000;
B=[0; 1/par.Izz];
for ii=1:length(u_sched)
    u=u_sched(ii);
    A=ComputeA(u,par);
    R=u/10000^2;
    Q=diag([0 alpha/1^2]);
    [K,~,~]=lqr(A,B,Q,R);
    K_sched(ii,:)=K;
end
Kv_sched=K_sched(:,1)';
Kr_sched=K_sched(:,2)';

%% Model run

if strcmp(contr,'PD')
    sim('simulink_PD.slx')
else
    sim('simulink_LQR.slx')
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
delta=steering_wheel_angle/par.i_steer;
dM=delta_yaw_moment;
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

%% Plots for the report

% % Plot steering angle, yaw rate, lateral displacement and yaw moment
% 
% figure
% 
% subplot(221)
% plot(t(plot_range), delta(plot_range),'k','LineWidth',0.5)
% grid on; box on
% xlabel('t [s]', 'Interpreter','latex')
% ylabel('$\delta$ [rad]','Interpreter','latex')
% title('Steering angle')
% xlim([9 14])
% 
% subplot(223)
% hold on
% plot(t(plot_range), r(plot_range),'k','LineWidth',0.5)
% plot(t(plot_range),r_ref(plot_range),'k--','LineWidth',0.5)
% plot([tf+1 tf+1],[0.35*r_peak r_peak-0.2],'linewidth',2,color=[1 0 0 0.5])
% plot([tf+1 tf+1],[0 0.35*r_peak],'linewidth',2,color=[0 1 0 0.5])
% plot([tf+1.75 tf+1.75],[0.2*r_peak r_peak-0.2],'linewidth',2,color=[1 0 0 0.5])
% plot([tf+1.75 tf+1.75],[0 0.2*r_peak],'linewidth',2,color=[0 1 0 0.5])
% grid on; box on
% xlabel('t [s]', 'Interpreter','latex')
% ylabel('$\dot{\psi}$ [rad/s]','Interpreter','latex')
% title('Yaw rate')
% legend('$\dot{\psi}$','$\dot{\psi}_{ref}$','interpreter','latex','Location','best')
% xlim([9 14])
% 
% subplot(222)
% hold on
% plot(t(plot_range), y(plot_range),'k','LineWidth',0.5)
% plot([t0+1.07 t0+1.07],[0 1.83],'linewidth',2,color=[1 0 0 0.5])
% plot([t0+1.07 t0+1.07],[1.83 max(y)+2],'linewidth',2,color=[0 1 0 0.5])
% grid on; box on
% xlabel('t [s]', 'Interpreter','latex')
% ylabel('y [m]','Interpreter','latex')
% title('Lateral displacement')
% xlim([9 14])
% 
% subplot(224)
% plot(t(plot_range), dM(plot_range),'k','LineWidth',0.5)
% grid on; box on
% xlabel('t [s]', 'Interpreter','latex')
% ylabel('$dM_z$ [Nm]','Interpreter','latex')
% title('Yaw moment (added)')
% xlim([9 14])
% 
% % Plot saturation block
% 
% figure
% hold on
% plot(t(plot_range),r_ss(plot_range),'k--')
% plot(t(plot_range),r_sat(plot_range),'k')
% grid on; box on
% xlabel('t [s]', 'Interpreter','latex')
% ylabel('$\dot{\psi}$ [rad/s]','Interpreter','latex')
% title('Effect of the ay-limiter')
% legend('$\dot{\psi}_{ss}$','$\dot{\psi}_{sat}$','interpreter','latex','Location','best')
% xlim([9 14])
% hold off