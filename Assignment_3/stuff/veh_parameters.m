%% Non-tunable parameters
par.g = 9.81;
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
par.Caf  = 120000;                  % Front axle cornering stiffness
par.Car = 190000;                   % Rear axle cornering stiffness
par.Kus = par.m_f/par.Caf - par.m_r/par.Car; % understeer gradient
V_ref = 100 /3.6;                    % pre-maneuver speed, km/h
