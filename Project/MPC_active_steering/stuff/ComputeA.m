function A=ComputeA(u,par)
%
%   A=ComputeA(u,par)
%
% Computes A matrix
%   dx(t) = A * x(t) + B * delta(t)
% or
%   dx(t) = A * x(t) + BM * DMz(t)


% Extract parameters
m=par.m;
Caf=par.Caf;
Car=par.Car;
lf=par.lf;
lr=par.lr;
Iz=par.Iz;

% Compute A matrix
A=zeros(2,2);
A(1,1)=-(Caf+Car)/(m*u);
A(1,2)=(lr*Car-lf*Caf)/(m*u)-u;
A(2,1)=(lr*Car-lf*Caf)/(Iz*u);
A(2,2)=-(lf^2*Caf+lr^2*Car)/(Iz*u);

end