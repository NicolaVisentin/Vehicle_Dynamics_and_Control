function [Ad,Bd]=ComputeDiscretizedMatrices(A,B,Dt)
%
%   [Ad,Bd]=ComputeDiscretizedMatrices(A,B,Dt)
%
% Computes discrete time matrices 
%   x(k+1) = Ad * x(k) + Bd * u(k)
% with discretization time Dt from continuous time matrices
%   dx(t) = A * x(t) + B * u(t)


% Matrices dimensions
nx=length(A);
nu=size(B,2);

% Discretize
expmat=expm(Dt*[A B; zeros(nu,nx) zeros(nu,nu)]);
Ad=expmat(1:nx,1:nx);
Bd=expmat(1:nx,nx+1:end);

end