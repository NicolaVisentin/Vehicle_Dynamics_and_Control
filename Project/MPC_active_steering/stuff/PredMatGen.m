function [T,S]=PredMatGen(A,B,C,N)

% Prediction matrices generation. Computes prediction matrices T and S for
% dense MPC formulation:
%   Y = T*x0 + S*U
% where:
%  -Y=[y(0)' y(1)' y(2)' ... y(N)']: outputs sequence from time k=0 to
%                                    k=N | dim=((N+1)*ny,1);
%  -x0: initial condition vector | dim=(nx,1);
%  -U=[u(0)' u(1)' u(2)' ... u(N-1)']: input sequence from time k=0 to time
%                                      k=N-1 | dim=(N*nu,1)
% 
% Syntax:
%
%   [T,S]=PredMatGen(A,B,C,N)
%
% Inputs
%  -A,B,C...matrices A (nx,nx), B (nx,nu), C (ny,nx) that define the discrete
%           time, linear time invariant system
%  -N.......number of discrete time steps
% Outputs:
%  ! TO HAVE S AND T AS STATE PREDICTION MATRICES, JUST USE C=I, OBVIOUSLY
%  -T.......Ty output prediction matrix (initial state) | dim=((N+1)*ny,nx)
%  -S.......Sy output prediction matrix (input) | dim=((N+1)*ny,N*nu)


% Compute dimensions
nx=length(A);
nu=size(B,2);
ny=size(C,1);

% Create matrix T
T=zeros((N+1)*ny,nx);
for k=0:N
    T(k*ny+1:(k+1)*ny,:)=C*A^k;
end

% Create matrix S
S=zeros((N+1)*ny,N*nu);
for k=1:N
    for ii=0:k-1
        S(k*ny+1:(k+1)*ny,ii*nu+1:(ii+1)*nu)=C*A^(k-1-ii)*B;
    end
end

end