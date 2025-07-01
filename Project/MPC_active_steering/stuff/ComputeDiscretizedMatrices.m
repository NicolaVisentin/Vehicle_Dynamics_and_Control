function [Ad,BMd]=ComputeDiscretizedMatrices(A,BM,Dt)
%
%   [Ad,BMd]=ComputeDiscretizedMatrices(A,BM,Dt)
%
% Computes discrete time matrices 
%   x(k+1) = Ad * x(k) + BMd * DMz(k)
% with discretization time Dt from continuous time matrices
%   dx(t) = A * x(t) + BM * DMz(t)

n = size(A, 1);  % number of states
m = size(BM, 2);  % number of inputs

expmat = expm(Dt * [A, BM; zeros(m, n + m)]);

% Extract discretized matrices
Ad = expmat(1:n, 1:n);
BMd = expmat(1:n, n+1:n+m);

end