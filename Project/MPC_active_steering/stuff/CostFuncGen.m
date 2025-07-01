function [H,h]=CostFuncGen(T,S,Q,R,P)

% Cost function generation. Computes cost function V(U) for dense MPC 
% formulation:
%   V(U) = 0.5*U'*H*U + (h*x0)'*U + const
% from a typical quadratic cost function:
%   V(Y,U) = 0.5 * sum{k=0,N-1}( yk'*Q*yk + uk'*R*uk ) + 0.5 * yN'*P*yN 
% where:
%  -Y=[y(0)' y(1)' y(2)' ... y(N)']: outputs sequence from time k=0 to
%                                    k=N | dim=((N+1)*ny,1)
% -U=[u(0)' u(1)' u(2)' ... u(N-1)']: input sequence from time k=0 to time
% k=N-1 where u(k) = [DMz(k); delta_as(k)] | dim=(N*nu,1) with nu=2
% for a linear discrete time invariant dynamics (dense formulation):
% Y=T*x0+S*U
% 
% Syntax:
%
%   [H,h]=CostFuncGen(T,S,Q,R,P)
%
% Inputs
%  -T...........T prediction matrix (from initial state) | dim=((N+1)*ny,nx)
%  -S...........S prediction matrix (from input) | dim=((N+1)*ny,N*nu) 
%  -Q,R,P.......weighting matrices of the cost function | dimQ=(nx,nx), 
%               dimR=(nu,nu), dimP=(nx,nx)
% Outputs:
%  -H,h.........matrices for the dense formulation of the cost function


[nu, ~] = size(R);
N=size(S,2)/nu;

assert(all(isfinite(S(:))), 'S contains NaN or Inf');
assert(all(isfinite(Q(:))), 'Q contains NaN or Inf');
assert(all(isfinite(R(:))), 'R contains NaN or Inf');
assert(all(isfinite(P(:))), 'P contains NaN or Inf');

Q_bar = kron(eye(N),Q);
Q_bar = blkdiag(Q_bar,P);  % now of size ((N+1)*ny x (N+1)*ny)
R_bar = kron(eye(N),R);

assert(all(isfinite(Q_bar(:))), 'Q_bar contains NaN or Inf');
assert(all(isfinite(R_bar(:))), 'R_bar contains NaN or Inf');

H = S'*Q_bar*S + R_bar;
assert(all(isfinite(H(:))), 'H has NaN or Inf');
h=S'*Q_bar'*T;

H=(H+H')/2;   % enforce symmetry on H

% NOTE: only H and h matrices are given, while "const" is not needed. In fact
%    const=0.5*x0'*(T'*Q_bar*T)*x0;
% however, x0 is given (measured state), so it's constant, it's not
% something we need to minimize in our optimisation, so we can not account
% for it.

end
