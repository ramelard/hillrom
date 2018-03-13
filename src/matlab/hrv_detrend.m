% HRV_DETREND   Detrend heart rate signal.
% From Tarvainen et al, "An advanced detrending method with application to
% HRV analysis", IEEE TBME 2002.
%   z_stat = hrv_detrend(z) - produce stationary estimate from signal z
%   Each column is a signal.
%   z_stat = hrv_detrend(z,lambda) - regulization parameter (penalizes
%   smoothness).
function [z_stat,z_trend] = hrv_detrend(z, lambda)

if nargin < 2
  lambda = 500;
end

if size(z,1) == 1
  z = z';
end

% T = length(z);
T = size(z,1);
I = speye(T);
D2 = spdiags(ones(T-2,1)*[1 -2 1],[0:2],T-2,T);
z_stat = (I-inv(I+lambda^2*(D2')*D2))*z;

if nargout > 1
  z_trend = z-z_stat;
end

% figure,subplot(211),plot(z),subplot(212),plot(z_stat)