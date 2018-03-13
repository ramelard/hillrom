% KALMANFILT   Kalman filter
%   xest = kalmanfilt(xhat, sigmap, sigmam)
%     For now, assumes 2D position, velocity setup.
% For codegen: 
%   codegen -args {coder.typeof(double(0), [1 Inf]) double(0) double(0)} kalmanfilt.m
%#codegen
function xest = kalmanfilt(z, sigmap, sigmam)

if nargin < 2
  sigmap = 3;  % process error, stddev
  sigmam = 0.01;  % measurement error, stddev
end
if nargin < 1
  t = 0:.001:1;
  x = sin(2*pi*3*t);  % true signal (cannot observe)
  xhat = x+sigmap*randn(size(x));  % actual signal output
  z = xhat + sigmam*randn(size(xhat));  % measured signal
end

Q = [1/4 1/2; 1/2 1]*sigmap^2;  % process noise covariance
R = sigmam^2;  % measurement noise variance

A = [1 1; 0 1];  % state transition matrix
B = [1/2; 1];
u = 0;
H = [1 0];  % observation matrix

xprior = [z(1); 0];  % x,v initially
Pprior = [1 0; 0 1];  % we are certain, thus 0 stddev

% Posterior estimate of signal x
xpost = zeros(2,size(z,2));
Ppost = [];
for k = 1:size(z,2)
  % Correct/Update (supports multiple observations)
  for o = 1:size(z,1)  % number of observations
    K = Pprior*H'*pinv(H*Pprior*H' + R);  % Kalman gain
    xpost(:,k) = real(xprior + K*(z(o,k)-H*xprior));  % estimate x using msmt data
    Ppost = (eye(2) - K*H)*Pprior;  % update posterior error covariance
  end

  % Predict
  xprior = A*xpost(:,k) + B*u;  % build prior based on previous position
  Pprior = A*Ppost*A' + Q;  % build prior error covariance
end


% figure, hold on
% plot(x,'--k')
% plot(xhat,'-r')
% plot(z,'-g')
% plot(xpost(1,:),'b','LineWidth',2)
% legend('x','xhat','z','xest')

xest = xpost(1,:);