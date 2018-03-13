% z = -log(R);
% sigmap = 0.03;
% sigmam = 0.0037;

% sigmam = .1;
% sigmap = 0;
% z = 0.4*ones(1,1000) + sigmam*randn(1,1000);
dt = .1;
duration = 10;


sigmap = 0;  % process error, stddev
sigmam = 0.0037;  % measurement error, stddev
t = 0:dt:duration;
x = sin(2*pi*3/duration*t);  % true signal (cannot observe)
xhat = x+sigmap*randn(size(x));  % actual signal output
z = xhat + sigmam*randn(size(xhat));  % measured signal

Q_loc = xhat;
Q_loc_meas = z;

xest = kalmanfilt(z, sigmap, sigmam);

figure, hold on
plot(0:dt:duration,x,'-r.')
plot(0:dt:duration,z,'-k.')
plot(0:dt:duration,xest,'-g.')
xlim([0 duration])
legend('x','z','xest')

%%
sigmap = 3.7E-5;
sigmam = 0.0037;
xest = kalmanfilt(z, sigmap, sigmam);

figure, hold on
plot(z,'Color',[0.5 0.5 0.5])
plot(xest,'-r','LineWidth',2)
xlim([0 numel(z)])
legend('z','xest')
title(sprintf('Kalman, \\sigma_p=%0.2g',sigmap))

%% Parametric grid plot
sigmam = .0037;  % measurement error, stddev
xx = linspace(tppg(1),tppg(end),numel(R));
yy = spline(tppg,ppg,xx);
z = R(1:numel(R)/2);
ztrue = yy(1:numel(yy)/2);
ztrue_white = (ztrue-mean(ztrue))./std(ztrue).*std(-log(z))+mean(-log(z));

figure;
idx = 1;
for sigmap = logspace(-5, -3, 15)
  xest = kalmanfilt(z, sigmap, sigmam);

  subplot(3,5,idx), hold on
  plot(-log(z),'k')
  plot(-log(xest),'-r','LineWidth',2)
  plot(ztrue_white,'Color',[0.5 0.5 0.5],'LineStyle','--')
  xlim([0 numel(z)])
  legend('z','xest','ztrue')
  title(sprintf('Kalman, \\sigma_p=%0.2g, \\rho=%0.2g',sigmap,corr(ztrue',-log(xest)')))

  idx = idx + 1;
end