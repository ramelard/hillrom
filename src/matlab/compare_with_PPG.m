function [Apost] = compare_with_PPG(R,tppg,ppg,fps,ppg_tidx)

if nargin < 5
  ppg_tidx = [];
end

tppg = tppg-tppg(1);

ppg_sample_rate = numel(tppg)/tppg(end);

% Sample PPG so its sample rate is the same as the camera frame rate.
% (Makes it easy for comparing).
xx = linspace(tppg(1),tppg(end),numel(R));
yy = spline(tppg,ppg,xx);

A = -log(R);
A(A==Inf) = 0;

figure;
plot(xx,A)
hold on
ppg_scaled = (yy-mean(yy))./std(yy).*std(A)+mean(A);
plot(xx,ppg_scaled,'r')
plot(xx(ppg_tidx), ppg_scaled(ppg_tidx), 'xr', 'MarkerSize', 10)
ylabel('absorbance (a.u.')
legend('PPGI','PPG')

[c,lags] = xcorr(A,yy);
figure;
plot(lags,c)
title('Cross Correlation')

Fhi = 3;  % 3 Hz cutoff
Ahat = denoise_LED_PPG(A,fps,'Fhi',Fhi);

A_detrend = hrv_detrend(A);
Ahat_detrend = hrv_detrend(Ahat);
[xest_detrend,xest] = clean_ippg(R);

figure;
ax = subplot(141);
plot_power_spectrum(ppg, ppg_sample_rate, ax);
title('PPG')
ax = subplot(142);
plot_power_spectrum(yy, fps, ax);
title('PPG (interpolated)')
ax = subplot(143);
plot_power_spectrum(A, fps, ax);
title('PPGI')
ax = subplot(144);
plot_power_spectrum(xest, fps, ax);
title('PPGI (detrend)')

figure;
t = (0:numel(R)-1)./fps;
subplot(321), plot(t,A), xlabel('t')
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(A)+mean(A),'r')
subplot(323), plot(t,Ahat), xlabel('t')
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Ahat)+mean(Ahat),'r')
subplot(325)
plot(t,xest)
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(xest)+mean(xest),'r')

% Detrend signals
subplot(322)
plot(t,A_detrend);
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(A_detrend)+mean(A_detrend),'r')
subplot(324)
plot(t,Ahat_detrend);
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Ahat_detrend)+mean(Ahat_detrend),'r')
subplot(326)
plot(t,xest_detrend);
hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(xest_detrend)+mean(xest_detrend),'r')

ax1=subplot(321); title(sprintf('correlation = %g\n',corr(A',yy')))
ax2=subplot(323); title(sprintf('Butterworth, correlation = %g\n',corr(Ahat',yy')))
ax3=subplot(325); title(sprintf('Kalman, correlation = %g\n',corr(xest',yy')))
ax4=subplot(322); title(sprintf('Detrend, correlation = %g\n',corr(A_detrend',yy')))
ax5=subplot(324); title(sprintf('Detrend, correlation = %g\n',corr(Ahat_detrend',yy')))
ax6=subplot(326); title(sprintf('Detrend, correlation = %g\n',corr(xest_detrend',yy')))
linkaxes([ax1 ax2 ax3 ax4 ax5 ax6], 'x')

if exist('ppgtemplate.mat','file')
  load ppgtemplate.mat
  [c,lags] = xcorr(xest_detrend, ppgtemplate);
  figure;
  plot(lags,c)
  title('Cross Correlation with Template')
end

% Spectral coherence (http://www.mathworks.com/help/signal/examples/measuring-signal-similarities.html)
[Cxy,f] = mscohere(xest_detrend, yy, [], [], [], fps);
Pxy     = cpsd(xest_detrend,yy,[],[],[],fps);
phase   = -angle(Pxy)/pi*180;
thresh = 0.4;
[pks,locs] = findpeaks(Cxy,'MinPeakHeight',thresh);
figure
subplot(211);
plot(f,Cxy);
xlabel('Frequency (Hz)')
title('Coherence Estimate');
grid on;
hgca = gca;
set(hgca,'XTick',f(locs));
set(hgca,'YTick',thresh);
axis([0 50 0 1])
subplot(212);
plot(f,phase);
title('Cross Spectrum Phase (deg)');
grid on;
hgca = gca;
set(hgca,'XTick',f(locs))
set(hgca,'YTick',sort(round(phase(locs))));
xlabel('Frequency (Hz)');
axis([0 50 -180 180])

resp_rate_from_ppg(t,xest_detrend);

if nargout > 0
  Apost = xest_detrend;
end