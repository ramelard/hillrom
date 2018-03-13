% Plots: Raw, Ambient-corrected, Denoised, Detrended
% Figures: Raw, Ambient-corrected

function comp_plots(im_i, im_inorm, fps, ppg, tppg, varargin)

PPG_PLOT_HEIGHT = 200;

zoom_first = false;
plot_ppg = false;
i = 6;
while i < nargin
  pname = varargin{i};
  if strcmpi(pname,'zoom')
    zoom_first = true;
  elseif strcmpi(pname,'plot_ppg')
    plot_ppg = true;
  end
  i = i + 1;
end

rect = [];

[Rcorr,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect,'zoom',zoom_first);
[Rraw,rect] = plot_pixel_vals(im_i,'fps',fps,'roi','rect',rect,'zoom',zoom_first);

close all

% Raw
hraw = figure;
imshow(im_i(:,:,1));
r = rectangle('position', rect);
set(r, 'edgecolor', 'r');

% Ambient-corrected
hcorr = figure;
imshow(im_inorm(:,:,1))
r = rectangle('position', rect);
set(r, 'edgecolor', 'r');


tppg = tppg-tppg(1);

% Sample PPG so its sample rate is the same as the camera frame rate.
% (Makes it easy for comparing).
xx = linspace(0,tppg(end),numel(Rraw));
yy = spline(tppg,ppg,xx);

Araw = -log(Rraw);
Araw(Araw==Inf) = 0;
Acorr = -log(Rcorr);
Acorr(Acorr==Inf) = 0;

% sigmam = 0.0037;
% sigmap = 3.7E-5;
% sigmam = -log(0.0037/255);
sigmam = 0.0037;
sigmap = 0.14;
xest = kalmanfilt(Acorr, sigmap, sigmam);

% DETREND
% xest_detrend = hrv_detrend(xest);
lambda = 500;
T = length(xest);
I = speye(T);
D2 = spdiags(ones(T-2,1)*[1 -2 1],[0:2],T-2,T);
xest_detrend = xest*(I-inv(I+lambda^2*(D2')*D2));
thetahat = xest*inv(I'*I+lambda^2*I'*(D2')*D2*I)*I';

disp(' ');

t = (0:numel(Rraw)-1)./fps;
hAraw = figure;
plot(t,Araw), xlabel('t (s)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Araw)+mean(Araw),'r')
end
fprintf(1,'corr(Araw) = %g\n', corr(Araw',yy'));

hAcorr = figure;
plot(t,Acorr), xlabel('t (s)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Acorr)+mean(Acorr),'r')
end
fprintf(1,'corr(Acorr) = %g\n', corr(Acorr',yy'));

hxest = figure;
plot(t,xest), xlabel('t (s)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(xest)+mean(xest),'r')
end
fprintf(1,'corr(xest) = %g\n', corr(xest',yy'));

hxest_detrend = figure;
plot(t,xest_detrend), xlabel('t (s)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(xest_detrend)+mean(xest_detrend),'r')
end
fprintf(1,'corr(xest_detrend) = %g\n', corr(xest_detrend',yy'));

% Correlation plot
w = 100;  % 1s
C = zeros(numel(Acorr)-w, 1);
for i=1:numel(Acorr)-w;
  C(i) = corr(xest_detrend(i:i+w)',yy(i:i+w)');
end
hcorrelation = figure;
plot(linspace(0,tppg(end),numel(C)), C)
xlabel('t_{start} (s)')
ylabel('correlation')
title('Windowed correlation (1s)', 'FontSize', 14)

[tibi, ibi, freqibi, powibi, peak_idx] = resp_rate_from_ppg(t,xest_detrend);
hpeak = figure;
plot(t,xest_detrend)
hold on, plot(t(peak_idx),xest_detrend(peak_idx),'or')
xlabel('t (s)')

hztrend = figure;
plot(t,xest), hold on, plot(t,thetahat,'r','LineWidth',2), xlabel('t (s)')
legend('z','z_{trend}')

hibi = figure;
plot(tibi,ibi), hold on, plot(t(peak_idx(2:end)),ibi,'ro');
xlabel('t (s)'), ylabel('IBI (s)'), title('Interbeat intervals', 'FontSize', 14)
hibi_pow = figure;
plot(freqibi*60,powibi), xlabel('freq (min^-1)'), ylabel('power'), title('Lomb periodogram', 'FontSize', 14)

% Kalman grid
figure;
idx = 1;
z = Acorr;
for sigmap = logspace(-8, -3, 15)
  xest = kalmanfilt(z, sigmap, sigmam);

  subplot(3,5,idx), hold on
  plot(z,'Color',[0.5 0.5 0.5])
  plot(xest,'-r','LineWidth',2)
  xlim([0 numel(z)])
  legend('z','xest')
  title(sprintf('\\sigma_p=%0.2g',sigmap),'fontsize',14)

  idx = idx + 1;
end


pos = get(hAraw, 'position');
pos(4) = PPG_PLOT_HEIGHT;
set(hAraw,'position', pos)
set(hAcorr,'position', pos)
set(hxest,'position', pos)
set(hxest_detrend,'position', pos)
set(hpeak,'position', pos)
set(hztrend,'position', pos)


if strcmpi(input('Save figures? [y/n] ', 's'), 'y')
  prefix = input('Prefix: ', 's');
  base = sprintf('output/%s%%s.png',prefix);
  figure(hraw), export_fig(sprintf(base,'raw'),'-transparent');
  figure(hcorr), export_fig(sprintf(base,'corr'),'-transparent');
  figure(hAraw), export_fig(sprintf(base,'Araw'),'-transparent');
  figure(hAcorr), export_fig(sprintf(base,'Acorr'),'-transparent');
  figure(hxest), export_fig(sprintf(base,'xest'),'-transparent');
  figure(hxest_detrend), export_fig(sprintf(base,'xest_detrend'),'-transparent');
  figure(hpeak), export_fig(sprintf(base,'peak'),'-transparent');
  figure(hibi), export_fig(sprintf(base,'ibi'),'-transparent');
  figure(hibi_pow), export_fig(sprintf(base,'ibi_pow'),'-transparent');
  figure(hcorrelation), export_fig(sprintf(base,'correlation'),'-transparent');
end



% if MOVING_RECT
% %   [R,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect);
%   roi = extract_moving_rect(im_analyse,25,25);
%   R = squeeze(mean2(roi));
%   R = reshape(R,[1, numel(R)]);
% end