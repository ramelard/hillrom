function journal_plots(im_i, im_inorm, fps, ppg, tppg, ppg_tidx, varargin)

PPG_PLOT_HEIGHT = 200;


zoom_first = false;
plot_ppg = false;
rect = [];
prefix = [];
i = 1;
while i+6 <= nargin
  pname = varargin{i};
  if strcmpi(pname,'zoom')
    zoom_first = true;
  elseif strcmpi(pname,'plot_ppg')
    plot_ppg = true;
  elseif strcmpi(pname,'rect')
    rect = varargin{i+1};
    i = i + 1;
  elseif strcmpi(pname,'save')
    prefix = varargin{i+1};
    i = i + 1;
  end
  i = i + 1;
end

[Rcorr,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect,'zoom',zoom_first);
[Rraw,rect] = plot_pixel_vals(im_i,'fps',fps,'roi','rect',rect,'zoom',zoom_first);

fprintf(1,'\nrect=');
disp(sprintf('[%u %u %u %u]',rect(1),rect(2),rect(3),rect(4)))

close all

% RAW IMAGE
hraw = figure;
imshow(im_i(:,:,1));
r = rectangle('position', rect);
set(r, 'edgecolor', 'r');

% AMBIENT-CORRECTED IMAGE
hcorr = figure;
imshow(im_inorm(:,:,1))
r = rectangle('position', rect);
set(r, 'edgecolor', 'r');

Araw = -log(Rraw);
Araw(Araw==Inf) = 0;
sigmam = 0.0037;
sigmap = 7.2E-5;
xest = kalmanfilt(Rcorr, sigmap, sigmam);
Aest = -log(xest);
Aest_detrend = hrv_detrend(Aest);

% tppg = tppg-tppg(1);

% Sample PPG so its sample rate is the same as the camera frame rate.
% (Makes it easy for comparing).
xx = linspace(tppg(1),tppg(end),numel(Rraw));
yy = spline(tppg,ppg,xx);

t = (0:numel(Rraw)-1)./fps;
hAraw = figure;
plot(t,Araw), xlabel('t (s)','fontsize',12), ylabel('absorbance (a.u.)','fontsize',12)
if plot_ppg
  hold on, plot(tppg-tppg(1),(ppg-mean(ppg))./std(ppg).*std(Araw)+mean(Araw),'r')
end
set(gca,'ytick',[])
fprintf(1,'corr(Araw) = %g\n', corr(Araw',yy'));
title('Unprocessed PPGI','FontSize',14)

hAcorr = figure;
plot(t,Aest_detrend), xlabel('t (s)','fontsize',12), ylabel('absorbance (a.u.)','fontsize',12)
if plot_ppg
  hold on, plot(tppg-tppg(1),(ppg-mean(ppg))./std(ppg).*std(Aest_detrend)+mean(Aest_detrend),'r')
end
set(gca,'ytick',[])
fprintf(1,'corr(Aest_detrend) = %g\n', corr(Aest_detrend',yy'));
title('Processed PPGI','FontSize',14)

hppg = figure;
plot(xx-xx(1),yy), xlabel('t (s)','fontsize',12), ylabel('absorbance (a.u.)','fontsize',12)
xlim([0 xx(end)-xx(1)])
set(gca,'ytick',[])
title('Ground-Truth PPG','FontSize',14)

% DETREND EXAMPLE
hztrend = figure;
lambda = 500;
T = length(xest);
I = speye(T);
D2 = spdiags(ones(T-2,1)*[1 -2 1],[0:2],T-2,T);
xest_detrend = xest*(I-inv(I+lambda^2*(D2')*D2));
thetahat = xest*inv(I'*I+lambda^2*I'*(D2')*D2*I)*I';
plot(t,xest), hold on, plot(t,thetahat,'r','LineWidth',2)
xlabel('t (s)','fontsize',12)
ylabel('reflectance (a.u.)','fontsize',12)
legend('z','z_{trend}')
hzdetrend = figure;
plot(t,xest_detrend)
xlabel('t (s)','fontsize',12)
ylabel('reflectance (a.u.)','fontsize',12)

% POWER PLOT
hpow = figure;
hold on;
f30bpm = round(30/60*numel(Araw)/fps) + 1;
f200bpm = round(200/60*numel(Araw)/fps) + 1;

[freq,power] = plot_power_spectrum(yy,fps);
plot(freq, power./max(power(f30bpm:f200bpm)), '--s','Color',[0.5 0.5 0.5])
[freq,power] = plot_power_spectrum(Araw,fps);
plot(freq, power./max(power(f30bpm:f200bpm)), '-ko')
[freq,power] = plot_power_spectrum(Aest_detrend,fps);
plot(freq, power./max(power(f30bpm:f200bpm)), '-k*')

xlim([30/60 200/60])
ylim([0 1])
title('Power spectral density', 'fontsize',14)
xlabel('frequency (Hz)','FontSize',12)
ylabel('power (a.u.)','FontSize',12)
legend('PPG','Unprocessed','Processed','Location','northeast')

% CORRELATION PLOT
w = 100;  % 1s
Ccorr = zeros(numel(Aest_detrend)-w, 1);
Craw = zeros(numel(Araw)-w, 1);
for i=1:numel(Aest_detrend)-w;
  Craw(i) = corr(Araw(i:i+w)',yy(i:i+w)');
  Ccorr(i) = corr(Aest_detrend(i:i+w)',yy(i:i+w)');
end
hcorrelation = figure;
hold on;
plot(linspace(0,tppg(end)-tppg(1),numel(Craw)), Craw, 'b')
plot(linspace(0,tppg(end)-tppg(1),numel(Ccorr)), Ccorr, 'r')
xlim([0 tppg(end)-tppg(1)])
xlabel('t_{start} (s)','FontSize',12)
ylabel('correlation','FontSize',12)
title('Windowed correlation (1s)', 'FontSize', 14)
legend('Unprocessed','Processed','Location','southeast')
ylim([-1 1])
disp(sprintf('Raw (mean,std) windowed correlation: (%g,%g)',mean(Craw),std(Craw)))
disp(sprintf('Corr (mean,std) windowed correlation: (%g,%g)',mean(Ccorr),std(Ccorr)))

bland_altman(Ccorr);


pos = get(hAraw, 'position');
pos(4) = PPG_PLOT_HEIGHT;
handles = {hAraw, hAcorr, hppg, hpow, hcorrelation, hztrend, hzdetrend};
for i = 1:numel(handles)
  h = handles{i};
  figure(h);
  prev_ylim = ylim;
  set(h,'position', pos);
  ylim(prev_ylim);
end
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);
% figure(hAraw); prev_ylim = ylim; set(hAraw,'position', pos); ylim(prev_ylim);


% set(hAcorr,'position', pos)
% set(hppg,'position', pos)
% set(hpow,'position', pos)
% set(hcorrelation,'position', pos)
% set(hztrend,'position',pos)
% set(hzdetrend,'position',pos)

ppg_stats_test(Araw,Aest_detrend,tppg,ppg,ppg_tidx);


if ~isempty(prefix) || strcmpi(input('Save figures? [y/n] ', 's'), 'y')
  if isempty(prefix)
    prefix = input('Prefix: ', 's');
  end
  base = sprintf('output/%s%%s.png',prefix);
  figure(hraw), export_fig(sprintf(base,'img_Iraw'),'-transparent');
  figure(hcorr), export_fig(sprintf(base,'img_Icorr'),'-transparent');
  figure(hAraw), export_fig(sprintf(base,'sig_Araw'),'-transparent');
  figure(hztrend), export_fig(sprintf(base,'sig_trend'),'-transparent');
  figure(hzdetrend), export_fig(sprintf(base,'sig_detrend'),'-transparent');
  figure(hAcorr), export_fig(sprintf(base,'sig_Acorr'),'-transparent');
  figure(hppg), export_fig(sprintf(base,'sig_ppg'),'-transparent');
  figure(hpow), export_fig(sprintf(base,'pow'),'-transparent');
  figure(hcorrelation), export_fig(sprintf(base,'correlation'),'-transparent');
end



% if MOVING_RECT
% %   [R,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect);
%   roi = extract_moving_rect(im_analyse,25,25);
%   R = squeeze(mean2(roi));
%   R = reshape(R,[1, numel(R)]);
% end