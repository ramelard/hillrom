% Plots: Raw, Ambient-corrected, Denoised, Detrended
% Figures: Raw, Ambient-corrected

function spie_plots(im_i, im_inorm, fps, ppg, tppg, varargin)

PPG_PLOT_HEIGHT = 200;
% Mean, Std of squared error and correlation (5s sliding window)
E = [
0.0613975 0.121399  % f100-s3, 0-10s
0.0273055 0.0731667  % f100-s1, 0-10s
0.104965 0.177764  % f50-s3, 0-10s
0.214665 0.15785  % f50-s1, 0-10s
0.510488 0.344091  % f25-s3, 0-10s
0.0542111 0.123803  % f25-s1, 0-10s
0.311714 0.325061  % f20-s3, 0-10s
0.544923 0.228746  % f20-s1, 0-10s
0.323156 0.316013  % f15-s3, 0-10s
0.671962 0.379694  % f15-s1, 0-10s
];
C = [
0.143289 0.042714  % f100-s3, 0-10s
0.24666 0.0660451  % f100-s1, 0-10s
0.0307909 0.0466634  % f50-s3, 0-10s
0.265159 0.038726  % f50-s1, 0-10s
0.00271674 0.06716  % f25-s3, 0-10s
0.166983 0.0977997  % f25-s1, 0-10s
0.00247642 0.0572499  % f20-s3, 0-10s
-0.0252143 0.0490324  % f20-s1, 0-10s
-0.077443 0.0715915  % f15-s3, 0-10s
0.00402831 0.0678672  % f15-s1, 0-10s
];
hCorrS1 = figure;
errorbar([100 50 25 20 15],C(2:2:end,1), C(2:2:end,2), '-kx')
xlabel('frame rate (Hz)'), ylabel('correlation')
title('Windowed Correlation (5s), TCI 1:1','FontSize',14)
ylim([-.15 .35])
hCorrS3 = figure;
errorbar([100 50 25 20 15],C(1:2:end,1), C(1:2:end,2), '-kx')
xlabel('frame rate (Hz)'), ylabel('correlation')
title('Windowed Correlation (5s), TCI 3:1','FontSize',14)
ylim([-.15 .35])


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

disp(' ');

t = (0:numel(Rraw)-1)./fps;
hAraw = figure;
plot(t,Araw), xlabel('t (s)'), ylabel('absorbance (a.u.)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Araw)+mean(Araw),'r')
end
set(gca,'ytick',[])
fprintf(1,'corr(Araw) = %g\n', corr(Araw',yy'));

hAcorr = figure;
plot(t,Acorr), xlabel('t (s)'), ylabel('absorbance (a.u.)')
if plot_ppg
  hold on, plot(tppg,(ppg-mean(ppg))./std(ppg).*std(Acorr)+mean(Acorr),'r')
end
set(gca,'ytick',[])
fprintf(1,'corr(Acorr) = %g\n', corr(Acorr',yy'));

hppg = figure;
plot(xx,yy), xlabel('t (s)'), ylabel('absorbance (a.u.)')
set(gca,'ytick',[])

% Power spectra
hAraw_pow = figure;
[freq,power] = plot_power_spectrum(Araw,fps,gca);
title('Power spectral density', 'fontsize',14)
ylabel('power (dB)')
hAcorr_pow = figure;
[freq,power] = plot_power_spectrum(Acorr,fps,gca);
title('Power spectral density', 'fontsize',14)
ylabel('power (dB)')
hppg_pow = figure;
[freq,power] = plot_power_spectrum(yy,fps,gca);
title('Power spectral density', 'fontsize',14)
ylabel('power (dB)')


% w = 500;  % w/fps [s]
% P = zeros(numel(Araw)-w, 1);
% % nth bin is n*Fs/N, +1 for index offset
% npts = numel(Araw);
% f50bpm = floor(50/60*npts/fps) + 1;
% f200bpm = floor(200/60*npts/fps) + 1;
% power = power(f50bpm:f200bpm);
% freq = freq(f50bpm:f200bpm);
% for i=1:numel(Araw)-w;
%   [~, idx] = max(power);
%   P(i) = freq(idx);
% end
% figure;
% plot(linspace(0,tppg(end),numel(P)), P)

% Correlation plot
w = 100;  % 1s
C = zeros(numel(Acorr)-w, 1);
for i=1:numel(Acorr)-w;
  C(i) = corr(Acorr(i:i+w)',yy(i:i+w)');
end
hcorrelation = figure;
plot(linspace(0,tppg(end),numel(C)), C)
xlabel('t_{start} (s)')
ylabel('correlation')
title('Windowed correlation (1s)', 'FontSize', 14)

pos = get(hAraw, 'position');
pos(4) = PPG_PLOT_HEIGHT;
set(hAraw,'position', pos)
set(hAcorr,'position', pos)
set(hppg,'position', pos)


if strcmpi(input('Save figures? [y/n] ', 's'), 'y')
  prefix = input('Prefix: ', 's');
  base = sprintf('output/%s%%s.png',prefix);
  figure(hraw), export_fig(sprintf(base,'Iraw'),'-transparent');
  figure(hcorr), export_fig(sprintf(base,'Icorr'),'-transparent');
  figure(hAraw), export_fig(sprintf(base,'Araw'),'-transparent');
  figure(hAcorr), export_fig(sprintf(base,'Acorr'),'-transparent');
  figure(hppg), export_fig(sprintf(base,'ppg'),'-transparent');
  figure(hAraw_pow), export_fig(sprintf(base,'Araw_pow'),'-transparent');
  figure(hAcorr_pow), export_fig(sprintf(base,'Acorr_pow'),'-transparent');
  figure(hppg_pow), export_fig(sprintf(base,'ppg_pow'),'-transparent');
  figure(hcorrelation), export_fig(sprintf(base,'correlation'),'-transparent');
end



% if MOVING_RECT
% %   [R,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect);
%   roi = extract_moving_rect(im_analyse,25,25);
%   R = squeeze(mean2(roi));
%   R = reshape(R,[1, numel(R)]);
% end