% Shows how the different parameters work in FFT.
function fft_demo

%%
% The frequency bin resolution is 1/T (e.g., T=2s means f=0.5Hz per bin)
% The maximum frequency is fs/2
fs = 100;
t = 0:1/fs:2;
ystr = 'sin(2*pi*t-pi/4)';
y = eval(ystr);

figure;
subplot(411), plot(t,y)
title(ystr,'fontsize',14)

npts = numel(y);
Y = fft(y);
freqs = fs/npts * (0:floor(npts/2)-1);

% "2/N" because the Fourier spectrum is symmetric, so double count when
% just looking at one half of the spectrum (I think).
subplot(412), plot(freqs, 2/npts*abs(Y(1:floor(npts/2))), '-x')
title('Sinusoidal Magnitude','fontsize',14)
subplot(413), plot(freqs, 2/npts*real(Y(1:floor(npts/2))), '-x')
title('cos magnitude (real component)','fontsize',14)
% "-2" because it's cos-jsin
subplot(414), plot(freqs, -2/npts*imag(Y(1:floor(npts/2))), '-x')
title('sin magnitude (imaginary component)','fontsize',14)

%% Constant Q demo
% Note: Constant Q isn't quite what we want, because we want uniform density
% between F1 and F2, not logarithmic density.
minF = 0.5;
maxF = 3.5;
% octave(f) = 2*f, so same number of bins for 1-2Hz, 2-4Hz, 3-6Hz, etc.
bins_per_octave = 5;
Q = constQ(ppg, sparseKernel(minF, maxF, bins_per_octave, numel(tppg)/tppg(end)));
Qfreqs = minF*2.^[[0:numel(Q)-1]/bins_per_octave];
figure, plot(Qfreqs, abs(Q), '-x')