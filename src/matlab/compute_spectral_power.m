%PLOT_POWER_SPECTRUM Plot frequency power spectrum
%
% [freq,fpower] = plot_power_spectrum(y, sampling_rate)
%
% From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html
% y - signal (columns are observations)
function [freq,power] = compute_spectral_power(y, sampling_rate)

% if nargin < 4
%   linecolor = 'b';
% end

if size(y,1) == 1
  y = y(:);
end

% if size(y,2) > size(y,1)
%   warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2)))
% end

% npts = numel(y);
npts = size(y,1);

Y = fft(y, npts);
% power = (a^2+b^2)/N
% Should be same as 1/N*abs(Y)^2
Pyy = Y.*conj(Y)/npts;

% Low frequencies may be other signals (e.g. respirator arrhythmia, Mayer waves)
f30bpm = floor(30/60*npts/sampling_rate) + 1;
power = Pyy(1:floor(npts/2),:);
power(1:f30bpm,:) = 0;
% power = Pyy(f30bpm:floor(npts/2),:);

freq = sampling_rate/npts * (0:floor(npts/2)-1);
% % nth bin is n*Fs/N, +1 for index offset
% f30bpm = floor(30/60*npts/sampling_rate) + 1;
% f200bpm = floor(200/60*npts/sampling_rate) + 1;
