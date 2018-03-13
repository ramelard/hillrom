%PLOT_POWER_SPECTRUM Plot frequency power spectrum
%
% [freq,fpower] = plot_power_spectrum(y, sampling_rate)
%
% From http://www.mathworks.com/products/matlab/examples.html?file=/products/demos/shipping/matlab/fftdemo.html
% y - signal (columns are observations)
function [freq,power] = plot_power_spectrum(y, sampling_rate, ax, linecolor)

if nargin < 4
  linecolor = 'b';
end

if size(y,1) == 1
  sigs = y(:);
else
  sigs = y;
end

% if size(y,2) > size(y,1)
%   warning(sprintf('Are you sure columns are observations? (r%u,c%u)', size(y,1), size(y,2)))
% end

% npts = numel(y);
npts = size(sigs,1);

Y = fft(sigs, npts);
% power = (a^2+b^2)/N
% Should be same as 1/N*abs(Y)^2
Pyy = Y.*conj(Y)/npts;

f = sampling_rate/npts * (0:floor(npts/2)-1);
% nth bin is n*Fs/N, +1 for index offset
f30bpm = floor(30/60*npts/sampling_rate) + 1;
f200bpm = floor(200/60*npts/sampling_rate) + 1;

if nargout == 0 || nargin == 3
  if nargin < 3
    figure;
    ax = gca;
  end
  plot(ax, f, Pyy(1:floor(npts/2)), 'Color', linecolor)
  % xlim([f(f30bpm) f(f200bpm)])
  xlim(ax, [30/60 200/60])
  ylim(ax, [0 max(Pyy(f30bpm : f200bpm))])
  title('Power spectral density')
  xlabel('Frequency (Hz)')
end

if nargout > 0
  freq = f;
  power = Pyy(1:floor(npts/2),:);
end