% REST_RATE_FROM_PPG   Estimate respiratory rate from PPG signal.
%   resp_rate_from_ppg(tppg,ppg) plots the power spectral density of the
%   interbeat intervals using the Lomb periodogram, similar to Poh 2011.
%   The peak should correspond to respiratory rate.
function [t_out,ibi_out,freq_out,pow_out,peak_idx] = resp_rate_from_ppg(tppg,ppg,auto_peak_detect)

if size(tppg,2) == 1
  tppg = tppg';
  ppg = ppg';
end


if nargin < 3
  auto_peak_detect = true;
end

if auto_peak_detect
%   [pks,locs] = findpeaks(ppg);
  fs = 1/mean(diff(tppg));
  [pks,locs] = findpeaks(ppg,'minpeakdistance',floor(0.5*fs));  % dt=0.5s*100Hz
%   [pks,locs] = findpeaks(ppg,'minpeakheight',500);
else
  [x,~] = getpts;
  locs = zeros(size(x));
  for i = 1:numel(x)
    [~,locs(i)] = min(abs(tppg-x(i)));
  end
end

x = tppg(locs)';

t = x(2:end);
ibi = diff(x);
[freq,pow,prob] = lomb(t,ibi,4,1);

if nargout == 0
  figure;
  plot(tppg,ppg)
  hold on, plot(tppg(locs),ppg(locs),'or')

  figure;
  subplot(121),plot(t,ibi,'-o'), xlabel('t (s)'), ylabel('IBI (s)'), title('Interbeat intervals')
  subplot(122),plot(freq*60,pow), xlabel('freq (min^-1)'), ylabel('power'), title('Lomb periodogram')
else
  t_out = t;
  ibi_out = ibi;
  freq_out = freq;
  pow_out = pow;
  
  peak_idx = locs;
end