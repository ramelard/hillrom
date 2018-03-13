% DENOISE_LED_PPG(R,fs,Wn,order)
%   Apply Butterworth filter on noisy PPG signal from LED.
%   Rhat = denoise_LED_PPG(R,fs)
%   Rhat = denoise_LED_PPG(R,fs,varargin)
%     Flo - high-pass cutoff filter
%     Fhi - low-pass cutoff filter
%     ord - Butterworth filter order
function Rhat = denoise_LED_PPG(R,fs,varargin)

Flo = 0;  % high-pass cutoff frequency (Hz)
Fhi = Inf;  % low-pass cutoff frequency (Hz)
order = 10;  % Butterworth order
for i = 1:2:nargin - 2
  pname = varargin{i};
  if strcmpi(pname,'Flo')
    pval = varargin{i+1};
    Flo = pval;
  elseif strcmpi(pname,'Fhi')
    pval = varargin{i+1};
    Fhi = pval;
  elseif strcmpi(pname,'ord')
    pval = varargin{i+1};
    order = pval;
  else
    warning(sprintf('Could not interpret param %s', pname))
  end
end


[B,A] = butter(order,Fhi/(fs/2),'low');  % have to normalize cutoff freq
Rhat = filtfilt(B,A,R);

% Wn = .5;
% [B,A] = butter(order,Wn,'high');
% Rhat = filter(B,A,Rhat);
% 
% figure;
% t = (0:numel(R)-1)./fs;
% subplot(2,1,1), plot(t,R), xlabel('t')
% subplot(2,1,2), plot(t,Rhat), xlabel('t')

end