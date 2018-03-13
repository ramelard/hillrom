% CLEAN_IPPG Generate clean IPPG signal.
% 
% Acorr = clean_ippg(R)
% [Acorr,Adenoise] = clean_ippg(R)
%   Generate a clean IPPG signal by (1) denoising, (2) detrending. R must be
%   ambient-corrected.
function [Acorr,Adenoise] = clean_ippg(R)

sigmam = 0.0037;
sigmap = 7.2E-5;
xest = kalmanfilt(R, sigmap, sigmam);
Aest = -log(xest);
Acorr = hrv_detrend(Aest);

if nargout > 1
  Adenoise = xest;
end