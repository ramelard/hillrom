% Get normalized entropy from spectral power range.
function entr = get_spectral_entropy(power_range)
  coder.extrinsic('warning')

  if size(power_range,1) == 1
    power_range = power_range(:);
  end

%   A = sqrt(power_range);
  % Make spectrum a PDF
  power_range = bsxfun(@rdivide, power_range, sum(power_range,1));
  % Happens when whole spectrum range has 0 power.
  power_range(isnan(power_range) | power_range==0) = 1E-100;
  % Use normalized entropy.
  entr = -sum(power_range.*log2(power_range)) ./ log2(size(power_range,1));
  if any(isnan(entr))
    warning('Entropy has NaN values');
  end
  if any(isinf(entr))
    warning('Entropy has Inf values');
  end
  
%       f30bpm = 5;
%       f200bpm = 21;
%       
%       Acam_pow_norm = bsxfun(@rdivide, power_range, sum(power_range,1));
%       pct_in_hr = sum(Acam_pow_norm(f30bpm:f200bpm,:),1);
%       max_over_mean = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./mean(Acam_pow_norm(f30bpm:f200bpm,:),1);

%       entr = entr./max(Acam_pow_norm(f30bpm:f200bpm,:),[],1);
%       entr = 1./entr;
%       entr = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./entr;
%       entr = entr.*(1-pct_in_hr);
%       entr = entr.*(pct_in_hr<.2);
%       entr = pct_in_hr>.5;
%       entr = pct_in_hr;
%       a = .7165; b = 14.9;
%       x = 1-pct_in_hr;
%       entr = entr.*(a*log(b*x./(1-x)));
%       entr = pct_in_hr;