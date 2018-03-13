% Get normalized entropy from spectral power range.
function entr = compute_spectral_entropy(power_range)

if size(power_range,1) == 1
  power_range = power_range(:);
end

%   A = sqrt(power_range);
% Make spectrum a PDF
power_range = bsxfun(@rdivide, power_range, sum(power_range,1));
% Happens when whole spectrum range has 0 power.
power_range(isnan(power_range) | power_range==0) = 1E-100;
% Use normalized entropy.
entr = -sum(power_range.*log2(power_range)) / log2(size(power_range,1));
if any(isnan(entr))
  warning('Entropy has NaN values');
end
if any(isinf(entr))
  warning('Entropy has Inf values');
end

end