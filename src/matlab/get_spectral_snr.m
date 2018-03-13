% snr = get_spectral_snr(sig, sig_ref, Fs)
% snr = get_spectral_snr(sig, sig_ref, Fs, freq_range)
function snr = get_spectral_snr(sig, sig_ref, Fs, freq_range)
  [freq, sig_pow] = plot_power_spectrum(sig, Fs);
  sig_mag = sqrt(sig_pow);
  sig_mag(1,:) = 0;
  [~, sigref_pow] = plot_power_spectrum(sig_ref, Fs);
  sigref_mag = sqrt(sigref_pow);
  sigref_mag(1) = 0;
  
  if nargin < 4
  warning('Changed get_spectral_snr freq_range to be list of nums. Does this look right?')
    freq_range = [1 : numel(sigref_mag)];
  else
    freq_range = [find(freq>freq_range(1),1) : ...
                  find(freq<freq_range(2),1,'last')];
    
  end
  
  sigref_mag = sigref_mag(freq_range);
  sig_mag = sig_mag(freq_range, :);

  sigref_mag = sigref_mag./sum(sigref_mag);
  sig_mag = bsxfun(@rdivide, sig_mag, sum(sig_mag,1));

  S = sigref_mag;
  N = bsxfun(@minus, sig_mag, sigref_mag);

  snr = 10*log10(sum(S.^2,1)./sum(N.^2,1));
%   sqerr = bsxfun(@minus, Acam_mag, ppg_mag).^2;
%   snr = 10*log10(1./mean(sqerr,1));
%   snr = corr(Acam_mag, ppg_mag);
%   snr = corr(Acam_mag, ppg_mag).^2;