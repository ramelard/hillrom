function hr = hr_autocorr(tsig, sig)

Fs = 1/mean(diff(tsig));
Fs_subsample = 200;
minlag = 1/200*60;  % [s] fastest heart rate: 200bpm
maxlag = 1/30*60;  % [s] slowest heart rate: 30bpm

xx = 0:1/Fs_subsample:tsig(end);
yy = spline(tsig,sig,xx);
[c,lags] = xcorr(yy, maxlag*Fs_subsample,'unbiased');
origin = floor(numel(c)/2) + 1;
% [~,maxidx] = max(c(origin+ceil(minlag*Fs_subsample):end));
% valid_range = origin + [ceil(minlag*Fs_subsample) : ceil(maxlag*Fs_subsample)];
% [~,maxidx] = max(diff(c(valid_range)));
[~,peak_loc] = findpeaks(mat2gray((c(origin+ceil(minlag*Fs_subsample):end))), ...
                         'MinPeakDistance',40/120*Fs_subsample);
maxidx = origin+ceil(minlag*Fs_subsample)+peak_loc(1);
figure, plot(c), hold on, plot(maxidx,c(maxidx),'or'); pause(1); close;

%     figure, plot(lags,c)
shift = lags(maxidx);
hr = 1/(shift/Fs_subsample)*Fs;