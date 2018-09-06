function [hr,rr] = extract_vitals_tk1(frames_head, frames_body, timestamps, fps, block_size)
%#codegen
% assert(isa(frames,'double'))
% assert(isa(block_size,'double'))

% fprintf('%d,%d',size(frames_head, 1), size(frames_head, 2));
% fprintf('%d,%d',size(frames_body, 1), size(frames_body, 2));
% fprintf('%d,%d',size(timestamps, 1), size(timestamps, 2));

if nargin < 2
  block_size = 6;
end

% c_interp = 100*30;
% timestamps = floor(timestamps * 100); % ???
c_interp = 1000;
fps_target = 30;
timestamps = floor(timestamps * c_interp) / c_interp; % ???

Bbody = imresize(frames_body,1/block_size,'box');
Rbody = reshape(permute(Bbody, [3 1 2]),size(Bbody,3),[]);
% Interpolate between uneven timestamped measurements
Rbody_interp = zeros(numel(min(timestamps):1/fps_target:max(timestamps)), size(Rbody,2));
for i  = 1 : size(Rbody,2)
  Rbody_interp(:,i) = interp1(timestamps, Rbody(:,i)', min(timestamps):1/fps_target:max(timestamps));
end
Abody = -log(1+Rbody_interp);

Bhead = imresize(frames_head,1/block_size,'box');
Rhead = reshape(permute(Bhead, [3 1 2]),size(Bhead,3),[]);
% Interpolate between uneven timestamped measurements
Rhead_interp = zeros(numel(min(timestamps):1/fps_target:max(timestamps)), size(Rhead,2));
for i  = 1 : size(Rhead,2)
  Rhead_interp(:,i) = interp1(timestamps, Rhead(:,i)', min(timestamps):1/fps_target:max(timestamps));
end
Ahead = -log(1+Rhead_interp);

[T,~] = size(Ahead);

%%
% breathing = mean(Abody,2);
% heartrate = mean(Ahead,2);

% Get rid of breathing component of heartrate signal.
low_freq = 50/60; % low_freq = 30/60;
high_freq = 80/60; % high_freq = 350/60;
% x_ts = timeseries(Ahead, [0:T-1]./60);
% x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass');
% Ahead_filt = x_filt.Data;

% butter() requires constant valued inputs, so put in if statements.
if abs(fps-30) < 0.1
  nyq = 15;
  [b,a] = butter(3,[low_freq high_freq]./nyq,'bandpass');
elseif abs(fps-60) < 0.1
  nyq = 30;
  [b,a] = butter(3,[low_freq high_freq]./nyq,'bandpass');
else
  error('Unsupported frame rate needed for constant nyq criterion in butter(). Please update if statements in the code.');
end
Ahead_filt = zeros(size(Ahead));
for i = 1:size(Ahead_filt,2)
  Ahead_filt(:,i) = filter(b, a, Ahead(:,i));
end

low_freq = 5/60;
high_freq = 25/60;
if abs(fps-30) < 0.1
  nyq = 15;  % fs/2
  [b,a] = butter(3,[low_freq high_freq]./nyq,'bandpass');
elseif abs(fps-60) < 0.1
  nyq = 30;  % fs/2
  [b,a] = butter(3,[low_freq high_freq]./nyq,'bandpass');
else
  error('Unsupported frame rate needed for constant nyq criterion in butter(). Please update if statements in the code.');
end
% Abody = zeros(size(Abody));
% for i = 1:size(Abody,2)
%   Abody(:,i) = filter(b, a, Abody(:,i));
% end




% Do weighted average of signals based on (inverse) entropy to yield a
% signal (heartrate and breathing).

% power = (a^2+b^2)/N
% Should be same as 1/N*abs(Y)^2
npts = size(Abody,1);
Fheartrate = zeros(floor(npts/2), size(Ahead,2));
Fbreathing = zeros(floor(npts/2), size(Abody,2));

Y = fft(Abody, npts);
Pyy = Y.*conj(Y)/npts;
freq = fps/npts * (0:floor(npts/2)-1);
Fbreathing = Pyy(1:floor(npts/2),:);
Y = fft(Ahead, npts);
Pyy = Y.*conj(Y)/npts;
freq = fps/npts * (0:floor(npts/2)-1);
Fheartrate = Pyy(1:floor(npts/2),:);

% [freq, Fbreathing] = plot_power_spectrum(Abody, 60);
% [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60);
idx1 = find(freq<5/60, 1, 'first');
idx2 = find(freq>25/60, 1, 'first');
Fbreathing([1:idx1, idx2:end],:) = 0;
Fheartrate(1,:) = 0;
% Fbreathing(1,:) = zeros(1, size(Fbreathing,2));
% Fheartrate(1,:) = zeros(1, size(Fheartrate,2));

HRentr = get_spectral_entropy(Fheartrate);
whr = 1-HRentr;
whr = whr./sum(whr);
heartrate = Ahead_filt * whr';

RRentr = get_spectral_entropy(Fbreathing);
wrr = 1-RRentr;
wrr = wrr./sum(wrr);
% Use fft to get rid of phase differences between signals.
F = abs(fft(Abody));
F(1,:) = 0;
breathing = ifft(F * wrr');

sigmam = 1.4e-3;
sigmap = 2.3e-5;
R = exp(-heartrate)-1;
xhat = kalmanfilt(R', sigmap, sigmam);
heartrate = -log(xhat+1);

[~,maxidx] = max(whr);
heartrate_best = Ahead_filt(:,maxidx);
R = exp(-heartrate_best)-1;
xhat = kalmanfilt(R', sigmap, sigmam);
heartrate_best = -log(xhat+1);
%
% figure;
% subplot(1,2,1),
% [H,W,~] = size(Bhead);
% imshow(flipud((reshape(whr,[H W]))),[])
% title('Pulse weights')
% subplot(1,2,2)
% [H,W,~] = size(Bbody);
% imshow(flipud(reshape(wrr,[H W])),[]);
% title('Breathing weights')

%% Find vitals
[freq, Fbreathing] = plot_power_spectrum(breathing, fps);
[freq, Fheartrate] = plot_power_spectrum(heartrate, fps);

idx1 = find(freq<5/60, 1, 'first');
idx2 = find(freq>25/60, 1, 'first');
Fbreathing([1:idx1, idx2:end],:) = 0;
Fheartrate(1,:) = 0;

[~,maxidx] = max(Fheartrate);
hr = 60*freq(maxidx);
[~,maxidx] = max(Fbreathing);
rr = 60*freq(maxidx);

% t = [0:numel(heartrate_best)-1] ./ 60;
% figure;
% subplot(2,2,1), plot(t,heartrate_best), title('Heart rate')
% hold on, plot(t,heartrate,'--k')
% xlabel('time (s)')
% ylabel('blood volume (a.u.)')
% legend('strongest signal','weighted average')
%
% subplot(2,2,2), plot(t,breathing), title('Breathing')
% xlabel('time (s)')
% ylabel('breathing amplitude (a.u.)')
%
% subplot(2,2,3), plot(60*freq,Fheartrate), title(sprintf('HR=%u bpm',round(hr*60)))
% [~,maxidx] = max(Fheartrate);
% hold on, plot(60*freq(maxidx),Fheartrate(maxidx),'or')
% xlim([0 300])
% xlabel('frequency (1/min)')
% ylabel('spectral power (dB)')
%
% subplot(2,2,4), plot(60*freq,Fbreathing), title(sprintf('RR=%u bpm',round(rr*60)))
% [~,maxidx] = max(Fbreathing);
% hold on, plot(60*freq(maxidx),Fbreathing(maxidx),'or')
% xlim([0 300])
% xlabel('frequency (1/min)')
% ylabel('spectral power (dB)')
