function [hr,rr] = extract_vitals(frames, face_rect, block_size)
%#codegen
% assert(isa(frames,'double'))
% assert(isa(block_size,'double'))

if nargin < 2
  block_size = 6;
end

% tic
% frames = imrotate(frames,180);
% toc

if isempty(face_rect)
  fprintf(1,'Finding face...')
  % Need to make face pointing up so it works with Viola-Jones.
  frame = flipud(frames(:,:,1));
  % Viola-Jones cascade face detector. Min size chosen empirically based on
  % optical setup.
  faceDetector = vision.CascadeObjectDetector('MinSize',[75 75]);
  % faceDetector = vision.CascadeObjectDetector('ProfileFace','MinSize',[75 75]);
  face_rect = step(faceDetector, frame);
end

if isempty(face_rect)
  warning('No face detected. Using coarse ROIs.')
  body = frames(2:304,2:678,:);
  head = frames(443:851, 110:521, :);
else
  [H,W,T] = size(frames);
  x = face_rect(1);
  y = H-face_rect(2);
  w = face_rect(3);
  h = face_rect(4);
end

  % Draw the returned bounding box around the detected face.
%   videoOut = insertObjectAnnotation(frame,'rectangle',face_rect,'Face');
%   figure, imshow(videoOut), title('Detected face');

  % Expand the head ROI so we get the neck as well.
%   head = frames(y-h*1.5 : y+h*.10, x:x+w, :);
  head = frames(y-h : y, x:x+w, :);
  % The rest is body.
  body = frames(1 : y-h*2, 1 : end, :);
% end
  
Bbody = imresize(body,1/block_size,'box');
Rbody = reshape(permute(Bbody,[3 1 2]),size(Bbody,3),[]);
Abody = -log(1+Rbody);

Bhead = imresize(head,1/block_size,'box');
Rhead = reshape(permute(Bhead,[3 1 2]),size(Bhead,3),[]);
Ahead = -log(1+Rhead);

[T,~] = size(Ahead);

%%
% breathing = mean(Abody,2);
% heartrate = mean(Ahead,2);

% Get rid of breathing component of heartrate signal.
low_freq = 30/60;
high_freq = 350/60;
% x_ts = timeseries(Ahead, [0:T-1]./60);
% x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass');
% Ahead_filt = x_filt.Data;

fs = 60;
nyq = fs/2;
[b,a] = butter(3,[low_freq high_freq]./nyq,'bandpass');
Ahead_filt = zeros(size(Ahead));
for i = 1:size(Ahead_filt,2)
  Ahead_filt(:,i) = filter(b, a, Ahead(:,i));
end




% Do weighted average of signals based on (inverse) entropy to yield a
% signal (heartrate and breathing).

% power = (a^2+b^2)/N
% Should be same as 1/N*abs(Y)^2
npts = size(Abody,1);
Fheartrate = zeros(floor(npts/2), size(Ahead,2));
Fbreathing = zeros(floor(npts/2), size(Abody,2));

Y = fft(Abody, npts);
Pyy = Y.*conj(Y)/npts;
freq = 60/npts * (0:floor(npts/2)-1);
Fbreathing = Pyy(1:floor(npts/2),:);
Y = fft(Ahead, npts);
Pyy = Y.*conj(Y)/npts;
freq = 60/npts * (0:floor(npts/2)-1);
Fheartrate = Pyy(1:floor(npts/2),:);

% [freq, Fbreathing] = plot_power_spectrum(Abody, 60);
% [freq, Fheartrate] = plot_power_spectrum(Ahead_filt, 60);
Fbreathing(1,:) = 0;
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
[freq, Fbreathing] = plot_power_spectrum(breathing, 60);
[freq, Fheartrate] = plot_power_spectrum(heartrate, 60);

Fbreathing(1,:) = 0;
Fheartrate(1,:) = 0;

[~,maxidx] = max(Fheartrate);
hr = freq(maxidx);
[~,maxidx] = max(Fbreathing);
rr = freq(maxidx);

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