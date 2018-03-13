FRAME_RATES = [50];
SWITCH_RATES = [3 1];

% Parameteric correlation and error values (mean, std)
C = zeros(numel(FRAME_RATES), numel(SWITCH_RATES), 2);
E = zeros(numel(FRAME_RATES), numel(SWITCH_RATES), 2);

for F = 1:numel(FRAME_RATES)
for S = 1:numel(SWITCH_RATES)

disp(' ')

% Update these each time.
fps = FRAME_RATES(F);
switch_rate = SWITCH_RATES(S);  % Inf if no switching
disp(sprintf('FRAME_RATE = %u, SWITCH_RATE = %u', fps, switch_rate))
fstart = 0;
fstop = 10*fps;  % last file to process
ch = 1;  % 1-R, 2-G, 3-B

data_dir = sprintf('C:/Users/ramelard/Desktop/data/SPIE/RA/test-f%u-s%u/', fps, switch_rate);

D = dir([data_dir, '/*.pgm']);
last_file = D(end).name;
dash_idx = find(D(end).name == '-');

fbase = [data_dir, D(1).name(1:dash_idx(4)), '%04u.pgm'];
fend = str2double(last_file(dash_idx(end)+1:end-4));  % last file in full capture

iminfo = imfinfo(sprintf(fbase,fstart));

% Find first ambient frame
first_frames = zeros(iminfo.Height,iminfo.Width,4);
for i = 0:3
  im = imread(sprintf(fbase,fstart+i));
  first_frames(:,:,i+1) = im2double(im(:,:,ch));
end
first_frames = reshape(first_frames, [numel(first_frames(:,:,1)), size(first_frames,3)]);
[~,ambient_idx] = min(mean(first_frames));
fstart = fstart + (ambient_idx - 1);

% Try to load ppg data from saved file.
ppg_data_file = sprintf('%stppg_ppg.mat', data_dir);
if exist(ppg_data_file, 'file') == 2
  fprintf(1,'Loading data file %s\n', ppg_data_file)
  load(ppg_data_file)
  
  [~,sidx] = min(abs(tppg-fstart/fps));
  [~,eidx] = min(abs(tppg-fstop/fps));
  tppg = tppg(sidx:eidx);
  ppg = ppg(sidx:eidx);
end

% Ready for processing!
clear im_inorm
clear im_i
clear im_ambient
im_inorm = zeros(iminfo.Height,iminfo.Width,fstop-fstart);
im_i = zeros(iminfo.Height,iminfo.Width,fstop-fstart);
im_ambient = zeros(iminfo.Height,iminfo.Width,floor((fstop-fstart)/switch_rate));
idx = 1;
ambient_idx = 1;
% im_ambient0 = zeros(iminfo.Height,iminfo.Width,3);
h = waitbar(0,'Reading images');
drawnow
for i = fstart:fstop
  waitbar((i-fstart)/(fstop-fstart),h)
  t = i-fstart+1;
  if mod(t-1, switch_rate+1) < 1
    im = im2double(imread(sprintf(fbase,i)));
    im_ambient(:,:,ambient_idx) = im(:,:,ch);
    ambient_idx = ambient_idx + 1;
    if idx > 1
      % Reuse last frame to ensure constant frame rate.
      im_i(:,:,idx) = im_i(:,:,idx-1);
      im_inorm(:,:,idx) = im_i(:,:,idx-1) - im_ambient(:,:,ambient_idx-1);
      idx = idx + 1;
    end
  else
    im_illum = im2double(imread(sprintf(fbase,i)));
%     im_inorm(:,:,idx) = im_illum(:,:,ch) - im_ambient(:,:,ch);
    im_inorm(:,:,idx) = im_illum(:,:,ch) - im_ambient(:,:,ambient_idx-1);
    im_i(:,:,idx) = im_illum(:,:,ch);
    idx = idx + 1;
  end
end
waitbar(1,h,'Closing')
close(h)
if (idx-1) ~= size(im_inorm,3)
  warning(sprintf('idx-1 (%u) should be size(im_inorm,3) (%u)',idx-1,size(im_inorm,3)))
  im_inorm = im_inorm(:,:,1:idx-1);
  im_i = im_i(:,:,1:idx-1);
end

rect = [];

%% Extract and analyse squared error over time window
twin = 5*fps;

if ~exist('rect','var')
  rect = [];
end

APPLY_AMBIENT_SUBTRACTION = true;
MOVING_RECT = false;
ZOOM = false;  % zoom before choosing rect

if APPLY_AMBIENT_SUBTRACTION
  im_analyse = im_inorm;
  disp('Analysing ambient subtracted images.')
else
  im_analyse = im_i;
  disp('Analysing images without ambient subtraction.')
end

if MOVING_RECT
%   [R,rect] = plot_pixel_vals(im_inorm,'fps',fps,'roi','rect',rect);
  roi = extract_moving_rect(im_analyse,25,25);
  R = squeeze(mean2(roi));
  R = reshape(R,[1, numel(R)]);
else
  [R,rect] = plot_pixel_vals(im_analyse,'fps',fps,'roi','rect',rect,'zoom',ZOOM);
end

tppg = tppg-tppg(1);
ppg_sample_rate = numel(tppg)/tppg(end);
% Sample PPG so its sample rate is the same as the camera frame rate.
% (Makes it easy for comparing).
tppg_unif = linspace(0,tppg(end),numel(R));
ppg_unif = spline(tppg,ppg,tppg_unif);
A = -log(R);
A(A==Inf) = 0;

figure;
plot(tppg_unif,A)
hold on
plot(tppg_unif,(ppg_unif-mean(ppg_unif))./std(ppg_unif).*std(A)+mean(A),'r')
ylabel('absorbance (a.u.')
legend('PPGI','PPG')

% Analyse squared error over sliding time window.
Flo = 50/60;
Fhi = 150/60;
err = [];
cor = [];
for i = 1:numel(A)-twin-1
  ppgwin = ppg_unif(i:i+twin);
  Awin = A(i:i+twin);
  
  [ppg_freq, ppg_pow] = plot_power_spectrum(ppgwin, fps);
  [ppgi_freq, ppgi_pow] = plot_power_spectrum(Awin, fps);

  if any(ppgi_freq-ppg_freq)
    warning('PPGI and PPG frequencies are not the same!')
  end
  
  Flo_idx = find(ppgi_freq > Flo, 1);
  Fhi_idx = find(ppgi_freq > Fhi, 1);

  % Find HR
  [~, fmax_idx] = max(ppg_pow(Flo_idx:Fhi_idx));
  hr = ppg_freq(fmax_idx + Flo_idx - 1);

  % Find estimated HR
  [~, fmax_idx] = max(ppgi_pow(Flo_idx:Fhi_idx));
  fmax = ppgi_freq(fmax_idx + Flo_idx - 1);

  if hr ~= fmax
%     disp(sprintf('%g vs %g', hr, fmax));
  end
  
  err(i) = (hr-fmax)^2;
  cor(i) = corr(Awin',ppgwin');
end

E(F,S,1) = mean(err);
E(F,S,2) = std(err);
C(F,S,1) = mean(cor);
C(F,S,2) = std(cor);
figure, plot(err), title('Squared Error')
figure, plot(cor), title('Correlation')
disp(sprintf('Mean squared error: %g', mean(err)))
disp(sprintf('Std squared error: %g', std(err)))
disp(sprintf('Mean correlation: %g', mean(cor)))
disp(sprintf('Std correlation: %g', std(cor)))

end
end
%%
fprintf(1,'%% Mean, Std of squared error and correlation (%us sliding window)\n', twin/fps);
disp('E = [')
for i = 1:numel(FRAME_RATES)
  for j = 1:numel(SWITCH_RATES)
    fprintf(1,'%g %g  %% f%u-s%u, 0-10s\n', E(i,j,1), E(i,j,2), FRAME_RATES(i), SWITCH_RATES(j));
  end
end
disp(']')
disp('C = [')
for i = 1:numel(FRAME_RATES)
  for j = 1:numel(SWITCH_RATES)
    fprintf(1,'%g %g  %% f%u-s%u, 0-10s\n', C(i,j,1), C(i,j,2), FRAME_RATES(i), SWITCH_RATES(j));
  end
end
disp(']')

figure;
X = [100 50 25 20 15];
subplot(2,2,1),plot(X,E(2:2:end,1),'-x'), title('Error, s=1'), xlabel('frame rate (Hz)'), ylabel('Squared error')
subplot(2,2,2),plot(X,C(2:2:end,1),'-x'), title('Correlation, s=1'), xlabel('frame rate (Hz)'), ylabel('Correlation')
subplot(2,2,3),plot(X,E(1:2:end,1),'-x'), title('Error, s=3'), xlabel('frame rate (Hz)'), ylabel('Squared error')
subplot(2,2,4),plot(X,C(1:2:end,1),'-x'), title('Correlation, s=3'), xlabel('frame rate (Hz)'), ylabel('Correlation')
