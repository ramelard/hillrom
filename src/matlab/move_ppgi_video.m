function move_ppgi_video(src_dir, dest_dir, tstart, tend, fps, rect, query_user)

if nargin < 6
  rect = [];
end
if nargin < 7
  query_user = true;
end

fstart_idx = ceil(tstart*fps);
fstop_idx = floor(tend*fps);
dt = fstop_idx-fstart_idx;
% tppg = zeros(fstop-fstart,1);
% ppg = zeros(fstop-fstart,1);
tppg = [];
ppg = [];

if query_user
  % Choose a time frame that has a clean PPG signal.
  h=figure;
  while true
    [tppg,ppg] = load_ppg(src_dir, fstart_idx, fstop_idx, fps);
    if isempty(ppg)
      fstart_idx = 0;
      fstop_idx = fstart_idx + dt;
      [tppg,ppg] = load_ppg(src_dir, fstart_idx, fstop_idx, fps);
    end
    plot(tppg,ppg)
    if strcmpi(questdlg('Use this timerange?','Time Range','Yes','No','No'), 'yes')
      close(h)
      break;
    end
    fstart_idx = fstart_idx + floor(dt/3);
    fstop_idx = fstop_idx + floor(dt/3);
  end
end

D = dir([src_dir,'/*.avi']);
if isempty(D)
  return;
end
dest_file = [dest_dir, '/', D(1).name];

chunk_size = 200;

v = VideoWriter(dest_file, 'Motion JPEG AVI');
v.FrameRate = fps;
% v.LosslessCompression = true;
v.Quality = 100;
open(v)
for i = fstart_idx:chunk_size:fstop_idx
  frame1 = i;
  frameN = min(frame1+chunk_size-1, fstop_idx);
  [~, im_inorm, ~, ~, ~, ~, rect] = ...
    load_ppgi_images(src_dir, fps, frame1, frameN, Inf, rect);
  
  fprintf(1,'Writing %u frames...', size(im_inorm,3))
  
  writeVideo(v, reshape(im_inorm, [size(im_inorm, 1) size(im_inorm,2) 1 size(im_inorm,3)]));
  fprintf(1,'Done\n')
end
close(v)

[tppg, ppg, ~] = load_ppg(src_dir, fstart_idx, fstop_idx, fps);
tppg = tppg-tppg(1);
save([dest_dir, '/tppg_ppg.mat'], 'tppg', 'ppg')