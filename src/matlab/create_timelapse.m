% CREATE_TIMELAPSE Create timelapse video of CHI frames.
%   [timelapse] = create_timelapse(data_dir, fps, fps_ratio, tstart, tstop)
%   [timelapse] = create_timelapse(data_dir, fps, fps_ratio, tstart, tstop, rect_crop)
function [timelapse] = create_timelapse(data_dir, fps, fps_ratio, tstart, tstop, rect_crop)

if nargin < 4 || isempty(tstart)
  tstart = 0;
end
if nargin < 5 || isempty(tstop)
  M = csvread([data_dir, '/framecounts']);
  tstop = sum(M(:,2))/fps;
end
if nargin < 6
  rect_crop = false;
end

fstart = fps*tstart;
fstop = fstart + round(fps*10);  % last file to process
switch_rate = Inf;

timelapse = [];
frames_idx = 1;
while fstop/fps <= tstop
  disp(sprintf('%.1f-%.1fs',fstart/fps,fstop/fps))
  try
  [~, im_inorm, ~, ~, ~, ~] = ...
    load_ppgi_images(data_dir, fps, fstart, fstop, switch_rate, rect_crop);
  catch err
    disp('Reached the end')
    break;
  end
  
  if isempty(timelapse)
    [H,W,~] = size(im_inorm);
    timelapse = zeros(H,W,(tstop-tstart)*fps/fps_ratio);
  end
  
  subframes = im_inorm(:,:,1:fps_ratio:end);
  nframes = size(subframes,3);
  timelapse(:,:,frames_idx:frames_idx + nframes - 1) = subframes;
  frames_idx = frames_idx + nframes;
  
  fstart = fstop + 1;
  fstop = fstop + round(fps*10);
end

timelapse(:,:,frames_idx:end) = [];
save_path = sprintf('%s/timelapse_fratio-%u.mat',data_dir,fps_ratio);
disp(sprintf('Saving %s', save_path))
save(save_path,'timelapse')