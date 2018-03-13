function [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient, crop_rect] = ...
  load_ppgi_images(data_dir, fps, fstart_idx, fstop_idx, switch_rate, rect)

if nargin < 6
  rect = [];
end

if ~isempty(dir([data_dir, '/*.pgm']))
  [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient] = ...
    load_from_pgm(data_dir, fps, fstart_idx, fstop_idx, switch_rate);
elseif ~isempty(dir([data_dir, '/*.avi']))
  [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient, rect] = ...
    load_from_video(data_dir, fps, fstart_idx, fstop_idx, switch_rate, rect, 'avi');
elseif ~isempty(dir([data_dir, '/*.mov']))
  [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient, rect] = ...
    load_from_video(data_dir, fps, fstart_idx, fstop_idx, switch_rate, rect, 'mov');
else
  error(['No AVI video or PGM image files found for ', data_dir])
end

if nargout > 6
  crop_rect = rect;
end

end


function [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient, crop_rect] = ...
  load_from_video(data_dir, fps, fstart_idx, fstop_idx, switch_rate, rect, filetype)

if nargin < 6
  rect = [];
end
if nargin < 7
  filetype = 'avi';
end

fstart_time = fstart_idx/fps;

im_i = [];
im_inorm = [];
tppg = [];
ppg = [];
ppg_tidx = [];
im_ambient = [];

D = dir([data_dir, '/*.', filetype]);
last_file = D(end).name;
dash_idx = find(D(end).name == '-');

ch = 1;  % 1-R, 2-G, 3-B

% Find the video file contains the first target frame.
warning('clean make_framecounts up')
fbase = [data_dir, '/', D(1).name(1:dash_idx(end)), '%04u.', filetype];
if ~exist([data_dir, '/framecounts'])
  disp('Creating framecounts file...')
  make_framecounts(data_dir, fps, filetype);
  disp('Done.')
end

framecounts = csvread([data_dir, '/framecounts']);
cumulative_frames = 0;
for i = 1:size(framecounts,1)
  vid_idx = framecounts(i,1);
  vid_nframes = framecounts(i,2);
  if vid_nframes + cumulative_frames > fstart_idx
    Didx = framecounts(i,1);
    break;
  end
  cumulative_frames = cumulative_frames + vid_nframes;
end
vid = VideoReader(sprintf(fbase,vid_idx));



% % Find the video file contains the first target frame.
% cumulative_frames = 0;
% first_video = str2double(D(1).name(dash_idx(end)+1:dash_idx(end)+4));
% last_video = str2double(D(end).name(dash_idx(end)+1:dash_idx(end)+4));
% % for Didx = 0:numel(D)-1
% for Didx = first_video : last_video
%   vid = VideoReader(sprintf(fbase,Didx));
%   vid_nframes = ceil(vid.Duration*fps);
%   if vid_nframes + cumulative_frames > fstart_idx
%     break;
%   end
%   cumulative_frames = cumulative_frames + vid_nframes;
% end

vid.CurrentTime = (fstart_idx-cumulative_frames)/fps;
frame = readFrame(vid);
if islogical(rect) && rect == false
  rect = [];
elseif isempty(rect) || (islogical(rect) && rect == true)
  hr=figure; imshow(frame)
  rect = getrect;
  disp(floor(rect))
  close(hr)
end

vid.CurrentTime = 0;
frame = readFrame(vid);
frame = frame(:,:,1);
if ~isempty(rect)
  frame = imcrop(frame,rect);
end
[frame_height, frame_width] = size(frame);

% Find first ambient frame
if ~isinf(switch_rate)
  first_frames = zeros(frame_height, frame_width, switch_rate+1);
  vid.CurrentTime = (fstart_idx-cumulative_frames)/fps;
  for i = 1:switch_rate
    frame = readFrame(vid);
    if ~isempty(rect)
      im = imcrop(frame(:,:,1), rect);
    else
      im = frame(:,:,1);
    end
    first_frames(:,:,i+1) = im2double(im(:,:,ch));
  end
  first_frames = reshape(first_frames, [numel(first_frames(:,:,1)), size(first_frames,3)]);
  [~,first_ambient_idx] = min(mean(first_frames));
  fstart_idx = fstart_idx + (first_ambient_idx - 1);
end

[tppg, ppg, ppg_tidx] = load_ppg(data_dir, fstart_idx, fstop_idx, fps);

var_gb = frame_height*frame_width*(fstop_idx-fstart_idx)*8/1e9;
disp(sprintf('Image matrix variable will be %0.2g GB', var_gb))
if var_gb > 10
  if strcmpi(questdlg('Variable is more than 10GB. Continue?','Large Variable','Yes','No','No'), 'no')
    return;
  end
end

% Ready for processing!
im_inorm = zeros(frame_height,frame_width,fstop_idx-fstart_idx+1);
% im_i = zeros(iminfo.Height,iminfo.Width,fstop-fstart+1);
% im_ambient = zeros(iminfo.Height,iminfo.Width,max([1, floor((fstop-fstart)/switch_rate)]));
im_ambient = zeros(frame_height, frame_width);
idx = 1;
ambient_idx = 1;
if fstop_idx-fstart_idx+1 > 50
  h = waitbar(0,'Loading data...');
end
drawnow
vid_nframes = ceil(vid.Duration*fps);
vid.CurrentTime = (fstart_idx-cumulative_frames)/fps;
for i = fstart_idx:fstop_idx
  if exist('h')
    waitbar((i-fstart_idx)/(fstop_idx-fstart_idx),h)
  end
  t = i-fstart_idx+1;
  
  if vid_nframes + cumulative_frames < i+1
    cumulative_frames = cumulative_frames + vid_nframes;
    Didx = Didx + 1;
    vid = VideoReader(sprintf(fbase,Didx));
    vid_nframes = ceil(vid.Duration*fps);
  end
  
  frame = readFrame(vid);
%   frame = read(vid,i+1-cumulative_frames);
  if ~isempty(rect)
    im = im2double(imcrop(frame(:,:,1),rect));
  else
    im = im2double(frame(:,:,1));
  end
  if mod(t-1, switch_rate+1) < 1
    im_ambient(:,:,ambient_idx) = im(:,:,ch);
    ambient_idx = ambient_idx + 1;
    if idx > 1
      % Reuse last frame to ensure constant frame rate.
      im_i(:,:,idx) = im_i(:,:,idx-1);
      im_inorm(:,:,idx) = im_i(:,:,idx-1) - im_ambient(:,:,ambient_idx-1);
      idx = idx + 1;
    end
  else
    im_inorm(:,:,idx) = im(:,:,ch) - im_ambient(:,:,max([1, ambient_idx-1]));
%     im_i(:,:,idx) = im(:,:,ch);
    idx = idx + 1;
  end
end
if exist('h')
  waitbar(1,h,'Closing')
  close(h)
  drawnow
end
if (idx-1) ~= size(im_inorm,3)
  warning(sprintf('idx-1 (%u) should be size(im_inorm,3) (%u)',idx-1,size(im_inorm,3)))
  im_inorm = im_inorm(:,:,1:idx-1);
  im_i = im_i(:,:,1:idx-1);
end

% Reflectance target normalization
target_file = [data_dir, '/target/im_target.mat'];
if exist(target_file,'file') == 2
  if strcmpi(questdlg('Normalize by reflectance target?','Normalize','Yes','No','No'), 'yes')
    load(target_file)
    for t = 1:size(im_inorm,3)
      im_inorm(:,:,t) = im_inorm(:,:,t)./mean2(im_target(:,:,t));
    end
  end
else
  disp('No reflectance target file found (target/im_target.mat).')
end

% % Query the user to save im_inorm.mat
% if strcmpi(questdlg('Save im_inorm?','Save Action','Yes','No','No'), 'yes')
%   fprintf(1, 'Saving...')
%   tic
%   filename = sprintf('%s/im_inorm_t%u-%u_s%u_f%u.mat', data_dir, fstart, fstop, switch_rate, fps);
%   save(filename, 'im_inorm', 'tppg', 'ppg', 'fps', '-v7.3')
%   toc
% end

if nargout > 6
  crop_rect = rect;
end

end


function [im_i, im_inorm, tppg, ppg, ppg_tidx, im_ambient] = ...
  load_from_pgm(data_dir, fps, fstart_idx, fstop_idx, switch_rate)

im_i = [];
im_inorm = [];
tppg = [];
ppg = [];
ppg_tidx = [];
im_ambient = [];

D = dir([data_dir, '/*.pgm']);
dash_idx = find(D(end).name == '-');

% Update these each time.
% fps = 100;
% fstart = 0;
% fstop = 999;  % last file to process
% switch_rate = Inf;  % Inf if no switching
ch = 1;  % 1-R, 2-G, 3-B

fbase = [data_dir, D(1).name(1:dash_idx(4)), '%04u.pgm'];
% iminfo = imfinfo(sprintf(fbase,fstart));

frame = imread(sprintf(fbase,fstart_idx));
hr=figure; imshow(frame)
rect = getrect;
close(hr)

[frame_height, frame_width] = size(imcrop(frame,rect));

% Find first ambient frame
if ~isinf(switch_rate)
  first_frames = zeros(frame_height, frame_width, switch_rate+1);
  for i = 0:switch_rate
    im = imcrop(imread(sprintf(fbase,fstart_idx+i)),rect);
    first_frames(:,:,i+1) = im2double(im(:,:,ch));
  end
  first_frames = reshape(first_frames, [numel(first_frames(:,:,1)), size(first_frames,3)]);
  [~,first_ambient_idx] = min(mean(first_frames));
  fstart_idx = fstart_idx + (first_ambient_idx - 1);
end

[tppg, ppg, ppg_tidx] = load_ppg(data_dir, fstart_idx, fstop_idx, fps);

var_gb = frame_height*frame_width*(fstop_idx-fstart_idx)*8/1e9;
if var_gb > 10
  if strcmpi(questdlg('Variable is more than 10GB. Continue?','Large Variable','Yes','No','No'), 'no')
    return;
  end
end

% Ready for processing!
im_inorm = zeros(frame_height,frame_width,fstop_idx-fstart_idx+1);
% im_i = zeros(iminfo.Height,iminfo.Width,fstop-fstart+1);
% im_ambient = zeros(iminfo.Height,iminfo.Width,max([1, floor((fstop-fstart)/switch_rate)]));
im_ambient = zeros(frame_height, frame_width);
idx = 1;
ambient_idx = 1;
if fstop_idx-fstart_idx+1 > 50
  h = waitbar(0,'Loading data...');
end
drawnow
for i = fstart_idx:fstop_idx
  if exist('h')
    waitbar((i-fstart_idx)/(fstop_idx-fstart_idx),h)
  end
  t = i-fstart_idx+1;
  if mod(t-1, switch_rate+1) < 1
    im = im2double(imcrop(imread(sprintf(fbase,i)),rect));
    im_ambient(:,:,ambient_idx) = im(:,:,ch);
    ambient_idx = ambient_idx + 1;
    if idx > 1
      % Reuse last frame to ensure constant frame rate.
%       im_i(:,:,idx) = im_i(:,:,idx-1);
%       im_inorm(:,:,idx) = im_i(:,:,idx-1) - im_ambient(:,:,ambient_idx-1);
      im_inorm(:,:,idx) = im_inorm(:,:,idx-1);
      idx = idx + 1;
    end
  else
    im_illum = im2double(imcrop(imread(sprintf(fbase,i)),rect));
    im_inorm(:,:,idx) = im_illum(:,:,ch) - im_ambient(:,:,max([1, ambient_idx-1]));
%     im_i(:,:,idx) = im_illum(:,:,ch);
    idx = idx + 1;
  end
end
if exist('h')
  waitbar(1,h,'Closing')
  close(h)
end
if (idx-1) ~= size(im_inorm,3)
  warning(sprintf('idx-1 (%u) should be size(im_inorm,3) (%u)',idx-1,size(im_inorm,3)))
  im_inorm = im_inorm(:,:,1:idx-1);
%   im_i = im_i(:,:,1:idx-1);
end

% Reflectance target normalization
target_file = [data_dir, '/target/im_target.mat'];
if exist(target_file,'file') == 2
  if strcmpi(questdlg('Normalize by reflectance target?','Normalize','Yes','No','No'), 'yes')
    load(target_file)
    for t = 1:size(im_inorm,3)
      im_inorm(:,:,t) = im_inorm(:,:,t)./mean2(im_target(:,:,t));
    end
  end
else
  disp('No reflectance target file found (target/im_target.mat).')
end

% Query the user to save im_inorm.mat
if strcmpi(questdlg('Save im_inorm?','Save Action','Yes','No','No'), 'yes')
  fprintf(1, 'Saving...')
  tic
  filename = sprintf('%s/im_inorm_t%u-%u_s%u_f%u.mat', data_dir, fstart_idx, fstop_idx, switch_rate, fps);
  save(filename, 'im_inorm', 'tppg', 'ppg', 'fps', '-v7.3')
  toc
end

end


function [] = make_framecounts(data_dir, fps, filetype)

D = dir([data_dir, '/*.', filetype]);
dash_idx = find(D(end).name == '-');
fbase = [data_dir, '/', D(1).name(1:dash_idx(end)), '%04u.', filetype];

% Find the video file contains the first target frame.
first_video = str2double(D(1).name(dash_idx(end)+1:dash_idx(end)+4));
last_video = str2double(D(end).name(dash_idx(end)+1:dash_idx(end)+4));

framecounts = [];
for Didx = first_video : last_video
  vid = VideoReader(sprintf(fbase,Didx));
  vid_nframes = ceil(vid.Duration*fps);
  framecounts = [framecounts; Didx vid_nframes];
end
csvwrite([data_dir, '/framecounts'],framecounts)

end