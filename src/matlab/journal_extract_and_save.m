function journal_extract_and_save()

dir_base = 'C:/Users/ramelard/Desktop/data/experiments/%s/test%s/';
users = {'RA','AW','CS','FK','BL'};
% Values from powerpoint 01.23.2015
rects = {{[180 55 154 259], [320 249 23 32]}, ... % RA
         {[234 29 155 216], [370 239 21 32]}, ...   % AW
         {[168 34 155 284], [274 240 34 53]}, ...   % CS
         {[243 19 171 274], [318 227 35 39]}, ...  % FK
         {[291 30 124 214], [348 227 29 39]}};     % BL
fstarts = {[0,0],[0,4000],[0,350],[0,0],[4000,1000]};

for i = 1:numel(users)
  % Experiment 1 - Short distance
  data_dir = sprintf(dir_base, users{i}, '1');
  rect = rects{i}{1};
  fstart = fstarts{i}(1);
  [im_i, im_inorm, fps, ppg, tppg, ppg_tidx] = ...
    import_images(data_dir, fstart, fstart + 1000);
  journal_plots(im_i, im_inorm, fps, ppg, tppg, ppg_tidx, 'rect', rect, ...
                'save',sprintf('%s_e1_',lower(users{i})));
  close all;
  
  % Experiment 2 - Long distance
  data_dir = sprintf(dir_base, users{i}, '5');
  rect = rects{i}{2};
  fstart = fstarts{i}(2);
  [im_i, im_inorm, fps, ppg, tppg, ppg_tidx] = ...
    import_images(data_dir, fstart, fstart + 1000);
  journal_plots(im_i, im_inorm, fps, ppg, tppg, ppg_tidx, 'rect', rect, ...
                'save',sprintf('%s_e2_',lower(users{i})));
  close all;
end

end

function [im_i, im_inorm, fps, ppg, tppg, ppg_tidx] = ...
  import_images(data_dir, fstart, fstop)

D = dir([data_dir, '/*.pgm']);
last_file = D(end).name;
dash_idx = find(D(end).name == '-');

% Update these each time.
fps = 100;
% fstart = 1000;
% fstop = 2000;  % last file to process
switch_rate = 3;  % Inf if no switching
ch = 1;  % 1-R, 2-G, 3-B

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

% Try to load ppg data from saved file.
ppg_tidx_file = sprintf('%sppg_tidx.mat', data_dir);
if exist(ppg_tidx_file, 'file') == 2
  fprintf(1,'Loading data file %s\n', ppg_tidx_file)
  load(ppg_tidx_file)
  
  sidx = find(ppg_tidx-fstart >= 0, 1, 'First');
  eidx = find(ppg_tidx-fstop > 0, 1, 'First') - 1;
  ppg_tidx = ppg_tidx(sidx:eidx) - fstart;
  
  if ppg_tidx(1) == 0
    ppg_tidx(1) = 1;
  end
end

% Ready for processing!
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

end