[tppg, ppg] = ReadEasyPulse;
save('X:/data/omnitv/p1/tppg_ppg.mat', 'tppg', 'ppg');

%%
data_dir = 'X:/data/omnitv/p1/';
load_path();

% Update these each time.
fps = 60;
fstart = fps*8;
fstop = fps*13;  % last file to process
switch_rate = Inf;  % Inf if no switching
ch = 1;  % 1-R, 2-G, 3-B
rect = false;  % can also be a rect

[~, im_inorm, tppg, ppg, ppg_tidx, ~] = ...
  load_ppgi_images(data_dir, fps, fstart, fstop, switch_rate, rect);

%%
mask = true(size(im_inorm2(:,:,1)));
if strcmpi(questdlg('Use mask?'),'yes')
  if exist([data_dir, '/mask.mat'],'file') && ...
     strcmpi(questdlg('Load existing mask?'),'yes')
    load([data_dir, '/mask.mat'])
  else
    figure;
    mask = roipoly(im_inorm2(:,:,1));
    close;
    save([data_dir, '/mask.mat'],'mask')
  end
end
block_size = [10 10];
% block_size = [6 6];
process_head(im_inorm2-.05, fps, tppg-tppg(1), ppg, block_size, mask);