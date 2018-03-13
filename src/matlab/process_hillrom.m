% load_path();

data_dir = 'C:/Users/ramelard/Desktop/hillrom/demo';

% Update these each time.
fps = 60;
fstart = fps*0;
fstop = round(fps*15);  % last file to process
switch_rate = Inf;  % Inf if no switching
ch = 1;  % 1-R, 2-G, 3-B
rect = false;  % false, [], or a rect

[~, im_inorm, tppg, ppg, ppg_tidx, ~] = ...
  load_ppgi_images(data_dir, fps, fstart, fstop, switch_rate, rect);
if isempty(tppg)
  tppg = [0:size(im_inorm,3)-1]/fps;
  ppg = sin(2*pi*tppg*60/60);
end


%% GUI processing
block_size = 6;
[hr,rr] = extract_vitals(im_inorm, block_size);
