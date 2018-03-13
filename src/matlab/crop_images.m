read_dir = 'X:/data/lowel/day3/sunglasses2/';
write_dir = 'X:/data/lowel/day3/sunglasses2/head/';

if ~exist(write_dir, 'file')
  disp(['Creating directory ', write_dir])
  mkdir(write_dir)
end

if ~strcmpi(read_dir, write_dir)
  ppg_data_file = sprintf('%stppg_ppg.mat', read_dir);
  if exist(ppg_data_file, 'file') == 2
    copyfile(ppg_data_file, sprintf('%stppg_ppg.mat', write_dir))
  end
end

D = dir([read_dir, '/*.pgm']);
hwait = waitbar(0,'Reading images');
drawnow
rect = [];
for i = 1:numel(D)
  waitbar(i/numel(D),hwait)
  im_illum = imread([read_dir, D(i).name]);
  if isempty(rect)
    hr=figure; imshow(im_illum)
    rect = getrect;
    close(hr)
  end
  imwrite(imcrop(im_illum,rect), [write_dir, D(i).name]);
end
close(hwait)