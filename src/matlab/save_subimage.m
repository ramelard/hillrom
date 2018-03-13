data_dir = 'C:/tmp/standing2/';
dir_out = 'wrist/';

D = dir([data_dir, '/*.pgm']);
last_file = D(end).name;
dash_idx = find(D(end).name == '-');

fbase = [data_dir, D(1).name(1:dash_idx(4)), '%04u.pgm'];
fend = str2double(last_file(dash_idx(end)+1:end-4));  % last file in full capture
iminfo = imfinfo(sprintf(fbase,0));

h = figure;
frame0 = imread(sprintf(fbase,0));
figure, imshow(frame0)
rect = floor(getrect);
close(h);

fbase_indir = [D(1).name(1:dash_idx(4)), '%04u.pgm'];

for i = 0:fend
  im = im2double(imread(sprintf(fbase,i)));
  im = im(rect(2):rect(2)+rect(4), rect(1):rect(1)+rect(3));
  imwrite(im, [data_dir, dir_out, sprintf(fbase_indir,i)], 'pgm');
end