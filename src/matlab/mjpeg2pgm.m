v = VideoReader('X:\data\lowel\day3\mrshughson\3head\fc2_save_2015-12-03-150016-0000.avi');
write_dir = 'X:/data/lowel/day3/test/fc2_save_2015-12-03-104406-';

hwait = waitbar(0,'Reading images');
drawnow
rect = [];
% while hasFrame(v)
for i = 1:300
  im_illum = read(v,i);
    waitbar(i/300,hwait)
%     im_illum = imread([read_dir, D(i).name]);
    if isempty(rect)
      hr=figure; imshow(im_illum)
      rect = getrect;
      close(hr)
    end
    imwrite(imcrop(im_illum,rect), sprintf('%s%0.4u.pgm', write_dir, i));
end
close(hwait)