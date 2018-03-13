D1 = dir('C:\Users\ramelard\Desktop\data\multicam\flea\*.pgm');
D2 = dir('C:\Users\ramelard\Desktop\data\multicam\gh\*.pgm');
cam1 = [];
cam2 = [];

for i = 1:numel(D1)
  
  im = imread(['C:\Users\ramelard\Desktop\data\multicam\flea\',D1(i).name]);
  cam1(:,:,i) = im2double(im(:,:,1));
  im = imread(['C:\Users\ramelard\Desktop\data\multicam\gh\',D2(i).name]);
  cam2(:,:,i) = im2double(im(:,:,1));
  
end


%%

figure;
for i = 1:size(cam1,3)
  subplot(1,2,1), imshow(cam1(:,:,i))
  subplot(1,2,2), imshow(cam2(:,:,i))
  if strcmpi(input('','s'), 'x')
    break;
  end
end