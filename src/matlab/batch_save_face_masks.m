data_dir_fmt = 'X:/data/DATA COLLECTION/complete/P%0.2u/torso_overhead/';
% data_dir_fmt = 'D:/Research Data/2015 PPGI Trial 1/P%0.2u/1torso_overhead/';
out_dir = 'C:/Users/ramelard/Dropbox/papers/2016.2 Entropy Fusion [BOE]/results/face masks/manual/P%0.2u_mask.mat';
facedetect = 'manual';

for P = 1:24
  data_dir = sprintf(data_dir_fmt, P);
  D = dir(sprintf([data_dir, '*.avi'], P));
  vin = VideoReader([data_dir, D(1).name]);
  frame1 = read(vin,1);
  mask_face = rotate_and_face_detect(frame1,facedetect);
  
  if ~isempty(mask_face)
    save(sprintf(out_dir, P), 'mask_face')
  end
end

