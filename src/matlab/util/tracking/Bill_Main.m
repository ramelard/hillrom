%% Input args
video_path = './Robert_Head_Sway/';

% Part of image to track (It is better to track a small portion (e.g. nose)
start_pos = [291, 274];
track_target_sz = [27, 64];

% Size of head used by out_im
height_top = 275;
height_bottom = 400;
width = 450;

%% Code
% Use code from tracker_release2 to get position for each frame
% [positions, time, imgs] = Bill_run_tracker(video_path, start_pos, track_target_sz, '*.pgm', 0);
[positions, time, imgs] = Bill_run_tracker(start_pos, track_target_sz, im_inorm, 0);


% Get output video that moves with head
[ out_im ] = Bill_get_out_imgs( positions, imgs, height_top, height_bottom, width);

%% Save Video
%save_video('Processed_Robert_Sway', out_im, 'Motion JPEG AVI', 'FrameRate', 60);
%save('processed_robert_sway.mat', 'out_im')

%%
yx = [115 146];
hw = [35 18];
[positions, time, imgs] = Bill_run_tracker('C:/Users/ramelard/Desktop/data/sam/', yx, hw, '*.pgm', 1);

%%
% Nose
yx = [657 708]; hw = [86 143];
% Eyebrow
% yx = [503 552]; hw = [51 91];
[positions, time, imgs] = Bill_run_tracker('C:/Users/ramelard/Desktop/data/sam2/', yx, hw, '*.pgm', 1);

%% hadfield
yx = [210 160];
hw = [23 61];
[positions, time, imgs] = Bill_run_tracker(yx, hw, im_inorm, 0);

%%
imgs2 = imgs;
for i = 2:size(imgs,3)
  imgs2(:,:,i) = imtranslate(imgs(:,:,i), -fliplr(positions(i,:) - positions(1,:)));
end

block_size = [10 10];
t = [0:size(imgs2,3)-1]./29;
tsig = sin(2*pi*t*105/60);
tsig = sum(sin(2*pi*t'*[100:110]./60),2)';
process_head(imgs2, 29, t, tsig, block_size);


