% crop_video(filename_in, filename_out, t1, tn, vwriter_profile)
% crop_video(filename_in, filename_out, t1, tn, vwriter_profile, rect)
function crop_video(filename_in, filename_out, t1, tn, vwriter_profile, rect)

if nargin < 6
  rect = [];
end

fprintf('Creating VideoReader...')
vidIn = VideoReader(filename_in);
fprintf('Done\n')

frame1 = read(vidIn,t1*vidIn.FrameRate+1);
if isempty(rect)
  h=figure;
  imshow(frame1)
  rect = floor(getrect)
  close(h);
end

vidOut = VideoWriter(filename_out, vwriter_profile);
vidOut.FrameRate = vidIn.FrameRate;
open(vidOut)

if isempty(tn)
  tn = vidIn.Duration - t1;
end

nframes = (tn-t1)*vidIn.FrameRate;
frame1 = floor(t1*vidIn.FrameRate)+1;
frameN = floor(tn*vidIn.FrameRate);
h = waitbar(0,'Please wait...');
for framei = frame1:frameN
  frame = read(vidIn,framei);
  frame_crop = frame(rect(2):rect(2)+rect(4), rect(1):rect(1)+rect(3), :);
  writeVideo(vidOut, frame_crop);
  waitbar((framei-frame1)/(frameN-frame1),h,sprintf('Cropping and writing... (%u)',framei-frame1))
end
close(h);
close(vidOut);