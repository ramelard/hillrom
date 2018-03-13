function [corrected_frames,roi_rect,translations] = motion_correction(frames, roi)

if nargin < 2 || isempty(roi)
  figure, imshow(frames(:,:,1))
  roi = floor(getrect);
  close;
end

yx = [roi(2) roi(1)];
hw = [roi(4) roi(3)];
[positions, time, ~] = Bill_run_tracker(yx, hw, frames, 0);

disp('Positions estimated. Correcting frames.')
corrected_frames = frames;
hwait = waitbar(0,'Translating...');
for i = 2:size(frames,3)
  waitbar(i/size(frames,3),hwait,sprintf('Translating frame %u',i))
  translation = -fliplr(positions(i,:) - positions(1,:));
  corrected_frames(:,:,i) = imtranslate(frames(:,:,i), translation);
end
close(hwait)

if nargout > 1
  roi_rect = roi;
end
if nargout > 2
  nframes = size(frames,3);
  translations = zeros(2,nframes-1);
  for i = 2:nframes
    translations(:,i-1) = -fliplr(positions(i,:) - positions(1,:));
  end
end

% block_size = [10 10];
% t = [0:size(corrected_frames,3)-1]./29;
% tsig = sin(2*pi*t*105/60);
% tsig = sum(sin(2*pi*t'*[100:110]./60),2)';
% process_head(corrected_frames, 29, t, tsig, block_size);

