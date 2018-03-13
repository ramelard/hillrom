% R = plot_pixel_red_vals(filename)
% R = plot_pixel_red_vals(frames)
% R = plot_pixel_red_vals(...,coord)
% R = plot_pixel_red_vals(...,coord,patch_width)
%
% Frames must be [w h t]
function [R, pcoord] = plot_pixel_vals(frames, varargin)

patch_width = 1;  % !!
coord = [];
rect = [];
frame_rate = 60;  % default frame rate
plot_inv = false;
select_rect = false;
zoom_first = false;
gen_plots = false;
ch = 1;  % 1-R, 2-G, 3-B
i = 1;
% for i = 1:2:nargin-1
while i < nargin
  pname = varargin{i};
  if strcmpi(pname,'coord')
    pval = varargin{i+1};
    coord = pval;
    i = i + 1;
  elseif strcmpi(pname,'rect')
    pval = varargin{i+1};
    rect = pval;
    i = i + 1;
  elseif strcmpi(pname,'ch')
    pval = varargin{i+1};
    ch = pval;
    i = i + 1;
  elseif strcmpi(pname,'fps')
    pval = varargin{i+1};
    frame_rate = pval;
    i = i + 1;
  elseif strcmpi(pname,'zoom')
    pval = varargin{i+1};
    zoom_first = pval;
    i = i + 1;
  elseif strcmpi(pname,'roi')
    select_rect = true;
  elseif strcmpi(pname,'inv')
    plot_inv = true;
  elseif strcmpi(pname,'gen_plots')
    gen_plots = true;
  end
  i = i + 1;
end

% if isa(frames,'char')
%   % Video file was specified.
%   filename = frames;
%   vidObj = VideoReader(filename);
%   frame_rate = vidObj.FrameRate;
%   fprintf(1,'Retrieving frames...')
%   frames = get_video_frames(vidObj);
%   fprintf(1,'Done\n')
% elseif isa(frames,'double') || isa(frames,'uint8') || isa(frames,'uint16')
%   % Frames were specified as a matrix.
%   frames_matrix = frames;
%   frames = struct([]);
%   for i = 1:size(frames_matrix,1)
%     frames(i).cdata = squeeze(frames_matrix(i,:,:,:));
%   end
% elseif isa(frames,'struct')
%   % Frames were given as an array of struct with cdata.
% else
%   error('Did not expect frames to be %s', class(frames))
% end

% nFrames = numel(frames);
nFrames = size(frames, 3);
% fprintf('Frame rate: %g\n',frame_rate)

% Only process 1 channel of the frames.
% for i = 1:nFrames
%   frames(i).cdata = frames(i).cdata(:,:,ch);
% end

t = [0:nFrames-1]./frame_rate;

if select_rect
  if isempty(rect)
    frame1 = frames(:,:,1);
    frame1_no_encoding = frame1(2:end, 2:end);
    figure;
    imshow(im2double(frame1),[min(frame1_no_encoding(:)) max(frame1_no_encoding(:))]);
    [rect,rvals] = get_rvals_with_rect(frames, rect, zoom_first);
    r = rectangle('Position',rect);
    set(r,'EdgeColor','r')
  else
    [rect,rvals] = get_rvals_with_rect(frames, rect, zoom_first);
    if gen_plots
      figure;
      imshow(im2double(frames(:,:,1)),[]);
      r = rectangle('Position',rect);
      set(r,'EdgeColor','r')
    end
  end
else
  [coord,rvals] = get_rvals_with_points(frames, coord);
  x0 = coord(1);
  y0 = coord(2);
  plot([x0 x0*0]',[y0 y0*0]','x','LineWidth',2);
end


if nargout == 0
  figure
  if plot_inv
    plot(t, 1./rvals)
  else
    plot(t, rvals)
  end
  xlabel('t (s)'), xlim([0, t(end)])
  legend('1','2','3','4','5','6')
end

if nargout > 1
  if select_rect
    pcoord = rect;
  else
    pcoord = coord;
  end
end
if nargout > 0
  R = rvals;
end

end


function [rect,rvals] = get_rvals_with_rect(frames, rect, zoom_first)

nFrames = size(frames,3);
frame1 = frames(:,:,end);

if nargin < 2 || isempty(rect)
  % Select point(s)
  h = figure;
  frame1_no_encoding = frame1(2:end, 2:end);
  imshow(im2double(frame1(:,:,1)),[min(frame1_no_encoding(:)) max(frame1_no_encoding(:))]);
  if zoom_first
    zoom_rect = getrect;
    xlim([zoom_rect(1), zoom_rect(1)+zoom_rect(3)])
    ylim([zoom_rect(2), zoom_rect(2)+zoom_rect(4)])
  end
  rect = getrect;
  rect = floor(rect);
  close(h);
end
  
[y,x] = meshgrid(rect(2):rect(2)+rect(4), rect(1):rect(1)+rect(3));
x = uint32(x(:));
y = uint32(y(:));

idx = sub2ind([size(frame1,1), size(frame1,2)], y, x);

rvals = zeros(1,nFrames);
for k = 1:nFrames
  frameR = frames(:,:,k);
  rvals(k) = mean(frameR(idx));
end

% x = mean(x);
% y = mean(y);

end

function [coord,rvals] = get_rvals_with_points(frames, coord)

nFrames = size(frames,3);

% Select point(s)
h = figure;
frame1 = frames(:,:,1);
imshow(im2double(frame1),[]);
if nargin < 2 || isempty(coord)
  [x,y] = ginput;
%   x = uint8(x); y = uint8(y);
  for i = 1:numel(x)
    fprintf('x=%u, y=%u\n',x(i),y(i))
  end
else
  x = coord(1);
  y = coord(2);
end

x = uint32(x);
y = uint32(y);
  
idx = sub2ind([size(frame1,1), size(frame1,2)], y, x);
close(h);

rvals = zeros(numel(x),nFrames);
for k = 1:nFrames
  frameR = frames(:,:,k);
  rvals(:,k) = reshape(frameR(idx), [numel(x), 1]);
end

end