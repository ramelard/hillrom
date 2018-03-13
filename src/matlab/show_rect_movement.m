%SHOW_RECT_MOVEMENT    Show how the location of a rectangle changes.
%  SHOW_RECT_MOVEMENT(frames,rect,fps,savefile) plays the movie with the
%  rectangle overlay and plots the mean StO2 for a selected rectangle.
function show_rect_movement(frames,rect,fps,savefile)

if nargin < 4
  savefile = false;
end

if savefile
  vidOut = VideoWriter('rect.avi');
  vidOut.FrameRate = fps;
  open(vidOut)
end

tpause = .1;
nframes = size(frames,3);

hfig = figure;
t = [0:1/fps:nframes/fps];
% h = waitbar(0,'Saving frames...')
for i = 1:nframes
%   waitbar(i/nframes,h)
  
  frame = frames(:,:,i);
  
  hcur = gcf;
  figure(hfig);
  
  imshow(frame, [0 1]);
  r = rectangle('Position',rect);
  set(r,'EdgeColor','r')
  title(sprintf('%g s',t(i)));
  
  if savefile
    frameout = getframe;
    writeVideo(vidOut, frameout);
  end

  figure(hcur);
  drawnow
%   pause(1/fps)
%   pause(tpause)
end

% close(h)

if savefile
  close(vidOut)
end