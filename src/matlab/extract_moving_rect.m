function roi = extract_moving_rect(frames,width,height)

if nargin < 3
  height = width;
end

roi = zeros(height, width, size(frames,3));

h=figure;
% Force motion callback so CurrentPoint returns the hovered point, not the
% clicked point.
foo = @(obj,evt) [];
set(h,'WindowButtonMotionFcn',foo)

imshow(frames(:,:,1))
[x,y] = ginput(1);
rect = [x-floor(width/2), y-floor(height/2), width, height];
roi(:,:,1) = frames(rect(2):rect(2)+rect(4)-1, rect(1):rect(1)+rect(3)-1, 1);

for i = 2:size(frames,3)
  imshow(frames(:,:,i));
  hold on;
  r = rectangle('Position',rect);
  hold off;
  title(sprintf('%g/%g',i,size(frames,3)))
  set(r,'EdgeColor','r')
  
  drawnow;
  pause(.05);
  
  pt = get(gca,'CurrentPoint');
  x = pt(1,1);
  y = pt(1,2);
  rect = [x-floor(width/2), y-floor(height/2), width, height];
  
  roi(:,:,i) = frames(rect(2):rect(2)+rect(4)-1, rect(1):rect(1)+rect(3)-1, i);
end

close(h);