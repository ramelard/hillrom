% Use Parzen windowing to 
function [p] = parzen_sampling(im_votes, win, res)

if nargin < 2 || isempty(win)
  sigma = 3;
  win = fspecial('gaussian',sigma*3,sigma);
end
if nargin < 3
  res = 1;
end

% x, y, frequency
B = im_votes';
[X,Y] = meshgrid(1:size(im_votes,1), 1:size(im_votes,2));
xyf = [X(:), Y(:), B(:)];

p = parzen2d(xyf, res, win);
p = p(1:end-1,1:end-1)';