function cube = make_image_cube(im, stack_size)

if nargin < 1
  im = im2double(imread('cameraman.tif'));
end
if nargin < 2
  stack_size = 50;
end

%xy offset
offset = 5;

T = maketform('affine', [1 .1 0; 0 1 0; 0 0 1] );
% im_sheared = imtransform(im2double(im),T);
im_sheared = im;

[h,w,c] = size(im_sheared);
cube = ones(h+stack_size, w+stack_size, 1);

for i = stack_size:-1:1
  row = stack_size-i+1;
  col = i;
  cube(row:row+h-1, col:col+w-1) = im_sheared;
end

% cube([end-h,end], [1:w]) = 0;
% cube([end-h:end], [1,w]) = 0;