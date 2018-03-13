function [Aout] = minfilt2(im, wsize)

[h,w] = size(im);
Aout = zeros(ceil(h/wsize),ceil(w/wsize));
r = 1;
c = 1;
for i = 1:wsize:size(im,1)-wsize
  for j = 1:wsize:size(im,2)-wsize
    patch = im(i:i+wsize,j:j+wsize);
    Aout(r,c) = min(patch(:));
    c = c+1;
  end
  c = 1;
  r = r+1;
end

end