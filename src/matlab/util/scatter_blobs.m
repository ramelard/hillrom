function [] = scatter_blobs(x, y, s, c)

if nargin < 3
  s = 34;
end
if nargin < 4
  c = 'k';
end

[M,ai,mi] = unique([x y],'rows');
for i = 1:numel(ai)
  Mc(i) = sum(mi==i);
end
scatter(M(:,1), M(:,2), mat2gray(Mc)*70+10, 'k', 'filled')