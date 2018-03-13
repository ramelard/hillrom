%Blood flow estimation
% Alexander Wong, (a28wong@uwaterloo.ca), University of Waterloo, 2016
function cI = bloodflowest(I,blocksize,type)
switch lower(type)
  case 'iter'
    cI = bloodflow_iter(I,blocksize);
  case 'blur'
    cI = bloodflow_blurring(I,blocksize);
end
end

function cI = bloodflow_iter(I, blocksize)

cI = zeros(size(I));
[H,W,T] = size(I);
for t = 1:T
  It = I(:,:,t);
  scale = blocksize;
  It = imresize(It,1/scale,'nearest');


  %Set up initial estimate
  It = imresize(It,2,'bilinear');   
  %Set up compensation function
  compfct = fspecial('gaussian',[15 15],0.45*2);
  compfct = compfct./sum(compfct(:)); 
  %Set up compensation function in frequency domain
  compfct_hat = psf2otf(compfct,size(It));
  compfct_conj = conj(compfct_hat);

  %initial conditions
  It = edgetaper(It,compfct);     
%   minI = min(It(:));
%   It = It + minI;
  
  measurements = It;
  u = measurements;
  numiter = 0;
%   noisefloor = 0.000001;
  noisefloor = 1E-0;

      for i=1:numiter
  %             i
          est_conv      = real(ifftn(fftn(u).*compfct_hat));
          relative_blur = measurements./(est_conv+noisefloor);
          error_est      = real(ifftn(fftn(relative_blur-1).*compfct_conj));   
          u = u.* exp(error_est);
      end
%   u = u - minI;
  cI(:,:,t) = imresize(u,[H W],'bilinear');  
end
cI = imerode(cI,strel('disk',floor(blocksize/3),8));
% cI = imerode(cI,strel('line',floor(blocksize/1),0));
end

function cI = bloodflow_blurring(I, blocksize)
  sigma = blocksize/2;
  blur_kernel = fspecial('gaussian', 3*sigma, sigma);
  cI = imfilter(I, blur_kernel);
end