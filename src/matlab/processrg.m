data_dir = 'data/72bpm_red/';
data_dir = 'C:/Users/ramelard/Desktop/data/';

% Update these each time.
fbase = [data_dir, 'fc2_save_2014-07-22-154236-%04u.png'];
fstart = 11;
fend = 149;
% ch = 2;  % 1-R, 2-G, 3-B
fps = 13.89;
switch_rate = 14;  % Inf if no switching

imR_inorm = zeros(768,1024,fend-fstart);
imG_inorm = zeros(768,1024,fend-fstart);
imR_i = zeros(768,1024,fend-fstart);
imG_i = zeros(768,1024,fend-fstart);
idx = 1;
im_ambient = zeros(768,1024,3);
h = waitbar(0,'Reading images');
for i = fstart:fend
  waitbar((i-fstart)/(fend-fstart),h)
  t = i-fstart+1;
  if mod(t-1, switch_rate+1) < 1
    im_ambient = im2double(imread(sprintf(fbase,i)));
    if idx > 1
      imR_i(:,:,idx) = imR_i(:,:,idx-1);
      imG_i(:,:,idx) = imG_i(:,:,idx-1);
      imR_inorm(:,:,idx) = imR_inorm(:,:,idx-1);
      imG_inorm(:,:,idx) = imG_inorm(:,:,idx-1);
      idx = idx + 1;
    end
  else
    if mod(t-1,switch_rate+1) < 3
      % For now, drop 1st frame that is over-illuminated
      if idx > 1
        imR_i(:,:,idx) = imR_i(:,:,idx-1);
        imG_i(:,:,idx) = imG_i(:,:,idx-1);
        imR_inorm(:,:,idx) = imR_inorm(:,:,idx-1);
        imG_inorm(:,:,idx) = imG_inorm(:,:,idx-1);
        idx = idx + 1;
      end
      continue;
    end
    im_illum = im2double(imread(sprintf(fbase,i)));
    imR_inorm(:,:,idx) = im_illum(:,:,1) - im_ambient(:,:,1);
    imG_inorm(:,:,idx) = im_illum(:,:,2) - im_ambient(:,:,2);
    imR_i(:,:,idx) = im_illum(:,:,1);
    imG_i(:,:,idx) = im_illum(:,:,2);
    idx = idx + 1;
  end
end
close(h)
imR_inorm = imR_inorm(:,:,1:idx-1);
imG_inorm = imG_inorm(:,:,1:idx-1);
imR_i = imR_i(:,:,1:idx-1);
imG_i = imG_i(:,:,1:idx-1);
%%
if ~exist('rect','var')
  rect = [];
end
% R = plot_pixel_vals(shiftdim(im_inorm,2),'fps',fps,'inv','roi')
[Rr,rect] = plot_pixel_vals(shiftdim(imR_inorm,2),'fps',fps,'roi','rect',rect); title('Red')
[Rg,rect] = plot_pixel_vals(shiftdim(imG_inorm,2),'fps',fps,'roi','rect',rect); title('Green')
[freqr,fpow] = plot_power_spectrum(Rr,fps); title('Red')
[freqg,fpow] = plot_power_spectrum(Rg,fps); title('Green')

%%
if ~exist('normRect','var')
  normRect = [];
end
if ~exist('pulseRect','var')
  pulseRect = [];
end
[Rrnorm,normRect] = plot_pixel_vals(shiftdim(imR_inorm,2),'fps',fps,'roi','rect',normRect);
[Rrpulse,pulseRect] = plot_pixel_vals(shiftdim(imR_inorm,2),'fps',fps,'roi','rect',pulseRect);
[Rgnorm,normRect] = plot_pixel_vals(shiftdim(imG_inorm,2),'fps',fps,'roi','rect',normRect);
[Rgpulse,pulseRect] = plot_pixel_vals(shiftdim(imG_inorm,2),'fps',fps,'roi','rect',pulseRect);
figure,plot(Rrnorm./Rrpulse),title('Red Normalized')  % absorption
[freq,fpow] = plot_power_spectrum(Rrnorm./Rrpulse,fps); title('Red Normalized')
figure,plot(Rgnorm./Rgpulse),title('Green Normalized')  % absorption
[freq,fpow] = plot_power_spectrum(Rgnorm./Rgpulse,fps); title('Green Normalized')

% Rg = Rpulse./Rnorm;

%% EI
EI = log(1./Rg) - log(1./Rr);
figure,plot(EI),title('EI')
[freq,fpow] = plot_power_spectrum(EI,fps); title('EI')
