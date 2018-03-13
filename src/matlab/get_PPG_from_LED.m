% data_dir = 'C:/Users/ramelard/Desktop/data/spie_christian/';
data_dir = 'C:/Users/ramelard/Desktop/data/';

% Update these each time.
fbase = [data_dir, 'fc2_save_2014-08-21-111239-%04u.png'];
fstart = 0;
fend = 73;
ch = 3;  % 1-R, 2-G, 3-B
fps = 13.89;
fps = 15;
switch_rate = Inf;  % Inf if no switching

im_inorm = zeros(768,1024,fend-fstart);
im_i = zeros(768,1024,fend-fstart);
idx = 1;
im_ambient = zeros(768,1024,3);
h = waitbar(0,'Reading images');
for i = fstart:fend
  waitbar((i-fstart)/(fend-fstart),h)
  t = i-fstart+1;
  if mod(t-1, switch_rate+1) < 1
    im_ambient = im2double(imread(sprintf(fbase,i)));
    if idx > 1
      im_i(:,:,idx) = im_i(:,:,idx-1);
      im_inorm(:,:,idx) = im_inorm(:,:,idx-1);
      idx = idx + 1;
    end
  else
    im_illum = im2double(imread(sprintf(fbase,i)));
%     im_illum = rgb2hsv(imread(sprintf(fbase,i)));
    im_inorm(:,:,idx) = im_illum(:,:,ch) - im_ambient(:,:,ch);
    im_i(:,:,idx) = im_illum(:,:,ch);
    idx = idx + 1;
  end
end
close(h)
im_inorm = im_inorm(:,:,1:idx-1);
im_i = im_i(:,:,1:idx-1);
%%
if ~exist('LEDrect','var')
  LEDrect = [];
end
% R = plot_pixel_vals(shiftdim(im_inorm,2),'fps',fps,'inv','roi')
% [R,LEDrect] = plot_pixel_vals(shiftdim(im_i,2),'fps',fps,'roi','rect',LEDrect);
[R,LEDrect] = plot_pixel_vals(shiftdim(im_inorm,2),'fps',fps,'roi','rect',LEDrect);
ylabel('reflectance (a.u.)')
[freq,fpow] = plot_power_spectrum(R,fps);

T = size(im_inorm,3)/fps;
figure;
plot(linspace(0,T,numel(R)),-log(R));
xlabel('t (s)')
ylabel('absorbance (a.u.)')
