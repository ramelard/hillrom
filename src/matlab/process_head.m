function [frames_out, Acam_out] = process_head(im_inorm, fps, tppg, ppg, block_size, mask, select_roi)

if nargin < 6
  mask = true(size(im_inorm(:,:,1)));
end
if nargin < 7
  answer = questdlg('Select ROI?','ROI Selection','Yes','No','No');
  select_roi = strcmpi(answer, 'yes');
end

if nargin < 5
  block_size = [30 30];
end

% fstart = 1;
% fend = fstart + nframes;

  function r = block_border(block_struct)
    r = block_struct.data;
    r([1,end],:) = 0;
    r(:,[1,end]) = 0;
  end

  function b = block_pixel_vals(block_struct)
    RCsize = block_struct.blockSize;
    if sum(RCsize) < 2
      b = 0;
    else
      [b,~] = plot_pixel_vals(block_struct.data,'fps',fps,'roi','rect',[1 1 RCsize(2)-1 RCsize(1)-1]);
      b = b';
    end
  end

if select_roi
  h=figure;
  frame1 = im_inorm(:,:,1);
  frame1_no_encoding = frame1(2:end,2:end);
  imshow(frame1, [min(frame1_no_encoding(:)) max(frame1_no_encoding(:))])
  head_rect = getrect;
  head_rect = floor(head_rect);
  % im_head = imcrop(im_i(:,:,1), head_rect); -- this can't keep temporal
  im_head = im_inorm(head_rect(2):head_rect(2)+head_rect(4), head_rect(1):head_rect(1)+head_rect(3), :);
  mask = mask(head_rect(2):head_rect(2)+head_rect(4), head_rect(1):head_rect(1)+head_rect(3));
  close(h)
else
  im_head = im_inorm;
end
% Make it a clean grid (no partial blocks at the edges).
[h,w,~] = size(im_head);
hend = floor(h/block_size(1))*block_size(1);
wend = floor(w/block_size(2))*block_size(2);
im_head = im_head(1:hend, 1:wend, :);
mask = mask(1:hend, 1:wend, :);

if all(block_size > 1)
%   [h,w,T] = size(im_head);
%   B = zeros(h/block_size(1), w/block_size(2), T);
%   fprintf(1,'Min filtering...')
%   for k = 1:T
%     B(:,:,k) = minfilt2(im_head(:,:,k), block_size(1));
%   end
%   fprintf(1,'Done\n')

  B = imresize(im_head,1/block_size(1),'box');
  Rcam = reshape(permute(B,[3 1 2]),size(B,3),[]);
else
  Rcam = reshape(permute(im_head,[3 1 2]),size(im_head,3),[]);
end

tppg2 = linspace(tppg(1),tppg(end),size(Rcam,1));
ppg2 = spline([0:numel(tppg)-1]*mean(diff(tppg)),ppg,tppg2);
Acam = -log(1+Rcam);
% Acam = Rcam; warning('ACAM=RCAM FOR LSI DATA');

if nargout > 0
  frames_out = im_head;
  Acam_out = Acam;
else
  % Acam_detrend = hrv_detrend(Acam);
  gui_ppgi_analyzer(im_head, block_size, Acam, fps, ppg2, tppg2, mask);
  % GUI_Demo(im_head, block_size, Acam, fps, ppg2, tppg2, mask);
end

end
