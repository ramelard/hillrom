function [tppgi,ppgi] = extract_whole_bpw(data_dir,x,y,tstart,tstop)

if nargin < 2
  x = [];
  y = [];
end

if nargin < 4
  tstart = 0;
  tstop = Inf;
end

load_path();

% Update these each time.
block_size = 16;
fps = 60;
fstart = fps*tstart;
fstop = fstart + round(fps*10);
switch_rate = Inf;  % Inf if no switching
rect = false;  % can also be a rect

ppgi = zeros(1,9000);
ppgi_idx = 1;
while fstart/fps < tstop
  disp(sprintf('%.1f-%.1fs',fstart/fps,fstop/fps))
  try
  [~, im_inorm, ~, ~, ~, ~] = ...
    load_ppgi_images(data_dir, fps, fstart, fstop, switch_rate, rect);
  catch err
    disp('Reached the end')
    break;
  end
  
  B = imresize(im_inorm, 1/block_size, 'box');
  Rcam = reshape(permute(B, [3 1 2]), size(B, 3), []);
  Acam = -log(1+Rcam);
  npts = size(Acam, 1);

%   Acam = hrv_detrend(Acam);

  if isempty(x) && isempty(y)
    [freq, Acam_pow] = compute_spectral_power(Acam, fps);
    Acam_pow(1,:) = 0;

    f40bpm = floor(40/60*npts/fps) + 1;
    f200bpm = floor(200/60*npts/fps) + 1;
    % strength_map = compute_spectral_entropy(Acam_pow(f40bpm:f200bpm,:));
    strength_map = compute_spectral_entropy(Acam_pow);

    [~, fmax_idx] = max(Acam_pow, [], 1);
    entropy_mask = (fmax_idx > f200bpm);
    strength_map(entropy_mask) = 1;
    strength_map = reshape(strength_map, [size(B,1) size(B,2)]);
    
    figure,
    imshow(im_inorm(:,:,1),[])
    figure;
    set(gcf,'position',[1 1 771   699])
    subplot(2,1,1), imshow(strength_map,[])
    found_points = false;
    while ~found_points
      coord = floor(ginput(1));
      if isempty(coord)
        return;
      end
      x = coord(1);
      y = coord(2);
      
      disp(sprintf('(x,y)=(%u,%u)',x,y))
      
      subplot(2,2,3)
      Apix = Acam(:,sub2ind(size(strength_map),y,x));
      low_freq = 0.5;
      high_freq = 3.5;
      Apix_ts = timeseries(Apix, [1:numel(Apix)]./fps);
      Apix = idealfilter(Apix_ts, [low_freq high_freq], 'pass');
      Apix = Apix.Data;

      plot(Apix)
      
      subplot(2,2,4)
      [freq,pow] = compute_spectral_power(Apix,fps);
      plot(freq,pow)
      user_happy = questdlg('Extract the waveform here?','Extraction','Yes','No','Yes');
      found_points = strcmpi(user_happy, 'yes');
    end
  end
  
  % Extract at x,y
  [H,W,T] = size(B);
  bpw = Acam(:,sub2ind([H,W],y,x));
  ppgi(ppgi_idx : ppgi_idx+numel(bpw)-1) = bpw;
  
  ppgi_idx = ppgi_idx + numel(bpw);
  fstart = fstop + 1;
  fstop = fstop + round(fps*10);
  
  tppgi = [1:numel(ppgi)]./fps;
end

ppgi(ppgi_idx:end) = [];
tppgi = [1:numel(ppgi)]./fps;
save_path = sprintf('%s/tppgi_ppgi_x%uy%u.mat',data_dir,x,y);
disp(sprintf('Saving %s', save_path))
save(save_path,'tppgi','ppgi')