function varargout = gui_ppgi_analyzer(varargin)
% GUI_PPGI_ANALYZER MATLAB code for gui_ppgi_analyzer.fig
%      GUI_PPGI_ANALYZER, by itself, creates a new GUI_PPGI_ANALYZER or raises the existing
%      singleton*.
%
%      H = GUI_PPGI_ANALYZER returns the handle to a new GUI_PPGI_ANALYZER or the handle to
%      the existing singleton*.
%
%      GUI_PPGI_ANALYZER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_PPGI_ANALYZER.M with the given input arguments.
%
%      GUI_PPGI_ANALYZER('Property','Value',...) creates a new GUI_PPGI_ANALYZER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_ppgi_analyzer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_ppgi_analyzer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_ppgi_analyzer

% Last Modified by GUIDE v2.5 21-Sep-2017 10:43:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_ppgi_analyzer_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_ppgi_analyzer_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_ppgi_analyzer is made visible.
function gui_ppgi_analyzer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_ppgi_analyzer (see VARARGIN)

% Choose default command line output for gui_ppgi_analyzer
handles.output = hObject;

handles.im = varargin{1};
handles.block_size = varargin{2};
handles.Acam0 = varargin{3};
handles.Acam_detrend = [];
handles.Acam = [];  % don't know bandpass yet
handles.Apix = [];  % no pixel clicked yet
handles.Acam_filt = [];  % created filtered signal when necessary
handles.fps = varargin{4};
handles.ppg = varargin{5};
handles.tppg = varargin{6};
handles.mask = varargin{7};
handles.vistype = 'blur';

% Initial filter bandwidth
handles.denoisefun = @denoise_ideal;
low_freq = str2double(handles.lpftext.String);
high_freq = str2double(handles.hpftext.String);
Acam_ts = timeseries(handles.Acam0, handles.tppg);
Acam = idealfilter(Acam_ts, [low_freq high_freq], 'pass');
handles.Acam = real(Acam.Data);
handles.Acam_detrend = hrv_detrend(handles.Acam);

% Update handles structure
guidata(hObject, handles);

% Default to displaying SNR
display_success_map(handles, 'SNR');

% UIWAIT makes gui_ppgi_analyzer wait for user response (see UIRESUME)
% uiwait(handles.figure1);


function display_success_map(handles, plot_type)

axes(handles.axes1)

if get(handles.cb_detrend, 'value') == 1
  Acam = handles.Acam_detrend;
else
  Acam = handles.Acam;
end

npts = size(Acam,1);
f30bpm = ceil(30/60*npts/handles.fps) + 1;
f200bpm = floor(200/60*npts/handles.fps) + 1;
[freq, Acam_pow] = plot_power_spectrum(Acam, handles.fps);
Acam_pow(1,:) = 0;
Acam_mag = sqrt(Acam_pow);
% Remove DC and normalize.
Acam_mag(1,:) = 0;
%   fpow = bsxfun(@rdivide, fpow, max(fpow, [], 1));
Acam_mag = bsxfun(@rdivide, Acam_mag, max(Acam_mag,[],1));
% Acam_mag = bsxfun(@rdivide, Acam_mag, sum(Acam_mag,1));

[freq, ppg_pow] = plot_power_spectrum(handles.ppg, numel(handles.tppg)/(handles.tppg(end)-handles.tppg(1)));
ppg_mag = sqrt(ppg_pow);
% Remove DC and normalize.
ppg_mag(1) = 0;
% ppg_mag = ppg_mag/max(ppg_mag(f30bpm : f200bpm));
ppg_mag = ppg_mag/max(ppg_mag);

low_freq = str2double(handles.lpftext.String);
high_freq = str2double(handles.hpftext.String);
  
corr_clim = [];
switch lower(plot_type)
  case 'snr'
    success_metric = get_spectral_snr(Acam, handles.ppg, ...
      handles.fps, [low_freq high_freq]);
%     success_metric(success_metric<3)=0;
%     success_metric = 10.^success_metric;
% success_metric = exp(success_metric);
    title_str = 'SNR (dB)';
    corr_clim = [0 8];
%     corr_clim = [0 4];
    
  case 'invsnr'
    if exist('signals/carotid.mat', 'file') == 2
      load signals/carotid.mat  % loads variable Apix
      disp('Loading template from signals/carotid.mat')
    else
      Apix = handles.ppg(:);
      disp('Using PPG as template signal.')
    end
%   Apix_nophase = ifft(real(fft(Apix(:))));
    Apix = squeeze(Apix.ppgi);
    success_metric = corr(-log(Apix-min(Apix)+1), Acam);
    title_str = 'Correlation to inverse pulse';
    corr_clim = [-1 1];
    
  case 'corr'
%     PPGnophase = ifft(real(fft(handles.ppg)));
%     Acam_nophase = ifft(real(fft(Acam)));
%     success_metric = corr(PPGnophase',Acam_nophase);
%     success_metric = corr(circshift(handles.ppg',floor(-.07*60)), Acam);
    success_metric = corr(handles.ppg', Acam);
%     success_metric = success_metric.^2; warning('displaying corr^3')
    title_str = 'Pearson''s Correlation';
    corr_clim = [-1 1];
    
  case 'fmag'
    [~,hridx] = max(ppg_mag);
%     success_metric = Acam_mag(hridx,:).^2./max(Acam_mag([1:hridx-2,hridx+2:end],:),[],1).^2;
    magsort = sort(Acam_mag,1,'descend');
    success_metric = magsort(1,:).^2./magsort(2,:).^2;
    title_str = 'FMAG Ratio';
    corr_clim = [0 max(4, max(success_metric(:)))];

  case 'entr'
    success_metric = get_spectral_entropy(Acam_pow);
    [~,fmax_idx] = max(Acam_pow,[],1);
    entropy_mask = (fmax_idx>f200bpm);
    success_metric(entropy_mask) = 1;
%     fweight = 1-1./(1+3033*exp(-freq(fmax_idx)./0.4));
%     success_metric = (1-success_metric.*fweight);
      
    title_str = 'Normalized Entropy';
    
    if get(handles.cb_fusion, 'value') == 1
      plot_entropy_grid(success_metric, handles)
    end
    
    set(handles.cb_fusion, 'value', 0)
    
  case 'spat'
    success_metric = get_spatial_weight(Acam, handles.im, handles.block_size);
    title_str = 'Spatial Weight';

  case 'hrmag'
    [~,hr_idx] = max(ppg_mag);
%     success_metric = Acam_mag(hr_idx,:);
%     success_metric(success_metric<.8) = 0;

%     [~,hrhat_idx] = max(Acam_mag,[],1);
    [~,Acam_mag_sort] = sort(Acam_mag,1,'descend');
    hrhat_idx = Acam_mag_sort([1,2],:);
    
    df = handles.fps/npts;
    hr_error = abs(60*(hrhat_idx*df - hr_idx*df));
    hr_error = min(hr_error,[],1);
    % From: cftool([0 2 4 10],[1 .8 .5 0])
    success_metric = 2.015./(1.834+exp(0.62*hr_error-1.67));
    title_str = 'HR MAG';
    corr_clim = [.8 1];
end

if numel(success_metric) ~= size(Acam_pow,2)
  warning('Must have a success metric for each pixel.')
end

% Save for use by menu items.
handles.success_metric = success_metric;

% Init so that there is some overflow room at the end. We'll trim it after.
corr_map = zeros(size(handles.im,1)+handles.block_size(1), size(handles.im,2)+handles.block_size(2));
idx = 1;
for c = 1:handles.block_size(2):size(handles.im,2)
  for r = 1:handles.block_size(1):size(handles.im,1)
    corr_map(r:r+handles.block_size(1)-1, c:c+handles.block_size(2)-1) = success_metric(idx);
    idx = idx + 1;
  end
end
corr_map = corr_map(1:size(handles.im,1), 1:size(handles.im,2));
corr_map = bsxfun(@times, corr_map, handles.mask);

if strcmpi(plot_type, 'entr') && false
  enhance_pulse(handles.im, corr_map, handles.fps);
end

im_overlay = corr_map;
if get(handles.rb_demomode, 'value') == 1
%   sigma = 5;
%   blur_kernel = fspecial('gaussian', 3*sigma, sigma);
%   im_overlay = imfilter(corr_map, blur_kernel);
    im_overlay = bloodflowest(corr_map, handles.block_size(1), handles.vistype);
    
    num_iter = 50;
    delta_t = 1/7;
    kappa = 30;
    option = 2;
    im_overlay = anisodiff2D(corr_map,num_iter,delta_t,kappa,option);
end

% h_image = imshow(corr_map, []);
% cmap = cbrewer('seq', 'YlOrRd', 255);
cmap = jet;
if get(handles.cb_overlay, 'value')
%   back_image = im2double(handles.im(:,:,1));
%   h2 = imshow(repmat(back_image./max(back_image(:)), [1 1 3]));
%   hold on
%   h_image = imshow(im_overlay, []);
%   h_image = imshow(im_overlay, []);
%   hold off
%   corr_map_norm = corr_map./max(corr_map(:));
  switch lower(plot_type)
    case 'snr'
      fitobj = map_snr_to_vis(success_metric);
%         x = [0,1,2.5,3,4];
%         y = [0,.1,.5,.99,1];
%         fitobj = fit(x',y','1/(1+exp(a*x+b))');
        corr_map_norm = feval(fitobj, im_overlay);
        corr_map_norm = reshape(corr_map_norm, size(im_overlay));
%         corr_map_norm = 1./(1+exp(-2.24*im_overlay+4.47));
        cmap = cbrewer('seq', 'YlOrRd', 255);
        
    case 'entr'
      corr_map_norm = im_overlay;
%       corr_map_norm = reshape(corr_map_norm, size(im_overlay));
      
    case 'invsnr'
      corr_map_norm = im_overlay.^2;
%       set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))

    case 'corr'
%       corr_map_norm = abs(0.67*im_overlay.^3+0.33*im_overlay);
      corr_map_norm = (im_overlay*1.2).^2;
      corr_map_norm(corr_map_norm>1) = 1;
%       corr_map_norm = 0.67*im_overlay.^3+0.33*im_overlay;
%       set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm)
%       set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm)
%       cmap = flipud(cbrewer('div', 'Spectral', 255));

    case 'fmag'
      % Maps 1->0.1, 2->0.5, 4->1
      corr_map_norm = 1./(1+exp(-2.24*im_overlay+4.47));

    case 'spat'
      % Normalize 0->1 (!!make this not relative).
      corr_map_norm = mat2gray(im_overlay);

    case 'hrmag'
      corr_map_norm = im_overlay>.95;
  end
  
  back_image = im2double(handles.im(:,:,1));
  h2 = imshow(repmat(back_image./max(back_image(:)*1), [1 1 3]));
  hold on
  h_image = imshow(im_overlay, []);
  set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm.*handles.mask)
  hold off
else
  h_image = imshow(im_overlay, []);
  
%   if ~isempty(corr_clim)
%     set(gca, 'CLim', corr_clim);
%   end
end
if ~isempty(corr_clim)
  set(gca, 'CLim', corr_clim);
end
colormap(handles.axes1, cmap)
% % colormap(jet)
% % colormap(diverging_cmap(0:.01:1,[1 1 1],[1 0 0]))
colorbar
% title(title_str, 'fontsize', 14)
% set(gcf,'color','w')
  

handles.corr_map = corr_map;
guidata(gcf, handles);

set(h_image, 'ButtonDownFcn', @image_ButtonDownFcn)

function fitobj = map_snr_to_vis(snr_map)
% x = [0,1,2.5,3,4];
% y = [0,.1,.5,.99,1];
snr_sort = sort(snr_map(:),'descend');
snrtop1 = snr_sort(ceil(numel(snr_sort)*.01));
snrtop2 = snr_sort(ceil(numel(snr_sort)*.10)); 
x = [1,2,snrtop2,snrtop1];
% warning('CHANGED SNR FIT')
x = [1,1.5,3,4];
y = [.1,.5,.9,1];

fitobj = fit(x',y','1/(1+exp(a*x+b))');

% curfig = gcf;
% figure, plot(0:.1:6, feval(fitobj,0:.1:6))
% figure(curfig)


function gen_pulsing(frames_refl, frames_absorp, snr_map, block_width, filename)

handles = guidata(gcf);
handles.vistype = 'blur';
if nargin < 5
  filename = 'pulsing';
end

% snr_thresh = 4;
% frames_refl = frames_refl(:,:,1:50);
% frames_absorp = frames_absorp(:,:,1:50);

% Maps 1dB->0.1, 2dB->0.5, 4dB->1
snr_map_norm = 1./(1+exp(-2.24*snr_map+4.47));
% sigmoid_func = @(x,xdata) 1./(1+exp(x(1)*xdata+x(2)));
% max_snr = max(snr_map(:));
% x_snr = linspace(1, max_snr, 5);
% y_weight = [0.05 0.1 .5 .9 1];
% sigmoid_params = lsqcurvefit(sigmoid_func, [0,0], x_snr, y_weight);
% snr_map_norm = 1./(1+exp(sigmoid_params(1)*snr_map + sigmoid_params(2)));

% snr_map_norm = snr_map./max(snr_map(:));

% sigma = 5;
% blur_kernel = fspecial('gaussian', 3*sigma, sigma);
% snr_map_norm = imfilter(snr_map_norm, blur_kernel);
snr_map_norm = bloodflowest(snr_map_norm, handles.block_size(1), handles.vistype);

mask = true(size(frames_refl(:,:,1)));
if strcmpi(questdlg('Draw mask?','Pulsing Mask','Yes','No','No'), 'yes')
  figure;
  mask = roipoly(frames_refl(:,:,1));
  close;
end


%   im_overlay = bloodflowest(handles.corr_map, handles.block_size(1), handles.vistype);
%   switch lower(plot_type)
%   case 'snr'
%       % Maps 1dB->0.1, 2dB->0.5, 4dB->1
% %       snr_map_norm = 1./(1+exp(-2.24*snr_map+4.47));
% %       alpha_map = imresize(snr_map, 1/block_width, 'nearest');
%       fitobj = map_snr_to_vis(handles.success_metric);
%       alpha_map = feval(fitobj, im_overlay);
%       alpha_map = reshape(alpha_map, size(im_overlay));
% 
%       % sigmoid_func = @(x,xdata) 1./(1+exp(x(1)*xdata+x(2)));
%       % max_snr = max(snr_map(:));
%       % x_snr = linspace(1, max_snr, 5);
%       % y_weight = [0.05 0.1 .5 .9 1];
%       % sigmoid_params = lsqcurvefit(sigmoid_func, [0,0], x_snr, y_weight);
%       % snr_map_norm = 1./(1+exp(sigmoid_params(1)*snr_map + sigmoid_params(2)));
% 
%       % snr_map_norm = snr_map./max(snr_map(:));
%   case 'entr'
%       % Maps 0->1, 0.5->1, 0.75->0.5, 0.8->0.25
%       alpha_map = 1-1./(1+exp(-22.1*snr_map+16.6));
%   end
  im_overlay = bloodflowest(handles.corr_map, handles.block_size(1), handles.vistype);
warning('Saving pulsing video only works for corr right now.')
  alpha_map = im_overlay.^2;
  alpha_art = alpha_map;
  alpha_art(im_overlay<0) = 0;
  alpha_ven = alpha_map;
  alpha_ven(im_overlay>0) = 0;
  
  sigma = handles.block_size(1)/2;
  [H,W,T] = size(frames_refl);
  pulsing_vid = reshape(permute(frames_absorp,[2 1]),...
    H/handles.block_size(1),W/handles.block_size(2),T);
  pulsing_vid = imresize(pulsing_vid,handles.block_size(1));
%   pulsing_vid = imfilter(pulsing_vid, fspecial('gaussian',3*sigma,sigma));
  pulsing_vid = bloodflowest(pulsing_vid, handles.block_size(1), handles.vistype);
  pulsing_vid = bsxfun(@minus, pulsing_vid, min(pulsing_vid,[],3));
  pulsing_vid = bsxfun(@rdivide, pulsing_vid, max(pulsing_vid,[],3));

curfig = gcf;


  cmap = cbrewer('seq', 'OrRd', 256);
  % Start with orange to show diastolic flow (the rest will be transparent).
  cmap_art = cmap(100:end,:);
  cmap = cbrewer('seq', 'Blues', 256);
  cmap_ven = cmap(100:end,:);
  [H,W,T] = size(frames_refl);
  % % Make sure A is in the range from 1 to size(cm,1)
  % a = max(1,min(im2uint8(pulsing_vid)+1,size(cmap,1)));
  % 
  % % Extract r,g,b components
  % pulsing_frames = zeros([H,W,3,T]);
  % pulsing_frames(:,:,1,:) = reshape(cmap(a,1), [H,W,1,T]);
  % pulsing_frames(:,:,2,:) = reshape(cmap(a,2), [H,W,1,T]);
  % pulsing_frames(:,:,3,:) = reshape(cmap(a,3), [H,W,1,T]);

  warning('Not storing handles.videoframes in cache.')
%   if isempty(handles.videoframes)
    videoframes = zeros([H,W,3,T]);
    hwait = waitbar(0,'Processing video...');
    for t = 1:size(frames_refl,3)
      waitbar(t/T, hwait)
%       im1 = repmat(im2double(frames_refl(:,:,t)),[1 1 3]);
%       im2 = pulsing_vid(:,:,t);
%       im2star = ind2rgb(im2uint8(im2), cmap);
%       alpha = im2.*alpha_map.*mask;
% 
%       im3 = bsxfun(@times,(1-alpha),im1) + bsxfun(@times,alpha,im2star);
%       videoframes(:,:,:,t) = im3;

      im1 = repmat(im2double(frames_refl(:,:,t)),[1 1 3]);
      im2 = pulsing_vid(:,:,t);
      im2art = ind2rgb(im2uint8(im2), cmap_art);
      im2ven = ind2rgb(im2uint8(im2), cmap_ven);
      alpha1 = im2.*mat2gray(alpha_art).*mask;
      alpha2 = im2.*mat2gray(alpha_ven).*mask;
      im3 = bsxfun(@times, (1-(alpha1+alpha2)), im1) + ...
            bsxfun(@times, alpha1, im2art) + ...
            bsxfun(@times, alpha2, im2ven);
      videoframes(:,:,:,t) = im3;
    end
    
    waitbar(1, hwait, 'Saving video...')
    vout = VideoWriter(sprintf('C:/Users/ramelard/Desktop/%s.avi',filename),'Motion JPEG AVI');
    vout.FrameRate = 10;
    open(vout)
    writeVideo(vout, videoframes);
    close(vout)
    close(hwait);


% pulsing_vid = frames_absorp;
% % pulsing_vid(repmat(snr_map,[1 1 size(frames,3)]) < snr_thresh) = 0;
% % pulsing_vid = pulsing_vid.^2;
% pulsing_vid = bsxfun(@minus, pulsing_vid, min(pulsing_vid,[],3));
% pulsing_vid = bsxfun(@rdivide, pulsing_vid, max(pulsing_vid,[],3));
% % pulsing_vid = imfilter(pulsing_vid, fspecial('gaussian',3*sigma,sigma));
% pulsing_vid = bloodflowest(pulsing_vid, handles.block_size(1), handles.vistype);
% pulsing_vid = bsxfun(@times, pulsing_vid, snr_map_norm);
%     
% v = VideoWriter(sprintf('C:/Users/ramelard/Desktop/%s.avi',filename),'Motion JPEG AVI');
% v.FrameRate = 10;
% open(v)
% hfig = figure;
% set(hfig, 'Color', [0 0 0])
% % movegui(hwait, 'southwest')
% opengl('software')  % alphadata is weird without this
% cmap = cbrewer('seq', 'OrRd', 256);
% % Start with orange to show diastolic flow (the rest will be transparent).
% cmap = cmap(100:end,:);
% for t = 1:size(frames_absorp,3)
%   imshow(repmat(im2double(frames_refl(:,:,t)),[1 1 3]))
%   hold on
%   h = imshow(pulsing_vid(:,:,t).*mask, []);
% %   h = imshow(pulsing_vid(:,:,t).*mask.*snr_map_norm, []);
%   hold off
%   set(h, 'AlphaData', snr_map_norm./1.5.*mask)
%   colormap(cmap);
%   drawnow
%   vidframe = getframe(hfig);
% 
%   writeVideo(v, vidframe);
% end
% close(v)
% close(hfig)

figure(curfig)


function Xnew = enhance_pulse(frames, entropy_map, fps)
X = frames;
dXdt = cat(3, X(:,:,2:end), X(:,:,end)) - X;
% Gaussian-type curve, fit according to:
%   cftool([0, .6, .8, 1],[1, .9, .01, .01])
alpha = 1-1./(1+exp(-34*entropy_map+22.6));

Xnew = zeros(size(X));
Xnew(:,:,1) = X(:,:,1);
for i = 2:size(X,3)
  Xnew(:,:,i) = Xnew(:,:,i-1) - 3*alpha.*dXdt(:,:,i-1);
end

Xnew(Xnew<0) = 0;
Xnew(Xnew>1) = 1;
% Xnew2 = zeros(size(Xnew,1), size(Xnew,2), 3, size(Xnew,3));
% Xnew2(:,:,3,:) = X;
% Xnew2(:,:,2,:) = X;
% Xnew2(:,:,1,:) = Xnew;
curfig = gcf;
implay(Xnew, fps)
figure(curfig)


function plot_sigs(Apix, ppg, tppg, ax)

if nargin < 4
  figure;
  ax = gca;
end

% Apix = mat2gray(Apix);

axes(ax)
hold off
plot(tppg, (mat2gray(ppg)-.5)*range(Apix)+mean(Apix),'color',[0.3 0.3 0.3], 'linewidth', 0.5)
hold on

plot(tppg, Apix, 'r', 'linewidth', 2)
% ylim([0 1])
xlabel('time (s)')
ylabel('amplitude (unitless)')
set(gca, 'ytick', [])
title([])


function plot_psd(Apix, fps, ppg, tppg, ax)
  
if nargin < 5
  figure;
  ax = gca;
end
  
npts = numel(Apix);
f30bpm = ceil(30/60*npts/fps) + 1;
f200bpm = floor(200/60*npts/fps) + 1;


[freq, Apix_mag] = get_normalized_fmag(Apix, fps);
[freq, ppg_mag] = get_normalized_fmag(ppg, numel(tppg)/(tppg(end)-tppg(1)));

Apix_mag = Apix_mag./max(Apix_mag);
ppg_mag = ppg_mag./max(ppg_mag);

axes(ax)
hold off
plot(freq, ppg_mag.^2, '--b', 'linewidth', 2)
hold on
plot(freq, Apix_mag.^2, 'r', 'linewidth', 2)
% [ax,h1,h2] = plotyy(freq,Apix_mag.^2,freq,ppg_mag.^2);
% set(h2,'color','b','linewidth',2,'linestyle','--')
% set(h1,'color','r','linewidth',2)
% set(ax,{'ycolor'},{'r';'b'})

% set(ax(1),'ylim',[0 5])
% set(ax(2),'ylim',[0 5])
%   xlim(ax(1),[0 5])
%   xlim(ax(2),[0 5])
xlim([0 10])


% plot(freq, ppg_mag.^2, 'r')
% hold on
% plot(freq, Apix_mag.^2, 'b', 'linewidth',2)
% xlim([00/60 200/60])
xlabel('frequency (Hz)')
ylabel('power')
title('Normalized Power Spectral Density','fontsize',14)
grid on
legend('PPG','CHI')


function [freq, fmag] = get_normalized_fmag(sig, fps)
[freq, fpow] = plot_power_spectrum(sig, fps);
fmag = sqrt(fpow);
% Remove DC and normalize.
fmag(1,:) = 0;
fmag = bsxfun(@rdivide, fmag, sum(fmag,1));

% [freq, ppg_pow] = plot_power_spectrum(ppg, numel(tppg)/(tppg(end)-tppg(1)));
% ppg_mag = sqrt(ppg_pow);
% % Remove DC and normalize.
% ppg_mag(1) = 0;
% % ppg_mag = ppg_mag/max(ppg_mag(f30bpm : f200bpm));
% % ppg_mag = ppg_mag/max(ppg_mag);
% ppg_mag = ppg_mag/sum(ppg_mag);


% --- Outputs from this function are returned to the command line.
function varargout = gui_ppgi_analyzer_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on mouse press over axes background.
function image_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fig_handles = guidata(hObject);

% Display PPGI trace on axes2.
a = get(gca,'CurrentPoint');
x = a(1,1);
y = a(1,2);
disp(sprintf('Strength: %g', fig_handles.corr_map(floor(y), floor(x))))
% fig_handles = guidata(hObject);

fps = fig_handles.fps;
block_size = fig_handles.block_size;
tppg = fig_handles.tppg;
ppg = fig_handles.ppg;

topleft = [x y];
[imR,imC,~] = size(fig_handles.im);
ind = sub2ind([ceil(imR/block_size(1)) ceil(imC/block_size(2))], ...
  floor(topleft(2)/block_size(1)+1), floor(topleft(1)/block_size(2)+1));
disp(sprintf('Clicked index: %u',ind))
if get(fig_handles.cb_detrend, 'value') == 1
  Acam = fig_handles.Acam_detrend;
else
  Acam = fig_handles.Acam;
end
Apix = Acam(:,ind);
if ~isreal(Apix)
  warning('Apix is an imaginary number. Using only real component.')
  Apix = real(Apix);
end

plot_sigs(Apix, ppg, tppg, fig_handles.axes2);
plot_psd(Apix, fps, ppg, tppg, fig_handles.axes3);

fig_handles.Apix = Apix;
guidata(gcf, fig_handles);


% --- Executes when selected object is changed in plotpanel.
function plotpanel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in plotpanel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(eventdata.NewValue, 'Tag'));


function spat = get_spatial_weight(Acam, im, block_size)
  B = reshape(Acam', [ceil(size(im,1)/block_size(1)) ceil(size(im,2)/block_size(2)) size(im,3)]);
  
%   Bgrad = zeros(size(B));
%   for t = 1:size(B,3)
%     Bgrad(:,:,t) = imgradient(B(:,:,t));
%   end

%   imvar = blockproc(im, block_size, @(bs) shiftdim(var(reshape(bs.data, [size(bs.data,1)*size(bs.data,2) size(bs.data,3)])),-1));
%   varN = max(reshape(imvar, [size(imvar,1)*size(imvar,2) size(imvar,3)]),[],2);
%   varN = mean(reshape(imvar, [size(imvar,1)*size(imvar,2) size(imvar,3)]),2);

%   Bgrad = imgradient(im(:,:,1));
%   varN = blockproc(Bgrad, block_size, @(bs) var(bs.data(:)));
  
%   Bgrad = zeros(size(B));
%   for t = 1:size(B,3)
%     Bgrad(:,:,t) = imgradient(B(:,:,t));
%   end
%   Bmean_var = var(Bgrad, 0, 3);
%   spat = Bmean_var(:)';
  
% NOTE: THIS IS DETRENDED, SO UNITS AREN'T BOUNDED PROPERLY
% B = exp(-B)-1;
  Bmean = mean(B,3);
%   varN = colfilt(Bmean, [3 3], 'sliding', @(xvec) var(xvec));
  Bmean_grad = imgradient(Bmean);
  spat = Bmean_grad(:)';
  
  
%   Bmean = mean(im,3);
  Bmean = blockproc(im(:,:,1), block_size, @(block_struct) mean(block_struct.data(:)));
  Bmean_grad = imgradient(Bmean);
  spat = Bmean_grad(:)';
  
%   Bmean = mean(im,3);
%   Bmean_grad = imgradient(Bmean);
%   Bmean_grad = blockproc(Bmean_grad, [10 10], @(block_struct) mean(block_struct.data(:)));
%   spat = Bmean_grad(:)';
  
  

function [best_sig, w1map, w2map, w3map, Wmap] = plot_entropy_grid(Aentr, handles)  
  % <0.01 ~ very sharp drop; 0.1 ~ drops to 0; 100 ~ flat line
  warning('changed logspace -3 to -1.5')
  alpha = logspace(-1.5, 1, 6);
  a2 = logspace(-1.5, 1, 6);
  a3 = logspace(-1.5, 1, 6);
  % Arithmetic mean
  mean_fcn = @(W,S) sum(bsxfun(@times,S,W),2)./sum(W);
%   % Geometric mean
%   mean_fcn = @(W,S) exp((sum(bsxfun(@times,W,log(S)),2))/(sum(W)));
  
%   % SPIE 2016 values
%   alpha = 0.00518;
%   a2 = Inf;

  if get(handles.cb_detrend, 'value') == 1
    Acam = handles.Acam_detrend;
  else
    Acam = handles.Acam;
  end
  spat = get_spatial_weight(Acam, handles.im, handles.block_size);
  
  blocksize = handles.block_size;
  mask = handles.mask(1:blocksize(1):end, 1:blocksize(2):end);
  Acam_masked = Acam(:,mask(:));
  Aentr_masked = Aentr(mask(:));
  spat_masked = spat(mask(:));
  
  [~, Acam_pow] = plot_power_spectrum(Acam_masked, handles.fps);
  Acam_pow = bsxfun(@rdivide, Acam_pow, sum(Acam_pow,1));
  [ppg_freq, ppg_pow] = plot_power_spectrum(handles.ppg, numel(handles.tppg)/(handles.tppg(end)-handles.tppg(1)));
  ppg_pow(1) = 0;

  FAcam = fft(Acam_masked);
  
  % Harmonics
  [~,maxidx] = max(Acam_pow);
  valid = (maxidx < size(Acam_pow,1)/2 & maxidx > 2);
  cellnums = 1:size(Acam_pow,2);
%   maxidx(maxidx > size(Acam_pow,1)/2) = floor(size(Acam_pow,1)/2);
%   maxidx(maxidx == 1) = floor(size(Acam_pow,1)/2);
  % Mark peak and harmonic +/- 1 frequency bin
  Acam_pow2 = zeros(size(Acam_pow));
  for k = [-2 0 2]
    freqmask = sub2ind(size(Acam_pow), maxidx(valid)+k, cellnums(valid));
    Acam_pow2(freqmask) = Acam_pow(freqmask);
    % Harmonic
    freqmask = sub2ind(size(Acam_pow), (2*maxidx(valid)-1)+k, cellnums(valid));
    Acam_pow2(freqmask) = Acam_pow(freqmask);
  end
  harm = (1-sum(Acam_pow2,1));
  
  Acam_mag = sqrt(Acam_pow);
  Acam_mag(1,:) = 0;
  Acam_mag = bsxfun(@rdivide, Acam_mag, max(Acam_mag,[],1));
%   Acam_mag = bsxfun(@rdivide, Acam_mag, sum(Acam_mag,1));
  [~,peakidx] = max(Acam_pow);
  for i = 1:size(Acam_mag,2)
    % How powerful is the largest peak outside of the fundamental range?
    l1 = peakidx(i)-2;
    l2 = peakidx(i)+2;
    l3 = min((2*peakidx(i)-1)-2, size(Acam_mag,1));
    l4 = min((2*peakidx(i)-1)+2, size(Acam_mag,1));
    Afmag(i) = max(Acam_mag([1:l1, l2:end],i)).^2;
  end
%   Afmag = max(Acam_mag([1:peakidx-2,peakidx+2:end],:),[],1).^2;
  warning('temporary overwrite of Aentr_masked')
  Aentr_masked = Afmag;

  Pw_best = [];
  Pw_best_metric = -1;

  cols = numel(a2);
  rows = ceil(numel(alpha)*numel(a2)*numel(a3)/cols);
  plotidx = 0;
  f30bpm = find(ppg_freq>30/60, 1)-1;
  f200bpm = find(ppg_freq>200/60, 1)-1;
  ppgi_color = 'r';
  ppg_color = 'k';
  
  curfig = gcf;
  fig_best = figure;
  hwait = waitbar(0,'Grid search...');
  movegui(hwait,'northwest')
  Wstar_vals = {};
  for i = 1:numel(alpha)
  for j = 1:numel(a2)
  for k = 1:numel(a3)
    waitbar(plotidx/(numel(alpha)*numel(a2)*numel(a3)),hwait)
    plotidx = plotidx + 1;

    % Combine weights
    wthresh = 1E-4;
    w = exp(-Aentr_masked.^2/alpha(i));
    w(w<wthresh) = 0;
    w2 = exp(-spat_masked.^2/a2(j));
    w2(w2<wthresh) = 0;
    w3 = exp(-harm.^2/a3(k));
    w3(w3<wthresh) = 0;
    W = w.*w2.*w3;
    if sum(W) == 0
      continue;
    end
    
    % Account for 2 types of signal (arterial, venous), and "no signal"
    [Wtop,Widx] = sort(W,'descend');
    sigs = Acam_masked(:,Widx);
    sigs_sorted = sigs(:,Wtop>0);
    kernel1 = sigs_sorted(:,1);
%     kernel2 = -log(kernel1-min(kernel1)+1);
%     kernel3 = sigs_sorted(:,end);
%     idx = kmeans(sigs',3,'distance','correlation','emptyaction','drop',...
%       'start',[kernel1 kernel2 kernel3]');
% %     figure;
% %     for l = unique(idx)'
% %       subplot(1,3,l), plot(sigs(:,idx==l))
% %     end

    kernel_corr = corr(kernel1, Acam_masked);

%     Wstar = W;
    B = zeros(size(mask));
    B(mask) = W;
    % 2D minimum filter across 5x5 pixel neighborhood
    Bfilt = ordfilt2(B, 1, true(5));
    Wstar = Bfilt(mask)';
%     warning('Not distinguishing weights between arterial and venous pulsing.')
%     Wstar(idx~=1) = 0;
%     Wstar(kernel_corr < 0) = 0;
    if sum(Wstar) == 0
      continue;
    end
    
    Pw_sum = mean_fcn(Wstar, Acam_pow);
    Pw_sum(1) = 0;
    
    
    Wstar_vals{i,j,k} = Wstar;
    
    Fppgi = mean_fcn(Wstar, FAcam);
    Fppgi(1) = 0;

    Fspectrum = Pw_sum./sum(Pw_sum);
    [~,maxidx] = max(Pw_sum);
    maxmask = false(size(Pw_sum));
    % Mark peak and harmonic +/- 1 frequency bin
    maxmask([maxidx-1:maxidx+1, (2*maxidx-1)-1:(2*maxidx-1)+1]) = true;
    fmag = sum(Fspectrum(maxmask))/sum(Fspectrum(~maxmask));
    fmag2 = 1-get_spectral_entropy(sqrt(Pw_sum));
    
%     [~,maxidx] = max(Pw_sum);
%     fmag = sum(Pw_sum([maxidx-1:maxidx+1, (2*maxidx-1)-1:(2*maxidx-1)+1]),1);

    if (Pw_best_metric < fmag && (maxidx > f30bpm && maxidx < f200bpm)) || Pw_best_metric == -1
      disp(sprintf('Best: S=%0.2g, H=%0.2g', fmag, fmag2))
      Pw_best_metric = fmag;
      Pw_best = ifft(Fppgi);
      params = [alpha(i) a2(j) a3(k) 0; ...
                w(:)     w2(:) w3(:) Wstar(:)];
      
      [b,a] = butter(2, 3.5*2/(handles.fps/2), 'low');  % have to normalize cutoff freq
      sig = Pw_best;
      sig_filt = filtfilt(b,a,sig);
      figure(fig_best)
      subplot(3,2,1), imshow(imresize(B, [200 200]),[])
      subplot(3,2,2), imshow(imresize(Bfilt, [200 200]),[])
      subplot(3,1,2)
      plotyy(handles.tppg, handles.ppg, handles.tppg, sig_filt)
      subplot(3,1,3)
      Fsig_filt = sqrt(abs(fft(sig_filt)));
      plot(Fsig_filt(1:floor(numel(Fsig_filt)/2))./max(Fsig_filt))
      ylim([0 1])
      set(fig_best, 'position', 1000*[0.0763    0.2697    1.0833    0.6060])
      title(sprintf('%0.2g, %0.3g', fmag, fmag2))
%       pause;
    end
  end
  end
  end
  close(hwait)
  
%   % Plot signals
  f1 = figure;
  f2 = figure;
%   plotidx = 0;
%   for i = 1:numel(alpha)
%   for j = 1:numel(a2)
%   for k = 1:numel(a3)
%     plotidx = plotidx + 1;
%     Wstar = Wstar_vals{i,j,k};
%     if isempty(Wstar)
%       continue
%     end
%     Pw_sum = mean_fcn(Wstar, Acam_pow);
%     Pw_sum(1) = 0;
%     
% %     % Plot frequency power
% %     figure(f1);
% %     set(f1, 'visible', 'off')
% %     subplot(rows, cols, plotidx);
% %     [ax,hline1,hline2] = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
% %     set(hline1,'color',ppgi_color,'linewidth',2)
% %     set(hline2,'color',ppg_color,'linestyle','--','linewidth',2)
% %     set(ax(2),'ycolor','k')
% %     set(ax(1),'ycolor','k','ytick',[])
% %     set(ax(2),'ytick',[])
% %     xlim(ax(1), [30/60 200/60])
% %     xlim(ax(2), [30/60 200/60])
% %     try
% %       ylim(ax(1), [0 max(Pw_sum(f30bpm:f200bpm))])
% %       ylim(ax(2), [0 max(ppg_pow(f30bpm:f200bpm))])
% %     catch
% %     end
% %     xlabel('frequency (Hz)','fontsize',14)
% %     ylabel('spectral power','fontsize',14)
% % %       title(sprintf('\\alpha_1=%0.3g, \\alpha_2=%0.3g', alpha(i), a2(j)))
% 
% 
%     % Plot reconstructed temporal 
%     figure(f2);
%     set(f2, 'visible', 'off')
%     subplot(rows, cols, plotidx);
%     Fppgi = mean_fcn(Wstar, FAcam);
%     Fppgi(1) = 0;
%     [ax,hline1,hline2] = plotyy(handles.tppg, ifft(Fppgi), handles.tppg, handles.ppg);
%     set(hline1,'color',ppgi_color,'linewidth',2)
%     set(hline2,'color',ppg_color,'linestyle','--','linewidth',2)
%     set(ax(2),'ycolor','k')
%     set(ax(1),'ycolor','k','ytick',[])
%     set(ax(2),'ytick',[])
%     xlim(ax(1), [0 handles.tppg(end)])
%     xlim(ax(2), [0 handles.tppg(end)])
%     xlabel('time (s)','fontsize',14)
%     ylabel('amplitude','fontsize',14)
%     
% %     lolim = find(ppg_freq > 30/60)-1;
% %     hilim = find(ppg_freq > 200/60)+1;
% %     title(sprintf('%0.3g,%0.3g,%0.3g,%0.3g', ...
% %         1-get_spectral_entropy(Pw_sum), ...
% %         1-get_spectral_entropy(Pw_sum(lolim:hilim)), ...
% %         sum(Pw_sum(maxidx:end)), ...
% %         sum(Pw_sum(maxmask).^2)./sum(Pw_sum(~maxmask).^2)));
%   end
%   end
%   end
  
  figure,
%   Aentr_im = zeros(size(mask));
%   Aentr_im(mask) = Aentr_masked;
%   spat_im = zeros(size(mask));
%   spat_im(mask) = spat_masked;
%   harm_im = zeros(size(mask));
%   harm_im(mask) = harm;
  
  w1star = zeros(size(mask));
  w1star(mask) = params(2:end,1);
  w2star = zeros(size(mask));
  w2star(mask) = params(2:end,2);
  w3star = zeros(size(mask));
  w3star(mask) = params(2:end,3);
  Wstar = zeros(size(mask));
  Wstar(mask) = params(2:end,4);
  
%   w1star = exp(-Aentr_im.^2/params(1));
%   w2star = exp(-spat_im.^2/params(2));
%   w3star = exp(-harm_im.^2/params(3));
%   w1star(~mask) = 0;
%   w2star(~mask) = 0;
%   w3star(~mask) = 0;
  ax1 = subplot(2,2,1); imshow(handles.im(1:handles.block_size(1):end,1:handles.block_size(2):end,1),[])
  ax2 = subplot(2,3,4); imshow(w1star,[0 1]), title(params(1,1))
  ax3 = subplot(2,3,5); imshow(w2star,[0 1]), title(params(1,2))
  ax4 = subplot(2,3,6); imshow(w3star,[0 1]), title(params(1,3))
  ax5 = subplot(2,2,2); imshow(Wstar, []), title('W')
  linkaxes([ax1,ax2,ax3,ax4,ax5], 'xy')
  
  % Plot average at the end
  figure(f1);
  subplot(rows, cols, rows*cols)
  Pw_sum = mean(Acam_pow,2);
  Pw_sum(1) = 0;
  [ax,hline1,hline2] = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
  set(hline1,'color',ppgi_color,'linewidth',2)
  set(hline2,'color',ppg_color,'linestyle','--','linewidth',2)
  set(ax(2),'ycolor','k')
  set(ax(1),'ycolor','k','ytick',[])
  set(ax(2),'ytick',[])
  xlim(ax(1), [30/60 200/60]), xlim(ax(2), [30/60 200/60])
  xlabel('frequency (Hz)','fontsize',14)
  ylabel('spectral power','fontsize',14)
  title('')
  
  figure(f2);
  subplot(rows, cols, rows*cols)
  Fppgi = mean(FAcam,2);
  Fppgi(1) = 0;
  ppgi = ifft(Fppgi);
  [ax,hline1,hline2] = plotyy(handles.tppg, ifft(Fppgi), handles.tppg, handles.ppg);
  set(hline1,'color',ppgi_color,'linewidth',2)
  set(hline2,'color',ppg_color,'linestyle','--','linewidth',2)
  set(ax(2),'ycolor','k')
  set(ax(1),'ycolor','k','ytick',[])
  set(ax(2),'ytick',[])
  xlim(ax(1), [0 handles.tppg(end)])
  xlim(ax(2), [0 handles.tppg(end)])
  xlabel('time (s)','fontsize',14)
  ylabel('amplitude','fontsize',14)
  
%   Pw_sum = mean(Acam_pow,2);
%   Pw_sum(1) = 0;
%   ax = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
%   xlim(ax(1), [0.5 3.5]), xlim(ax(2), [0.5 3.5])
  title('')

  set(f1,'color','w')
  set(f2,'color','w')
  
  [B,A] = butter(2, 3.5*2/(handles.fps/2), 'low');  % have to normalize cutoff freq
  sig = Pw_best;
  sig_filt = filtfilt(B,A,sig);
  
  if nargout > 0
    best_sig = sig_filt;
  end
  if nargout > 1
    w1map = w1star;
    w2map = w2star;
    w3map = w3star;
    Wmap = Wstar;
  end
  
  % Heart rate variability
  % First get mean signal
  figure;
  subplot(4,1,1)
  
  [ax,hline1,hline2] = plotyy(handles.tppg, sig_filt, handles.tppg, handles.ppg);
  set(hline1,'color',ppgi_color,'linewidth',2)
  set(hline2,'color',ppg_color,'linestyle','--','linewidth',2)
  set(ax(2),'ycolor','k')
  set(ax(1),'ycolor','k','ytick',[])
  set(ax(2),'ytick',[])
  xlim(ax(1), [0 handles.tppg(end)])
  xlim(ax(2), [0 handles.tppg(end)])
  xlabel('time (s)','fontsize',14)
  ylabel('amplitude','fontsize',14)
  
  [t_out,ibi_out,freq_out,pow_out,peak_idx] = ...
    resp_rate_from_ppg(handles.tppg, -log(sig_filt+min(sig_filt)+1));
  subplot(4,1,2)
  plot(handles.tppg, sig_filt)
  hold on, plot(handles.tppg(peak_idx), sig_filt(peak_idx), 'or')
  xlim([0 handles.tppg(end)])
  title('Detected Peaks')
  subplot(4,1,3)
  plot(t_out, ibi_out)
  title('Interbeat Intervals')
  subplot(4,1,4)
  plot(freq_out, pow_out)
  brfreq = 14/60;
  yrange = ylim;
  hold on, line([brfreq brfreq], yrange)
  title('Breathing rate estimation')
  
  figure(curfig)
  
  
function [] = plot_snr_scatter(SNR, pred_var, dep_var_str)
if nargin < 3
  dep_var_str = '?';
end

% SNR = SNR(pred_var>0);
% pred_var = pred_var(pred_var>0);
pred_var(pred_var==0) = 1;

curfig = gcf;
figure;
% plot(SNR(SNR~=0), entropy(SNR~=0), 'x')
plot(pred_var, SNR, 'x')
xlabel(dep_var_str)
ylabel('SNR (dB)')
title(sprintf('%s vs SNR (r=%0.2g)', dep_var_str, corr(SNR(:), pred_var(:))),'fontsize',14)
set(gcf,'color','w')
figure(curfig)



% --- Executes on button press in cb_overlay.
function cb_overlay_Callback(hObject, eventdata, handles)
% hObject    handle to cb_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes on button press in cb_fusion.
function cb_fusion_Callback(hObject, eventdata, handles)
% hObject    handle to cb_fusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes on button press in cb_detrend.
function cb_detrend_Callback(hObject, eventdata, handles)
% hObject    handle to cb_detrend (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cb_detrend
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes on button press in pb_extplot.
function pb_extplot_Callback(hObject, eventdata, handles)
% hObject    handle to pb_extplot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
if get(hObject, 'value') == 1
  curfig=gcf;
  figure,
  imshow(handles.corr_map,[]),
  colorbar,
  figure(curfig)
  set(hObject, 'value', 0)
end


% --- Executes on slider movement.
function sl_overlay_Callback(hObject, eventdata, handles)
% hObject    handle to sl_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% if get(handles.cb_overlay, 'value')
  display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
% end

% --- Executes during object creation, after setting all properties.
function sl_overlay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pb_savesig.
function pb_savesig_Callback(hObject, eventdata, handles)
% hObject    handle to pb_savesig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Apix = struct();
Apix.ppgi = handles.Apix;
Apix.tppg = handles.tppg;
Apix.ppg = handles.ppg;

answer = inputdlg('Signal filename', 'Save Signal');
filename = answer{1};
if numel(filename) < 4 || ~strcmpi(filename(end-3:end), '.mat')
  filename = [filename, '.mat'];
end
save(['signals/' filename], 'Apix')


% --------------------------------------------------------------------
function menu_plots_Callback(hObject, eventdata, handles)
% hObject    handle to menu_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function menu_map_Callback(hObject, eventdata, handles)
% hObject    handle to menu_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function menu_scatter_Callback(hObject, eventdata, handles)
% hObject    handle to menu_scatter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
if get(handles.cb_detrend, 'value') == 1
  Acam = handles.Acam_detrend;
else
  Acam = handles.Acam;
end
[~, Acam_pow] = plot_power_spectrum(Acam, handles.fps);
Acam_mag = sqrt(Acam_pow);
% Remove DC and normalize.
Acam_mag(1,:) = 0;
Acam_mag = bsxfun(@rdivide, Acam_mag, max(Acam_mag,[],1));
[~, ppg_pow] = plot_power_spectrum(handles.ppg, numel(handles.tppg)/(handles.tppg(end)-handles.tppg(1)));
ppg_mag = sqrt(ppg_pow);
% Remove DC and normalize.
ppg_mag(1) = 0;
% ppg_mag = ppg_mag/max(ppg_mag(f30bpm : f200bpm));
ppg_mag = ppg_mag/max(ppg_mag);
% SNR = get_spectral_snr(Acam_mag, ppg_mag, handles.fps);
low_freq = str2double(handles.lpftext.String);
high_freq = str2double(handles.hpftext.String);
SNR = get_spectral_snr(Acam, handles.ppg, handles.fps, [low_freq high_freq]);
plot_snr_scatter(SNR, handles.success_metric)


% --------------------------------------------------------------------
function menu_fusion_Callback(hObject, eventdata, handles)
% hObject    handle to menu_fusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmpi(get(get(handles.plotpanel, 'selectedobject'), 'tag'), 'entr')
  plot_entropy_grid(handles.success_metric, handles)
end


% --------------------------------------------------------------------
function pulsing_video_Callback(hObject, eventdata, handles)
% hObject    handle to pulsing_video (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% if isempty(handles.Acam_filt)
%   if get(handles.cb_detrend, 'value') == 1
%     Acam = handles.Acam_detrend;
%   else
%     Acam = handles.Acam;
%   end
%   d = dialog('Position', [800 600 100 40]); 
%   uicontrol('Parent',d, 'Style','text', 'Position',[0 0 150 25],...
%             'String','Filtering frames...');
%   drawnow;
% 
%   Apix_ts = timeseries(Acam, handles.tppg-handles.tppg(1));
%   Apix_filt = idealfilter(Apix_ts, [handles.lpf handles.hpf], 'pass');
%   pulsing_vid = zeros(size(handles.im,1)+handles.block_size(1), ...
%                       size(handles.im,2)+handles.block_size(2), ...
%                       size(handles.im,3));
%   idx = 1;
%   for c = 1:handles.block_size(2):size(handles.im,2)
%     for r = 1:handles.block_size(1):size(handles.im,1)
%       pulsing_vid(r:r+handles.block_size(1), c:c+handles.block_size(2), :) = ...
%         repmat(shiftdim(Apix_filt.Data(:,idx)',-1), [handles.block_size+1 1]);
%       idx = idx + 1;
%     end
%   end
%   handles.Acam_filt = pulsing_vid(1:end-handles.block_size(1), 1:end-handles.block_size(2), :);
%   guidata(gcf, handles);
%   gcf
%   close(d)
% end
answer = inputdlg({'Video filename'}, 'Save Video', 1, {'pulsing'});
if ~isempty(answer)
  filename = answer{1};
  gen_pulsing(handles.im, handles.Acam, handles.corr_map, handles.block_size(1), filename)
end


% --- Executes on button press in rb_demomode.
function rb_demomode_Callback(hObject, eventdata, handles)
% hObject    handle to rb_demomode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rb_demomode
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes on button press in plot_psd_external.
function plot_psd_external_Callback(hObject, eventdata, handles)
% hObject    handle to plot_psd_external (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

a = get(gca,'CurrentPoint');
x = a(1,1);
y = a(1,2);
fig_handles = guidata(hObject);

Apix = fig_handles.Apix;
fps = fig_handles.fps;
block_size = fig_handles.block_size;
tppg = fig_handles.tppg;
ppg = fig_handles.ppg;

% topleft = 1 + [x y];
% rect = [floor(topleft(1)/block_size(1))*block_size(1) + 1 ...
%         floor(topleft(2)/block_size(2))*block_size(2) + 1 ...
%         block_size(1)-1 block_size(2)-1];
% [Rpix,~] = plot_pixel_vals(fig_handles.im,'fps',fps,'roi','rect',rect);
% Apix = -log(Rpix+1);
% 
% if get(fig_handles.cb_detrend, 'value') == 1
%   Apix = hrv_detrend(Apix);
% end
% 
% axes(fig_handles.axes2)
% hold off
% ppg_offset = (ppg-mean(ppg))/std(ppg)*std(Apix(:)) + mean(Apix(:));
% plot(tppg, ppg_offset,'--b', 'linewidth', 2)
% hold on
% plot(linspace(0,tppg(end),numel(Apix)), Apix, '-r', 'linewidth', 2)

plot_psd(Apix, fps, ppg, tppg);
legend('PPG','CHI')
set(gcf, 'position', [400 700 600 200])
set(gcf, 'color', 'w')

plot_sigs(Apix, ppg, tppg);
legend('PPG','CHI')
set(gcf, 'position', [450 750 600 200])
set(gcf, 'color', 'w')


function [sigs, sigidx] = get_strongest_waveforms(handles, npts, type)

if strcmpi(type, 'pos')
  sort_type = 'descend';
else
  sort_type = 'ascend';
end

block_size = handles.block_size(1);
% mask = handles.mask(1:block_size:end, 1:block_size:end);
mask = imresize(handles.mask, 1/block_size, 'nearest');

[~, strength_idx] = sort(handles.success_metric, sort_type);
strength_idx = strength_idx(mask(strength_idx));

if get(handles.cb_detrend, 'value') == 1
  sigs = handles.Acam_detrend(:,strength_idx(1:npts));
else
  sigs = handles.Acam(:,strength_idx(1:npts));
end
if nargout > 1
  sigidx = strength_idx(1:npts);
end


% --------------------------------------------------------------------
function menu_maxresponse_Callback(hObject, eventdata, handles)
% hObject    handle to menu_maxresponse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


answer = inputdlg('How many points? (Empty for all points in mask)');
answer = answer{1};
if isempty(answer)
  npts = numel(strength_idx);
else
  npts = str2double(answer);
end

[sigs,sigidx] = get_strongest_waveforms(handles, npts, 'pos');

block_size = handles.block_size(1);
[a,b] = ind2sub(ceil(size(handles.corr_map)./block_size), sigidx);
axes(handles.axes1)
hold on
plot(min(b*block_size, size(handles.corr_map,2)), min(a*block_size, size(handles.corr_map,1)),...
  'or','markersize',10,'linewidth',4)
hold off

fprintf(1,'[');
for i = 1:npts
  fprintf(1, '%g,', handles.success_metric(sigidx(i)))
end
fprintf(1,']\n');

if npts == 1
  disp('Saving carotid waveform')
  ppg = handles.ppg;
  tppg = handles.tppg;
  Apix = handles.Acam_detrend(:,sigidx(1));
%   Apix_ts = timeseries(Apix, tppg-tppg(1));
%   Apix = idealfilter(Apix_ts, [handles.lpf handles.hpf], 'pass');
  
  axes(handles.axes2)
  hold on
  Aplot = squeeze(Apix.data);
  Aplot_offset = (Aplot-mean(Aplot))/std(Aplot)*std(handles.Apix) + mean(handles.Apix);
  plot(tppg, Aplot_offset, '--', 'color', .5*[1 1 1])
  hold off

  save('signals/carotid.mat', 'Apix')
end


% --------------------------------------------------------------------
function menu_minresponse_Callback(hObject, eventdata, handles)
% hObject    handle to menu_minresponse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

answer = inputdlg('How many points? (Empty for all points in mask)');
answer = answer{1};
if isempty(answer)
  npts = numel(strength_idx);
else
  npts = str2double(answer);
end

[sigs,sigidx] = get_strongest_waveforms(handles, npts, 'neg');

block_size = handles.block_size(1);
[a,b] = ind2sub(ceil(size(handles.corr_map)./block_size), sigidx);
axes(handles.axes1)
hold on
plot(min(b*block_size, size(handles.corr_map,2)), min(a*block_size, size(handles.corr_map,1)),...
  'ob','markersize',10,'linewidth',4)
hold off

fprintf(1,'[');
for i = 1:npts
  fprintf(1, '%g,', handles.success_metric(sigidx(i)))
end
fprintf(1,']\n');





% --------------------------------------------------------------------
function menu_mask_Callback(hObject, eventdata, handles)
% hObject    handle to menu_mask (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename,pathname] = uigetfile('*.mat','Select image mask');
if filename == 0
  return;
end

% Loads variable mask_face
A = whos(matfile([pathname, filename]));
load([pathname, filename])
eval(sprintf('handles.mask = %s;', A.name));

guidata(hObject, handles)
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --------------------------------------------------------------------
function menu_invpulse_plots_Callback(hObject, eventdata, handles)
% hObject    handle to menu_invpulse_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

npts = 5;
[sigA,sigAidx] = get_strongest_waveforms(handles, npts, 'pos');
[sigV,sigVidx] = get_strongest_waveforms(handles, npts, 'neg');

figure, plot(sigA), pause(0.5)
figure, plot(sigV), pause(0.5)

disp('Saving top 5 positive and negative waveforms.')
ppg = handles.ppg;
tppg = handles.tppg;
Apix = sigA(:,1);
% Apix_ts = timeseries(Apix, tppg-tppg(1));
% Apix = idealfilter(Apix_ts, [handles.lpf handles.hpf], 'pass');

tppg_ppg = timeseries(ppg(:), tppg);

axes(handles.axes2)
hold on
Aplot = squeeze(Apix.data);
Aplot_offset = (Aplot-mean(Aplot))/std(Aplot)*std(handles.Apix) + mean(handles.Apix);
plot(tppg, Aplot_offset, '--', 'color', .5*[1 1 1])
hold off

pid = inputdlg('Participant ID?');
pid = str2double(pid{1});
save(sprintf('signals/P%0.2u_strongest10.mat',pid), 'Apix', 'tppg_ppg')

M = [];
M(:,1) = tppg;
M(:,2) = ppg;
M(:,3:7) = sigA;
M(:,8:12) = sigV;
results_dir = 'C:/Users/ramelard/Dropbox/papers/2016.1 Inverse Pulse [Nat]/results/';
filename = sprintf([results_dir, 'P%0.2u_strongest10.csv'],pid);
headers = {'Time (s)', 'Ground Truth', ...
  'Positive 1', 'Positive 2', 'Positive 3', 'Positive 4', 'Positive 5', ...
  'Negative 1', 'Negative 2', 'Negative 3', 'Negative 4', 'Negative 5'};
csvwrite_with_headers(filename, M, headers);


% --------------------------------------------------------------------
function menu_fusion_plots_Callback(hObject, eventdata, handles)
% hObject    handle to menu_fusion_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% For this participant, extract signals: tppg, ppg, fusion,
% mean_violajones, Kumar_goodnessoffit, and save as struct P for analysis.

% Get info from handles
Acam = handles.Acam_detrend;
blocksize = handles.block_size;
tppg = handles.tppg;

answer = inputdlg('Participant ID', 'Save Signals');
pid = str2double(answer{1});
results_dir = 'C:/Users/ramelard/Dropbox/papers/2016.2 Entropy Fusion [BOE]/results';

% Loads mask_face
load(sprintf('%s/facemasks/violajones/P%0.2u_mask.mat',results_dir,pid))

% Save ground truth PPG
Pstruct = [];
Pstruct.im = handles.im(1:blocksize(1):end,1:blocksize(2):end,1);
Pstruct.tppg = handles.tppg(:);
Pstruct.ppg = mat2gray(handles.ppg(:));

% Save mean signal using Viola Jones
blocksize = handles.block_size;
mask_face = mask_face(1:blocksize(1):end, 1:blocksize(2):end);
Acam_masked = Acam(:,mask_face(:));
mean_violajones = mean(Acam_masked,2);
[b,a] = butter(2, 3.5*2/(handles.fps/2), 'low');
Pstruct.meanviolajones = mat2gray(filtfilt(b,a,mean_violajones));

% Save Kumar goodness of fit signal (with our blockproc)
Ath = 8/255; % from their paper, expressed as double
% Bandpass each signal 0.5-5Hz
% Kumar works on reflectance, not absorbance, and no detrending.
Rcam = exp(-handles.Acam)-1;
Rcam_ts = timeseries(Rcam, tppg);
warning('use denoise function')
Rcam_ts = idealfilter(Rcam_ts, [handles.lpf handles.hpf], 'pass');
Rcam_kumar = Rcam_ts.Data;
% Estimate PR by standard average
mask_thresh = (max(Rcam_kumar)-min(Rcam_kumar)) < Ath;
[~,Yi] = plot_power_spectrum(Rcam_kumar, handles.fps);
[~,PRidx] = max(abs(fft(mean(Rcam_kumar(:,mask_face(:) & mask_thresh(:)),2))));
[~,PRidx_true] = max(abs(fft(handles.ppg-mean(handles.ppg))));
PRidx = PRidx_true;  % do this to bypass facial segmentation
if PRidx ~= PRidx_true
  warning('Kumar PRidx is not right')
end
% Calculate Gi(PR) for each region
b = 1;
numerator = sum(Yi(PRidx-b:PRidx+b,:));
denominator = sum(Yi) - sum(Yi(PRidx-b:PRidx+b,:));
G = numerator./denominator;
% Reject ymax-ymin < Ath
G(~mask_thresh) = 0;
% Extracted signal = weighted average
Pstruct.kumar = Rcam_kumar*G';
% Express as absorbance (even though they don't)
Pstruct.kumar = mat2gray(-log(Pstruct.kumar-min(Pstruct.kumar)+1));

% Save fusion signal
npts = size(Acam,1);
f200bpm = floor(200/60*npts/handles.fps) + 1;
[~, Acam_pow] = plot_power_spectrum(Acam, handles.fps);
Acam_pow(1,:) = 0;
Aentr = get_spectral_entropy(Acam_pow);
[~,fmax_idx] = max(Acam_pow,[],1);
entropy_mask = (fmax_idx>f200bpm);
Aentr(entropy_mask) = 1;
[Pstruct.fusion, Pstruct.w1map, Pstruct.w2map, Pstruct.w3map, Pstruct.Wmap] = ...
  plot_entropy_grid(Aentr,handles);
Pstruct.fusion = mat2gray(Pstruct.fusion);

figure;
hold on;
plot(Pstruct.tppg, Pstruct.ppg,'--k')
plot(Pstruct.tppg, Pstruct.fusion,'r','linewidth',2)
plot(Pstruct.tppg, Pstruct.meanviolajones,'b','linewidth',2)
plot(Pstruct.tppg, Pstruct.kumar,'g','linewidth',2)
legend('ppg','fusion','mean','kumar')
set(gcf,'position',[ 53         790        1187         188])

save(sprintf('%s/P%0.2u.mat',results_dir,pid), 'Pstruct')


% --------------------------------------------------------------------
function menu_hotspots_plots_Callback(hObject, eventdata, handles)
% hObject    handle to menu_hotspots_plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
answer = inputdlg('Participant ID', 'Save Signals');
pid = str2double(answer{1});
results_dir = 'C:/Users/ramelard/Dropbox/papers/2016.3 PPGI Hotspots [JBO]/results';

display_success_map(handles, 'snr')
hnew = guidata(hObject);
corr_map = hnew.corr_map;
save(sprintf('%s/P%0.2u_SNR.mat',results_dir,pid), 'corr_map')

display_success_map(handles, 'entr')
hnew = guidata(hObject);
corr_map = hnew.corr_map;
save(sprintf('%s/P%0.2u_ENTR.mat',results_dir,pid), 'corr_map')

display_success_map(handles, 'corr')
hnew = guidata(hObject);
corr_map = hnew.corr_map;
save(sprintf('%s/P%0.2u_CORR.mat',results_dir,pid), 'corr_map')

display_success_map(handles, 'hrmag')
hnew = guidata(hObject);
corr_map = hnew.corr_map;
save(sprintf('%s/P%0.2u_HRMAG.mat',results_dir,pid), 'corr_map')

display_success_map(handles, 'fmag')
hnew = guidata(hObject);
corr_map = hnew.corr_map;
save(sprintf('%s/P%0.2u_FMAG.mat',results_dir,pid), 'corr_map')



function lpftext_Callback(hObject, eventdata, handles)
% hObject    handle to lpftext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lpftext as text
%        str2double(get(hObject,'String')) returns contents of lpftext as a double
handles.Acam = handles.denoisefun(handles.Acam0, handles);
handles.Acam_detrend = hrv_detrend(handles.Acam);
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


function hpftext_Callback(hObject, eventdata, handles)
% hObject    handle to hpftext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hpftext as text
%        str2double(get(hObject,'String')) returns contents of hpftext as a double
handles.Acam = handles.denoisefun(handles.Acam0, handles);
handles.Acam_detrend = hrv_detrend(handles.Acam);
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes on button press in button_motion.
function button_motion_Callback(hObject, eventdata, handles)
% hObject    handle to button_motion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

frames_corrected = motion_correction(handles.im);
[~, Acam] = process_head(frames_corrected, handles.fps, handles.tppg, handles.ppg, handles.block_size, handles.mask, false);

handles.im = frames_corrected;
handles.Acam0 = Acam;
handles.Acam_detrend = [];
handles.Acam = [];  % don't know bandpass yet
handles.Apix = [];  % no pixel clicked yet
handles.Acam_filt = [];  % created filtered signal when necessary

handles.Acam = handles.denoisefun(handles.Acam0, handles);
handles.Acam_detrend = hrv_detrend(handles.Acam);

% Update handles structure
guidata(hObject, handles);

% Default to displaying SNR
display_success_map(handles, 'SNR');

guidata(hObject, handles);


% --- Executes on button press in button_play.
function button_play_Callback(hObject, eventdata, handles)
% hObject    handle to button_play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

frame1 = handles.im(:,:,1);
implay(handles.im./max(frame1(:)),min(handles.fps,100))


% --- Executes on selection change in pmenu_denoisetype.
function pmenu_denoisetype_Callback(hObject, eventdata, handles)
% hObject    handle to pmenu_denoisetype (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns pmenu_denoisetype contents as cell array
%        contents{get(hObject,'Value')} returns selected item from pmenu_denoisetype
str = get(hObject, 'String');
val = get(hObject,'Value');
% Set current data to the selected data set.
switch str{val}
case 'Ideal' % User selects peaks.
   handles.denoisefun = @denoise_ideal;
   handles.lpftext.String = '0.1';
   handles.hpftext.String = 'Inf';
   handles.lpftext.Enable = 'on';
   handles.hpftext.Enable = 'on';
case 'Kalman' % User selects membrane.
   handles.denoisefun = @denoise_kalman;
   handles.sigmam = 1.4e-3;
   handles.sigmap = 2.3e-5;
   handles.lpftext.Enable = 'off';
   handles.hpftext.Enable = 'off';
end
% Save the handles structure.
guidata(hObject,handles)

handles.Acam = handles.denoisefun(handles.Acam0, handles);
fprintf(1,'Detrending...')
handles.Acam_detrend = hrv_detrend(handles.Acam);
fprintf(1,'Done.\n')
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));


% --- Executes during object creation, after setting all properties.
function pmenu_denoisetype_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pmenu_denoisetype (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function xhat = denoise_ideal(x, handles)
low_freq = str2double(handles.lpftext.String);
high_freq = str2double(handles.hpftext.String);

x_ts = timeseries(x, handles.tppg);
x_filt = idealfilter(x_ts, [low_freq high_freq], 'pass');
xhat = x_filt.Data;

function xhat = denoise_kalman(x, handles)
% Each column is a signal
R = exp(-x)-1;
fprintf(1,'Denoising...')
sigmap = handles.sigmap;
sigmam = handles.sigmam;
xhat = zeros(size(x));
tic
parfor i = 1:size(x,2)
  xhat(:,i) = kalmanfilt_mex(R(:,i)', sigmap, sigmam);
end
toc
xhat = -log(xhat+1);
