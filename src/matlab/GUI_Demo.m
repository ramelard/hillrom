function varargout = GUI_Demo(varargin)
  % GUI_DEMO MATLAB code for GUI_DEMO.fig
  %      GUI_DEMO, by itself, creates a new GUI_Demo or raises the existing
  %      singleton*.
  %
  %      H = GUI_Demo returns the handle to a new GUI_Demo or the handle to
  %      the existing singleton*.
  %
  %      GUI_Demo('CALLBACK',hObject,eventData,handles,...) calls the local
  %      function named CALLBACK in GUI_Demo.M with the given input arguments.
  %
  %      GUI_Demo('Property','Value',...) creates a new GUI_Demo or raises the
  %      existing singleton*.  Starting from the left, property value pairs are
  %      applied to the GUI before GUI_Demo_OpeningFcn gets called.  An
  %      unrecognized property name or invalid value makes property application
  %      stop.  All inputs are passed to GUI_Demo_OpeningFcn via varargin.
  %
  %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
  %      instance to run (singleton)".
  %
  % See also: GUIDE, GUIDATA, GUIHANDLES

  % Edit the above text to modify the response to help GUI_Demo

  % Last Modified by GUIDE v2.5 27-Jan-2016 15:22:11

  % Begin initialization code - DO NOT EDIT
  gui_Singleton = 0;
  gui_State = struct('gui_Name',       mfilename, ...
                     'gui_Singleton',  gui_Singleton, ...
                     'gui_OpeningFcn', @GUI_Demo_OpeningFcn, ...
                     'gui_OutputFcn',  @GUI_Demo_OutputFcn, ...
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
end


% --- Executes just before GUI_Demo is made visible.
function GUI_Demo_OpeningFcn(hObject, eventdata, handles, varargin)
  % This function has no output args, see OutputFcn.
  % hObject    handle to figure
  % eventdata  reserved - to be defined in a future version of MATLAB
  % handles    structure with handles and user data (see GUIDATA)
  % varargin   command line arguments to GUI_Demo (see VARARGIN)

  % Choose default command line output for GUI_Demo
  handles.output = hObject;

  handles.im = varargin{1};
  handles.block_size = varargin{2};
  handles.Acam = varargin{3};
  handles.Apix = [];  % no pixel clicked yet
  handles.Acam_filt = [];  % created filtered signal when necessary
  handles.fps = varargin{4};
  handles.ppg = varargin{5};
  handles.tppg = varargin{6};
  handles.mask = varargin{7};
  handles.corr_map = [];
  handles.timer = timer;
  handles.videoframes = [];  % pulsing frames
  handles.vistype = 'blur';

  % Update handles structure
  guidata(hObject, handles);

  % Default to displaying SNR
  display_success_map(handles, 'SNR');

  % UIWAIT makes GUI_Demo wait for user response (see UIRESUME)
  % uiwait(handles.figure1);
end


function ztrue = detrend_sig(zhat)
  ztrue = hrv_detrend(zhat);
%   ztrue = zhat;
end


function display_success_map(handles, plot_type)
  axes(handles.axes1)

  Acam = handles.Acam;
  % if get(handles.cb_detrend, 'value') == 1
    Acam = detrend_sig(Acam);
  % end

  npts = size(Acam,1);
  f30bpm = ceil(30/60*npts/handles.fps) + 1;
  f200bpm = floor(200/60*npts/handles.fps) + 1;
  [freq, Acam_pow] = plot_power_spectrum(Acam, handles.fps);
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

  corr_clim = [];
  switch lower(plot_type)
    case 'snr'
      success_metric = get_snr(Acam_mag, handles.fps, ppg_mag);
      warning('Circshift for SNR');
      rho = corr(circshift(handles.ppg',-0), Acam);
      success_metric(rho < .1) = 0;
      title_str = 'SNR (dB)';
      corr_clim = [0 4];

    case 'invsnr'
      if exist('signals/carotid.mat', 'file') == 2
        load signals/carotid.mat  % loads variable Apix
        disp('Loading template from signals/carotid.mat')
      else
        Apix = handles.ppg(:);
        disp('Using PPG as template signal.')
      end
  %   Apix_nophase = ifft(real(fft(Apix(:))));
  %     success_metric = corr(-log(Apix-min(Apix)+1), Acam);
      success_metric = abs(corr(handles.ppg', Acam));
      title_str = 'Correlation to inverse pulse';
      corr_clim = [-1 1];

    case 'corr'
  %     PPGnophase = ifft(real(fft(handles.ppg)));
  %     Acam_nophase = ifft(real(fft(Acam)));
  %     success_metric = corr(PPGnophase',Acam_nophase);
      success_metric = corr(handles.ppg', Acam);
      title_str = 'Pearson''s Correlation';
      corr_clim = [-1 1];

    case 'fmag'
      [~,hridx] = max(ppg_mag);
      success_metric = Acam_mag(hridx,:).^2./max(Acam_mag([1:hridx-2,hridx+2:end],:),[],1).^2;
      title_str = 'FMAG Ratio';
      corr_clim = [0 10];

    case 'entr'
      success_metric = get_entropy(Acam_pow);
      [~,fmax_idx] = max(Acam_pow,[],1);
      entropy_mask = (fmax_idx>f200bpm);
      success_metric(entropy_mask) = 1;
  %     fweight = 1-1./(1+3033*exp(-freq(fmax_idx)./0.4));
  %     success_metric = (1-success_metric.*fweight);

      title_str = 'Normalized Entropy';

  %     if get(handles.cb_fusion, 'value') == 1
  %       plot_entropy_grid(success_metric, handles)
  %     end

  %     set(handles.cb_fusion, 'value', 0)

    case 'spat'
      success_metric = get_spatial_weight(Acam, handles.im, handles.block_size);
      title_str = 'Spatial Weight';

    case 'hrmag'
      [~,hr_freq] = max(ppg_mag);
      success_metric = Acam_mag(hr_freq,:);
      title_str = 'HR MAG';
  end

  if numel(success_metric) ~= size(Acam_pow,2)
    warning('Must have a success metric for each pixel.')
  end

  % Save for use by menu items.
  handles.success_metric = success_metric;

  corr_map = zeros(size(handles.im,1)+handles.block_size(1), size(handles.im,2)+handles.block_size(2));
  idx = 1;
  for c = 1:handles.block_size(2):size(handles.im,2)
    for r = 1:handles.block_size(1):size(handles.im,1)
      corr_map(r:r+handles.block_size(1), c:c+handles.block_size(2)) = success_metric(idx);
      idx = idx + 1;
    end
  end
  corr_map = corr_map(1:end-handles.block_size(1), 1:end-handles.block_size(2));
  corr_map = bsxfun(@times, corr_map, handles.mask);

  if strcmpi(plot_type, 'entr') && false
    enhance_pulse(handles.im, corr_map, handles.fps);
  end

%   im_overlay = corr_map;
  % if get(handles.rb_demomode, 'value') == 1
  %   sigma = floor(handles.block_size(1)/2);
%     sigma = 5;
%     blur_kernel = fspecial('gaussian', 3*sigma, sigma);
%     im_overlay = imfilter(corr_map, blur_kernel);
  % end
  
  im_overlay = bloodflowest(corr_map,handles.block_size(1),handles.vistype);

  % h_image = imshow(corr_map, []);
  % cmap = cbrewer('seq', 'YlOrRd', 255);
  cmap = jet;
  % if get(handles.cb_overlay, 'value')
  %   h_image = imshow(im_overlay, []);
  %   corr_map_norm = corr_map./max(corr_map(:));
    switch lower(plot_type)
      case 'snr'
        % Maps 1dB->0.1, 2dB->0.5, 4dB->1
        fitobj = map_snr_to_vis(success_metric);
%         x = [0,1,2.5,3,4];
%         y = [0,.1,.5,.99,1];
%         fitobj = fit(x',y','1/(1+exp(a*x+b))');
        corr_map_norm = feval(fitobj, im_overlay);
        corr_map_norm = reshape(corr_map_norm, size(im_overlay));
%         corr_map_norm = 1./(1+exp(-2.24*im_overlay+4.47));
        cmap = cbrewer('seq', 'YlOrRd', 255);
        
%         curfig = gcf;
%         figure, subplot(2,1,1), hist(success_metric(:));
%         subplot(2,1,2), plot(0:.1:4, feval(fitobj, 0:.1:4))
%         figure(curfig)

      case 'invsnr'
        set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))

      case 'corr'
        corr_map_norm = 0.67*im_overlay.^3+0.33*im_overlay;
%         set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))
  %       set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm)
  %       cmap = flipud(cbrewer('div', 'Spectral', 255));

      case 'fmag'
        % Maps 1->0.1, 2->0.5, 4->1
        corr_map_norm = 1./(1+exp(-2.24*im_overlay+4.47));
%         set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm)

      case 'entr'
  %       set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))
%         set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*(1-im_overlay))
        cmap = flipud(cbrewer('seq', 'YlOrRd', 255));

      case 'spat'
        set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))

      case 'hrmag'
        set(h_image, 'AlphaData', get(handles.sl_overlay, 'value'))
    end
  % else
  %   h_image = imshow(im_overlay, []);
  %   
  %   if ~isempty(corr_clim)
  %     set(gca, 'CLim', corr_clim);
  %   end
  % end
  back_image = handles.im(:,:,1);
  h2 = imshow(repmat(back_image./max(back_image(:)), [1 1 3]));
  hold on
  h_image = imshow(im_overlay, []);
  set(h_image, 'AlphaData', get(handles.sl_overlay, 'value').*corr_map_norm.*handles.mask)
  hold off
  colormap(cmap)
  % colormap(jet)
  % colormap(diverging_cmap(0:.01:1,[1 1 1],[1 0 0]))
  % colorbar
  title('Coded Hemodynamic Imaging (CHI)', 'fontsize', 14)
  % set(gcf,'color','w')

  handles.corr_map = corr_map;
  guidata(gcf, handles);

  set(h_image, 'ButtonDownFcn', @image_ButtonDownFcn)
end


function make_timer(videoframes)
  handles = guidata(gcf);
  t = 1;
  fig = gcf;
  ax = handles.axes1;
%   fig = figure;
%   ax = gca;
  fps = 10;
  handles.timer.Period = 1/fps;
  handles.timer.TasksToExecute = size(videoframes,4);
  handles.timer.ExecutionMode = 'fixedRate';
  handles.timer.StartFcn = {@timerinit, fig, ax};
  handles.timer.TimerFcn = @timerfcn;%@(myTimerObj, thisEvent) ...
  %   eval('handles=guidata(fig); axes(handles.axes1); imshow(squeeze(videoframes(:,:,:,t))); t=t+1;');
  handles.timer.StopFcn = @(myTimerObj, thisEvent) ...
    display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
  start(handles.timer)

    function timerinit(tobj, evt, tfig, tax)
       tobj.UserData = [tfig, tax]; 
    end
  
    function timerfcn(tobj, evt)
%       handles=guidata(fig);
      garray = tobj.UserData;
      figure(garray(1))
      axes(garray(2));
      pause(.01)
      imshow(squeeze(videoframes(:,:,:,t)));
      drawnow;
      t = t + 1;
%       tobj.UserData = [garray(1) garray(2) evt.Data.time(end)];
    end
end


function fitobj = map_snr_to_vis(snr_map)
% x = [0,1,2.5,3,4];
% y = [0,.1,.5,.99,1];
snr_sort = sort(snr_map(:),'descend');
snrtop1 = snr_sort(round(numel(snr_sort)*.01));
snrtop2 = snr_sort(round(numel(snr_sort)*.10)); 
x = [1,2,snrtop2,snrtop1];
x = [1,2,3,4];
y = [.1,.5,.9,1];

fitobj = fit(x',y','1/(1+exp(a*x+b))');

% curfig = gcf;
% figure, plot(0:.1:6, feval(fitobj,0:.1:6))
% figure(curfig)
end


function gen_pulsing(plot_type, frames_refl, frames_absorp, snr_map, block_width)
  if nargin < 5
    filename = 'pulsing';
  end
  
  guifig = gcf;
  handles = guidata(guifig);

  % snr_thresh = 4;
  % frames_refl = frames_refl(:,:,1:50);
  % frames_absorp = frames_absorp(:,:,1:50);
  
%   sigma = 5;
%   blur_kernel = fspecial('gaussian', 3*sigma, sigma);
%   im_overlay = imfilter(handles.corr_map, blur_kernel);
  im_overlay = bloodflowest(handles.corr_map, handles.block_size(1), handles.vistype);

  switch lower(plot_type)
  case 'snr'
      % Maps 1dB->0.1, 2dB->0.5, 4dB->1
%       snr_map_norm = 1./(1+exp(-2.24*snr_map+4.47));
%       alpha_map = imresize(snr_map, 1/block_width, 'nearest');
      fitobj = map_snr_to_vis(handles.success_metric);
      alpha_map = feval(fitobj, im_overlay);
      alpha_map = reshape(alpha_map, size(im_overlay));

      % sigmoid_func = @(x,xdata) 1./(1+exp(x(1)*xdata+x(2)));
      % max_snr = max(snr_map(:));
      % x_snr = linspace(1, max_snr, 5);
      % y_weight = [0.05 0.1 .5 .9 1];
      % sigmoid_params = lsqcurvefit(sigmoid_func, [0,0], x_snr, y_weight);
      % snr_map_norm = 1./(1+exp(sigmoid_params(1)*snr_map + sigmoid_params(2)));

      % snr_map_norm = snr_map./max(snr_map(:));
  case 'entr'
      % Maps 0->1, 0.5->1, 0.75->0.5, 0.8->0.25
      alpha_map = 1-1./(1+exp(-22.1*snr_map+16.6));
  end
  

  mask = handles.mask;
%   mask = true(size(frames_refl(:,:,1)));
%   if strcmpi(questdlg('Draw mask?','Pulsing Mask','Yes','No','No'), 'yes')
%     figure;
%     mask = roipoly(frames_refl(:,:,1));
%     close;
%   end

  sigma = handles.block_size(1)/2;
  pulsing_vid = frames_absorp;
  % pulsing_vid(repmat(snr_map,[1 1 size(frames,3)]) < snr_thresh) = 0;
  % pulsing_vid = pulsing_vid.^2;
  pulsing_vid = imfilter(pulsing_vid, fspecial('gaussian',3*sigma,sigma));
  pulsing_vid = bsxfun(@minus, pulsing_vid, min(pulsing_vid,[],3));
  pulsing_vid = bsxfun(@rdivide, pulsing_vid, max(pulsing_vid,[],3));
%   for t = 1:size(pulsing_vid,3)
%     pulsing_vid(:,:,t) = bloodflowest(pulsing_vid(:,:,t), handles.block_size(1), 'iter');
%   end
%   pulsing_vid = bsxfun(@times, pulsing_vid, alpha_map);

  % curfig = gcf;

  % v = VideoWriter(sprintf('C:/Users/ramelard/Desktop/%s.avi',filename),'Motion JPEG AVI');
  % v.FrameRate = 10;
  % open(v)
  % hfig = figure;
  % set(hfig, 'Color', [0 0 0])
  % % movegui(hwait, 'southwest')
  % opengl('software')  % alphadata is weird without this
  cmap = cbrewer('seq', 'OrRd', 256);
  % Start with orange to show diastolic flow (the rest will be transparent).
  cmap = cmap(100:end,:);
  [H,W,T] = size(frames_refl);

  tic
  % % Make sure A is in the range from 1 to size(cm,1)
  % a = max(1,min(im2uint8(pulsing_vid)+1,size(cmap,1)));
  % 
  % % Extract r,g,b components
  % pulsing_frames = zeros([H,W,3,T]);
  % pulsing_frames(:,:,1,:) = reshape(cmap(a,1), [H,W,1,T]);
  % pulsing_frames(:,:,2,:) = reshape(cmap(a,2), [H,W,1,T]);
  % pulsing_frames(:,:,3,:) = reshape(cmap(a,3), [H,W,1,T]);

  toc
  warning('Not storing handles.videoframes in cache.')
%   if isempty(handles.videoframes)
    tic
    videoframes = zeros([H,W,3,T]);
    hwait = waitbar(0,'Processing video...');
    for t = 1:size(frames_refl,3)
      waitbar(t/T, hwait)
      im1 = repmat(im2double(frames_refl(:,:,t)),[1 1 3]);
      im2 = pulsing_vid(:,:,t);
%       im2 = im_overlay;
      im2star = ind2rgb(im2uint8(im2), cmap);
    %   im2star = squeeze(pulsing_frames(:,:,:,t));

%       alpha = snr_map_norm./1.5.*mask;
      alpha = im2.*alpha_map.*mask;
%       alpha = alpha.*(im2>.9);

      im3 = bsxfun(@times,(1-alpha),im1) + bsxfun(@times,alpha,im2star);
      videoframes(:,:,:,t) = im3;
    end
    toc
    close(hwait)
    handles.videoframes = videoframes;
    guidata(guifig, handles)
%   end
  % implay(videoframes,10)

  axes(handles.axes1);
  hold off;
  axes(handles.axes2)
  hold off;
  
  Apix_ts = timeseries(handles.Apix', handles.tppg-handles.tppg(1));
  Apix_filt_ts = idealfilter(Apix_ts, [.5 3.5], 'pass');
  Apix_filt = Apix_filt_ts.Data - min(Apix_filt_ts.Data);
  Apix_filt = squeeze(Apix_filt./max(Apix_filt)*.9);

%   make_timer(handles.videoframes);

  T = size(videoframes,4);
  for t = 1:T
%     axes(handles.axes1)
%     imshow(squeeze(videoframes(:,:,:,t)))
    im_handles = imhandles(handles.axes1);
    set(im_handles(1),'CData',squeeze(videoframes(:,:,:,t)));
    axes(handles.axes2)
    tidx = max(1, floor(t/T*numel(handles.Apix)));
    
    plot(handles.tppg(1:tidx), Apix_filt(1:tidx), 'r','linewidth',2)
    hold on
    plot(handles.tppg(tidx), Apix_filt(tidx), 'or')
    hold off
    xlim([0 handles.tppg(end)])
    ylim([0 1])
    set(gca, 'ytick', [])
    ylabel('')
    xlabel('time (s)')
    
    pause(0.05)
    drawnow
  end
  
  answer = inputdlg('Save video?', 'Save Video');
  if ~isempty(answer) && ~isempty(answer{1})
    filename = answer{1};
    vout = VideoWriter(sprintf('C:/Users/ramelard/Desktop/%s.avi',filename),'Motion JPEG AVI');
    vout.FrameRate = 10;
    open(vout)
    writeVideo(vout, videoframes);
    close(vout)
  end

  
  display_success_map(handles, 'snr');
%   release(videoPlayer);


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

  % figure(curfig)
end


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
end


function plot_sigs(Apix, ppg, tppg, ax)
  if nargin < 4
    figure;
    ax = gca;
  end

  axes(ax)
  hold off
  % plot(linspace(0,tppg(end),numel(Apix)), Apix, 'k', 'linewidth', 2)

  Apix_ts = timeseries(Apix, tppg-tppg(1));
  Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
  Apix_filt = Apix_filt-min(Apix_filt);
  Apix_filt = Apix_filt./max(Apix_filt);
%   plot((Apix_filt-mean(Apix_filt))/std(Apix_filt)*std(Apix)+mean(Apix), 'r', 'linewidth', 2)
  plot(Apix_filt,'r','linewidth',2)
  
  hold on;
  ppg_offset = ppg-min(ppg);
  ppg_offset = ppg_offset/max(ppg_offset);
%   plot(tppg, ppg_offset, '--k','linewidth',2)


  ylim([min(ppg_offset)-0.5*std(ppg_offset) max(ppg_offset)+0.5*std(ppg_offset)])
  xlabel('time (s)')
  ylabel('')
  set(gca, 'ytick', [])
  grid on
  box on
  title('')
%   legend('Video','Finger')

  xlim([tppg(1) tppg(end)])
end


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
  stem(freq*60, Apix_mag.^2, 'r', 'linewidth', 2)
  hold on;
  stem(freq*60, ppg_mag.^2, '--k', 'linewidth', 1)
  
  [~,maxidx] = max(ppg_mag);
  t = text(freq(maxidx)*60+2, .9, 'heart rate');
  t.Color = 'red';
  t.FontSize = 12;
  % [ax,h1,h2] = plotyy(freq,Apix_mag.^2,freq,ppg_mag.^2);
  % set(h2,'color','b','linewidth',2,'linestyle','--')
  % set(h1,'color','r','linewidth',2)
  % set(ax,{'ycolor'},{'r';'b'})

  % set(ax(1),'ylim',[0 5])
  % set(ax(2),'ylim',[0 5])
  %   xlim(ax(1),[0 5])
  %   xlim(ax(2),[0 5])
  xlim([30 120])


  % plot(freq, ppg_mag.^2, 'r')
  % hold on
  % plot(freq, Apix_mag.^2, 'b', 'linewidth',2)
  % xlim([00/60 200/60])
  xlabel('beats per minute')
  ylabel('confidence')
  set(gca,'ytick',[])
  % title('Normalized Power Spectral Density','fontsize',14)
  title('')
  grid on
  box on
  legend('Video','Finger')
%   set(gca,'color',[0 0 0])
%   set(gca,'xcolor',[1 1 1])
%   set(gca,'ycolor',[1 1 1])
  box on
end


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
end


% --- Outputs from this function are returned to the command line.
function varargout = GUI_Demo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --- Executes on mouse press over axes background.
function image_ButtonDownFcn(hObject, eventdata, handles)
  % hObject    handle to axes1 (see GCBO)
  % eventdata  reserved - to be defined in a future version of MATLAB
  % handles    structure with handles and user data (see GUIDATA)

  % Display PPGI trace on axes2.
  a = get(gca,'CurrentPoint');
  x = a(1,1);
  y = a(1,2);
  fig_handles = guidata(hObject);

  fps = fig_handles.fps;
  block_size = fig_handles.block_size;
  tppg = fig_handles.tppg;
  ppg = fig_handles.ppg;

  topleft = [x y];
  rect = [floor(topleft(1)/block_size(1))*block_size(1) + 1 ...
          floor(topleft(2)/block_size(2))*block_size(2) + 1 ...
          block_size(1)-1 block_size(2)-1];
  [Rpix,~] = plot_pixel_vals(fig_handles.im,'fps',fps,'roi','rect',rect);
  Apix = -log(Rpix+1);

  % if get(fig_handles.cb_detrend, 'value') == 1
    Apix = detrend_sig(Apix);
  % end
  
  [freq, Apix_mag] = get_normalized_fmag(Apix, fps);
  [~,hridx] = max(Apix_mag);

%   set(fig_handles.axes2, 'Visible', 'on')
  plot_sigs(Apix, ppg, tppg, fig_handles.axes2);
  text((tppg(end)-tppg(1))*.67,1.25,sprintf('Heart Rate: %0.2g bpm', 60*freq(hridx)),'color','red','fontsize',12,'fontweight','bold')
  
  
  [~,ppg_mag] = get_normalized_fmag(ppg,fps);
  
  disp(sprintf('SNR=%0.3gdB', get_snr(Apix_mag, fps, ppg_mag)));
  % axes(fig_handles.axes2)
  % hold off
  % ppg_offset = (ppg-mean(ppg))/std(ppg)*std(Apix(:)) + mean(Apix(:));
  % plot(tppg, ppg_offset,'--b', 'linewidth', 2)
  % hold on
  % % plot(linspace(0,tppg(end),numel(Apix)), Apix, 'k', 'linewidth', 2)
  % 
  % [~, Apix_mag] = get_normalized_fmag(Apix, fps);
  % [~, ppg_mag] = get_normalized_fmag(ppg, numel(tppg)/(tppg(end)-tppg(1)));
  % snr_ppg_ppgi = get_snr(Apix_mag, fps, ppg_mag);
  % 
  % Apix_ts = timeseries(Apix, tppg-tppg(1));
  % Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
  % plot((Apix_filt-mean(Apix_filt))/std(Apix_filt)*std(Apix)+mean(Apix), 'r', 'linewidth', 2)
  % 
  % % Aentr = get_entropy(Apix_pow(f30bpm:f200bpm,:));
  % Aentr = get_entropy(Apix_mag.^2);
  % % PPGentr = get_entropy(ppg_pow(f30bpm:f200bpm));
  % PPGentr = get_entropy(ppg_mag.^2);
  % title(sprintf('SNR=%0.3g dB, H_{norm}=%0.3g (%0.3g)',snr_ppg_ppgi, Aentr, PPGentr))
  % ylim([min(ppg_offset)-0.5*std(ppg_offset) max(ppg_offset)+0.5*std(ppg_offset)])
  % xlabel('time (s)')
  % ylabel('amplitude (unitless)')
  % set(gca, 'ytick', [])

%   plot_psd(Apix, fps, ppg, tppg, fig_handles.axes3);

  fig_handles.Apix = Apix;
  guidata(gcf, fig_handles);
end


% --- Executes when selected object is changed in plotpanel.
function plotpanel_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in plotpanel 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(eventdata.NewValue, 'Tag'));
end


function snr = get_snr(Acam_mag, fps, ppg_mag)
  npts = size(Acam_mag,1)*2;
  f30bpm = ceil(30/60*npts/fps) + 1;
  f200bpm = floor(200/60*npts/fps) + 1;
  
%   if size(Acam_mag,2) == 1
%     Acam_mag = Acam_mag';
%   end
%   
%   % Make sure PSD is normalized.
%   Acam_mag(:,[1,end-4:end]) = 0;
% %   curfig=gcf; figure, plot(Acam_mag); figure(curfig);
%   ppg_mag(1) = 0;
%   Acam_mag = bsxfun(@rdivide, Acam_mag, max(Acam_mag,[],1));
%   ppg_mag = bsxfun(@rdivide, ppg_mag, max(ppg_mag));
  

  ppg_mag = ppg_mag./sum(ppg_mag);
  Acam_mag = bsxfun(@rdivide, Acam_mag, sum(Acam_mag,1));

  S = ppg_mag;
  N = bsxfun(@minus, Acam_mag, ppg_mag);

  snr = 10*log10(sum(S.^2,1)./sum(N.^2,1));
%   sqerr = bsxfun(@minus, Acam_mag, ppg_mag).^2;
%   snr = 10*log10(1./mean(sqerr,1));
%   snr = corr(Acam_mag, ppg_mag);
%   snr = corr(Acam_mag, ppg_mag).^2;
end



function entr = get_entropy(power_range)
  if size(power_range,1) == 1
    power_range = power_range(:);
  end

%   A = sqrt(power_range);
  % Make spectrum a PDF
  power_range = bsxfun(@rdivide, power_range, sum(power_range,1));
  % Happens when whole spectrum range has 0 power.
  power_range(isnan(power_range) | power_range==0) = 1E-100;
  % Use normalized entropy.
  entr = -sum(power_range.*log2(power_range)) / log2(size(power_range,1));
  if any(isnan(entr))
    warning('Entropy has NaN values');
  end
  if any(isinf(entr))
    warning('Entropy has Inf values');
  end
  
      f30bpm = 5;
      f200bpm = 21;
      
      Acam_pow_norm = bsxfun(@rdivide, power_range, sum(power_range,1));
      pct_in_hr = sum(Acam_pow_norm(f30bpm:f200bpm,:),1);
      max_over_mean = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./mean(Acam_pow_norm(f30bpm:f200bpm,:),1);
%       entr = entr./max(Acam_pow_norm(f30bpm:f200bpm,:),[],1);
%       entr = 1./entr;
%       entr = max(Acam_pow_norm(f30bpm:f200bpm,:),[],1)./entr;
%       entr = entr.*(1-pct_in_hr);
%       entr = entr.*(pct_in_hr<.2);
%       entr = pct_in_hr>.5;
%       entr = pct_in_hr;
%       a = .7165; b = 14.9;
%       x = 1-pct_in_hr;
%       entr = entr.*(a*log(b*x./(1-x)));
%       entr = pct_in_hr;
end

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
  
% B = exp(-B)-1;
  Bmean = mean(B,3);
%   varN = colfilt(Bmean, [3 3], 'sliding', @(xvec) var(xvec));
  Bmean_grad = imgradient(Bmean);
  spat = Bmean_grad(:)';
end
  

function [] = plot_entropy_grid(Aentr, handles)
  % Entropy-weighted power spectra
  alpha = logspace(-3,3,8);
  a2 = logspace(-17,-8,8);
  
  alpha = logspace(-4, 2, 8);
  a2 = logspace(-28, -10, 8);
  
%   alpha = 0.00518;
%   a2 = Inf;
  
  Acam = handles.Acam;
  if get(handles.cb_detrend, 'value') == 1
    Acam = detrend_sig(Acam);
  end
  
  [~, Acam_pow] = plot_power_spectrum(Acam, handles.fps);
  [ppg_freq, ppg_pow] = plot_power_spectrum(handles.ppg, numel(handles.tppg)/(handles.tppg(end)-handles.tppg(1)));

  curfig = gcf;
  f1 = figure;
  f2 = figure;

  spat = get_spatial_weight(Acam, handles.im, handles.block_size);

  FAcam = fft(Acam);


  cols = 2;
  rows = ceil(numel(alpha)*numel(a2)/cols);
  plotidx = 0;
  f30bpm = find(ppg_freq>30/60, 1)-1;
  f200bpm = find(ppg_freq>200/60, 1)-1;
%   f30bpm = ceil(30/60*npts/fps) + 1;
%   f200bpm = floor(200/60*npts/fps) + 1;
  for i = 1:numel(alpha)
    for j = 1:numel(a2)
      plotidx = plotidx + 1;

      w = exp(-Aentr.^2/alpha(i));
      w2 = exp(-spat.^2/a2(j));
      W = w.*w2;

      Pw_sum = sum(bsxfun(@times, Acam_pow, W),2)./sum(W);
      Pw_sum(1) = 0;
      
      % Plot frequency power
      figure(f1);
      subplot(rows, cols, plotidx);
      [ax,hline1,hline2] = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
      set(hline2,'color','r')
      set(ax(2),'ycolor','r')
      set(ax(1),'ytick',[])
      set(ax(2),'ytick',[])
      xlim(ax(1), [30/60 200/60])
      xlim(ax(2), [30/60 200/60])
      try
        ylim(ax(1), [0 max(Pw_sum(f30bpm:f200bpm))])
        ylim(ax(2), [0 max(ppg_pow(f30bpm:f200bpm))])
      catch
      end
      xlabel('frequency (Hz)')
      ylabel('spectral power')
%       title(sprintf('\\alpha_1=%0.3g, \\alpha_2=%0.3g', alpha(i), a2(j)))
      
      % Plot reconstructed temporal 
      figure(f2);
      subplot(rows, cols, plotidx);
      Fppgi = sum(bsxfun(@times, FAcam, W),2)./sum(W);
      Fppgi(1) = 0;
      [ax,hline1,hline2] = plotyy(handles.tppg, ifft(Fppgi), handles.tppg, handles.ppg);
      set(hline2,'color','r')
      set(ax(2),'ycolor','r')
      set(ax(1),'ytick',[])
      set(ax(2),'ytick',[])
      xlabel('time (s)')
      ylabel('amplitude')
    end
  end
  % Plot average at the end
  figure(f1);
  subplot(rows, cols, rows*cols)
  Pw_sum = mean(Acam_pow,2);
  Pw_sum(1) = 0;
  [ax,hline1,hline2] = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
  set(hline2,'color','r')
  set(ax(2),'ycolor','r')
  set(ax(1),'ytick',[])
  set(ax(2),'ytick',[])
  xlim(ax(1), [30/60 200/60]), xlim(ax(2), [30/60 200/60])
  xlabel('frequency (Hz)')
  ylabel('spectral power')
  title('Mean')
  
  figure(f2);
  subplot(rows, cols, rows*cols)
  Fppgi = mean(FAcam,2);
  Fppgi(1) = 0;
  ppgi = ifft(Fppgi);
  [ax,hline1,hline2] = plotyy(handles.tppg, ifft(Fppgi), handles.tppg, handles.ppg);
  set(hline2,'color','r')
  set(ax(2),'ycolor','r')
  set(ax(1),'ytick',[])
  set(ax(2),'ytick',[])
  xlabel('time (s)')
  ylabel('amplitude')
  
%   Pw_sum = mean(Acam_pow,2);
%   Pw_sum(1) = 0;
%   ax = plotyy(ppg_freq,Pw_sum,ppg_freq(2:end),ppg_pow(2:end));
%   xlim(ax(1), [0.5 3.5]), xlim(ax(2), [0.5 3.5])
  title('Mean')

%   set(f1,'color','w')
%   set(f2,'color','w')
  figure(curfig)
end
  
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
end


% --- Executes on button press in cb_overlay.
function cb_overlay_Callback(hObject, eventdata, handles)
% hObject    handle to cb_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
end

% --- Executes on button press in cb_fusion.
function cb_fusion_Callback(hObject, eventdata, handles)
% hObject    handle to cb_fusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
end

% --- Executes on button press in cb_detrend.
function cb_detrend_Callback(hObject, eventdata, handles)
% hObject    handle to cb_detrend (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cb_detrend
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
end

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
end

% --- Executes during object creation, after setting all properties.
function sl_overlay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end

% --- Executes on button press in pb_savesig.
function pb_savesig_Callback(hObject, eventdata, handles)
% hObject    handle to pb_savesig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  Apix = handles.Apix;
  answer = inputdlg('Signal filename', 'Save Signal');
  filename = answer{1};
  if numel(filename) < 4 || ~strcmpi(filename(end-3:end), '.mat')
    filename = [filename, '.mat'];
  end
  save(['signals/' filename], 'Apix')
end

% --------------------------------------------------------------------
function menu_scatter_Callback(hObject, eventdata, handles)
% hObject    handle to menu_scatter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

  % display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
  Acam = handles.Acam;
  if get(handles.cb_detrend, 'value') == 1
    Acam = detrend_sig(Acam);
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
  SNR = get_snr(Acam_mag, handles.fps, ppg_mag);
  plot_snr_scatter(SNR, handles.success_metric)
end

% --------------------------------------------------------------------
function menu_fusion_Callback(hObject, eventdata, handles)
% hObject    handle to menu_fusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

  if strcmpi(get(get(handles.plotpanel, 'selectedobject'), 'tag'), 'entr')
    plot_entropy_grid(handles.success_metric, handles)
  end
end

% --------------------------------------------------------------------
function pulsing_video_Callback(hObject, eventdata, handles)
% hObject    handle to pulsing_video (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

  guifig = gcf;
  if isempty(handles.Acam_filt)
    Acam = handles.Acam;
  %   if get(handles.cb_detrend, 'value') == 1
      Acam = detrend_sig(Acam);
  %   end
    d = dialog('Position', [800 600 100 40]); 
    uicontrol('Parent',d, 'Style','text', 'Position',[0 0 150 25],...
              'String','Filtering frames...');
    drawnow;
  %     [imW, imH, imT] = size(handles.im);
  %     frames_vec = reshape(-log(1+handles.im), imW*imH, imT);
  %     Apix_ts = timeseries(frames_vec', handles.tppg-handles.tppg(1));
  %     Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
  %     handles.Acam_filt = reshape(Apix_filt.Data', [imW, imH, imT]);

    Apix_ts = timeseries(Acam, handles.tppg-handles.tppg(1));
    Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
    pulsing_vid = zeros(size(handles.im,1)+handles.block_size(1), ...
                        size(handles.im,2)+handles.block_size(2), ...
                        size(handles.im,3));
    idx = 1;
    for c = 1:handles.block_size(2):size(handles.im,2)
      for r = 1:handles.block_size(1):size(handles.im,1)
        pulsing_vid(r:r+handles.block_size(1), c:c+handles.block_size(2), :) = ...
          repmat(shiftdim(Apix_filt.Data(:,idx)',-1), [handles.block_size+1 1]);
        idx = idx + 1;
      end
    end
    handles.Acam_filt = pulsing_vid(1:end-handles.block_size(1), 1:end-handles.block_size(2), :);
    close(d)
    guidata(guifig, handles);
  end
  % answer = inputdlg({'Video filename'}, 'Save Video', 1, {'pulsing'});
  % if ~isempty(answer)
  %   filename = answer{1};
  plot_type = get(get(handles.plotpanel, 'selectedobject'), 'tag');
    gen_pulsing(plot_type, handles.im, handles.Acam_filt, handles.corr_map, handles.block_size(1))
  % end
end

% --- Executes on button press in rb_demomode.
function rb_demomode_Callback(hObject, eventdata, handles)
% hObject    handle to rb_demomode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rb_demomode
display_success_map(handles, get(get(handles.plotpanel, 'selectedobject'), 'tag'));
end

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
  %   Apix = detrend_sig(Apix);
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
end

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  guifig = gcf;

  if strcmp(handles.timer.Running, 'on')
    stop(handles.timer)
  %   set(handles.pushbutton4, 'CData', imread('pause.png'));
    return
  end

  if isempty(handles.Acam_filt)
    Acam = handles.Acam;
  %   if get(handles.cb_detrend, 'value') == 1
      Acam = detrend_sig(Acam);
  %   end
    d = dialog('Position', [800 600 100 40]); 
    uicontrol('Parent',d, 'Style','text', 'Position',[0 0 150 25],...
              'String','Filtering frames...');
    drawnow;
  %     [imW, imH, imT] = size(handles.im);
  %     frames_vec = reshape(-log(1+handles.im), imW*imH, imT);
  %     Apix_ts = timeseries(frames_vec', handles.tppg-handles.tppg(1));
  %     Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
  %     handles.Acam_filt = reshape(Apix_filt.Data', [imW, imH, imT]);

    Apix_ts = timeseries(Acam, handles.tppg-handles.tppg(1));
    Apix_filt = idealfilter(Apix_ts, [.5 3.5], 'pass');
    pulsing_vid = zeros(size(handles.im,1)+handles.block_size(1), ...
                        size(handles.im,2)+handles.block_size(2), ...
                        size(handles.im,3));
    idx = 1;
    for c = 1:handles.block_size(2):size(handles.im,2)
      for r = 1:handles.block_size(1):size(handles.im,1)
        pulsing_vid(r:r+handles.block_size(1), c:c+handles.block_size(2), :) = ...
          repmat(shiftdim(Apix_filt.Data(:,idx)',-1), [handles.block_size+1 1]);
        idx = idx + 1;
      end
    end
    handles.Acam_filt = pulsing_vid(1:end-handles.block_size(1), 1:end-handles.block_size(2), :);
    close(d)
    guidata(guifig, handles);
  end
  % answer = inputdlg({'Video filename'}, 'Save Video', 1, {'pulsing'});
  % if ~isempty(answer)
  %   filename = answer{1};
  plot_type = get(get(handles.plotpanel, 'selectedobject'), 'tag');
  gen_pulsing(plot_type, handles.im, handles.Acam_filt, handles.corr_map, handles.block_size(1))
  % end
end
