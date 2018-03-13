function label_heartbeats(data_dir)

if nargin < 1
  data_dir = 'C:/Users/ramelard/Desktop/data/experiments/RA/test1/';
end

D = dir([data_dir, '/*.pgm']);
last_file = D(end).name;
dash_idx = find(D(end).name == '-');

fbase = [data_dir, D(1).name(1:dash_idx(4)), '%04u.pgm'];

% Try to load ppg data from saved file.
ppg_data_file = sprintf('%stppg_ppg.mat', data_dir);
fprintf(1,'Loading data file %s\n', ppg_data_file)
load(ppg_data_file)

ppg_sample_rate = numel(tppg)/tppg(end);

% Sample PPG so its sample rate is the same as the camera frame rate.
xx = linspace(tppg(1),tppg(end),numel(D));
yy = spline(tppg,ppg,xx);
tppg_sampled = xx;
ppg_sampled = yy;

figure;
tvalley = [];  % times of valleys
for k = 5:5:round(numel(tppg)/ppg_sample_rate)
  T0 = max(round((k-5)*ppg_sample_rate), 1);
  T1 = min(round(k*ppg_sample_rate), numel(tppg));
  
  tppg_window = tppg(T0:T1);
  ppg_window = ppg(T0:T1);
  
  % Guided gradient descent.
  plot(tppg_window, ppg_window);
  xlim([tppg_window(1) tppg_window(end)])
  ylim([min(ppg) max(ppg)])
  [t,~] = ginput;
  
  tvalley = [tvalley; t(:)];
end

close;

% Gradient descent
ppg_grad = [0 diff(ppg)];
thresh = 0.00001;  % threshold for stopping
gamma = 0.1;  % learning rate
ppg_tidx = zeros(size(tvalley));  % index of valleys
for i = 1:numel(tvalley)
  x = find(tppg >= tvalley(i), 1, 'First');
  x = [x round(x(end)-gamma*ppg_grad(x(end)))];
  while abs(x(end)-x(end-1)) > thresh
    x = [x round(x(end)-gamma*ppg_grad(x(end)))];
    
    if numel(x) > 100
      x = median(x(end-10:end));
      break;
    end
  end
  ppg_tidx(i) = find(tppg_sampled >= tppg(x(end)), 1, 'First');
end

figure, plot(tppg_sampled, ppg_sampled)
hold on, plot(tppg_sampled(ppg_tidx),ppg_sampled(ppg_tidx),'or')

save(sprintf('%sppg_tidx.mat', data_dir), 'ppg_tidx')


