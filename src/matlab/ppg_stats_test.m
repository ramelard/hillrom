function ppg_stats_test(Apre,Apost,tppg,ppg,ppg_tidx)

L = log4m.getLogger('log.txt');
L.debug('ppg_stats_test',['Heart beat instances: ', num2str(ppg_tidx(:)')]);

% Sample PPG so its sample rate is the same as the camera frame rate.
% (Makes it easy for comparing).
xx = linspace(tppg(1),tppg(end),numel(Apost));
yy = spline(tppg,ppg,xx);

corr_pre = zeros(numel(ppg_tidx)-1,1);
corr_post = zeros(numel(ppg_tidx)-1,1);
for i = 1:numel(ppg_tidx)-1
  t0 = ppg_tidx(i);
  t1 = ppg_tidx(i+1);
  
  corr_pre(i) = corr(Apre(t0:t1)',yy(t0:t1)');
  corr_post(i) = corr(Apost(t0:t1)',yy(t0:t1)');
end

% Test the null hypothesis that the pairwise difference between correlation
% vectors has a mean=0. TWO TAILED TEST.
[h,p,ci,~] = ttest(corr_pre, corr_post);
disp(sprintf('Paired-sample t-test: h=%g, p=%g, CI [%g,%g], n=%u', ...
             h, p, ci(1), ci(2), numel(ppg_tidx)-1))
if h == 1
  disp('Reject null hypothesis')
else
  disp('Accept null hypothesis')
end

% % Test the null hypothesis that the correlation vectors comes from
% % independent normal distributions with equal means and equal but unknown
% % variances.
% [h,p,ci,~] = ttest2(corr_pre, corr_post);
% disp(sprintf('Two-sample t-test: h=%g, p=%g, CI [%g,%g]',h,p,ci(1),ci(2)))