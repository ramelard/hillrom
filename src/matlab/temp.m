R = rand(601,632);
  
% R = exp(-x)-1;
xhat = zeros(size(R));
tic;
parfor i = 1:size(R,2)
  xhat(:,i) = kalmanfilt(R(:,i)', 2.5e-4, 1.3e-3);
end
toc
% xhat = -log(xhat+1);