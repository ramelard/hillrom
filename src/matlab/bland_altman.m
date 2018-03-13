function bland_altman(y,x)

if nargin < 2
  x = 1:numel(y);
end

mu = mean(y);
sigma = std(y);

figure, hold on;
plot(x,y,'x','MarkerSize',3)
plot(x,mu*ones(1,numel(x)),'-k')
plot(x,mu+sigma*1.96*ones(1,numel(x)),'--k')
plot(x,mu-sigma*1.96*ones(1,numel(x)),'--k')