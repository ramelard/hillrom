% Scatter, Bland-Altman
figure,
hr_ppg = 60*hr(3,:)';
hr_ppgface = 60*hr(2,:)';
hr_ppgi = 60*hr(1,:)';

p=panel();
p.pack(1,4)
p(1,1).select()
plot(hr_ppgi, hr_ppg, '.k','markersize',10)
xvals = [0 200];
pfit = polyfit(hr_ppgi, hr_ppg, 1);
yvals = polyval(pfit, xvals);
hold on, plot(xvals, yvals, 'k')
xlabel('predicted heart rate (bpm)')
ylabel('true heart rate (bpm)')
xlim([50 95])
ylim([50 95])
text(50.5,93,sprintf('r^2=%0.4g',corr(hr_ppgi,hr_ppg)^2))

p(1,3).select()
plot(hr_ppgface, hr_ppg, '.k','markersize',10)
pfit = polyfit(hr_ppgface, hr_ppg, 1);
yvals = polyval(pfit, xvals);
hold on, plot(xvals, yvals, 'k')
xlabel('predicted heart rate (bpm)')
ylabel('true heart rate (bpm)')
xlim([50 95])
ylim([50 95])
text(50.5,93,sprintf('r^2=%0.4g',corr(hr_ppgface,hr_ppg)^2))

p(1,2).select()
h=plot((hr_ppg+hr_ppgi)./2, hr_ppg-hr_ppgi, 'ok', 'markersize', 4);
set(h,'markerfacecolor','k')
x0x1 = get(gca,'xlim');
mu = mean(hr_ppg-hr_ppgi);
sigma = std(hr_ppg-hr_ppgi);
line(x0x1, [mu mu], 'linewidth',2,'color',[0.5 0.5 0.5])
line(x0x1, mu+1.96*[sigma sigma], 'linestyle','--','color',[0.5 0.5 0.5])
line(x0x1, mu-1.96*[sigma sigma], 'linestyle','--','color',[0.5 0.5 0.5])
text(x0x1(2)-10,mu+1.96*sigma+1.5,'\mu+1.96\sigma')
text(x0x1(2)-10,mu-1.96*sigma-1.5,'\mu-1.96\sigma')
ylim([-30 30])
xlabel('heart rate (bpm)')
ylabel('error (\Deltabpm)')

set(gcf,'position',[1         437        1432         420])