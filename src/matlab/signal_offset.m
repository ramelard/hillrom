% SIGNAL_OFFSET Computes the time offset between two signals.
% 
%   [timeDiff] = SIGNAL_OFFSET(s1, s2, fs) computes the time difference
%   between signals s1 and s2
function [timeDiff] = signal_offset(s1, s2, fs)

if min(size(s1)) > 1 || min(size(s2)) > 1
  error(sprintf('Expected 1D signals ([%u %u], [%u %u])', size(s1), size(s2)))
end

s1 = s1(:);
s2 = s2(:);

% load signals/Abottom.mat;
% A1 = Apix;
% load signals/Achin.mat
% A1 = Apix;
% load signals/Atop.mat;
% A2 = Apix;

[acor,lag] = xcorr(s2,s1);

[~,I] = max(abs(acor));
lagDiff = lag(I);
timeDiff = lagDiff/fs;

disp(sprintf('Signals are offset by %gs', timeDiff))

figure
plot(lag/fs,acor)
a3 = gca;
a3.XTick = sort([-3000:1000:3000 lagDiff]);

t = 1/fs.*[0:numel(s1)-1];
figure, plot(t,s1), hold on, plot(t,s2, 'r')