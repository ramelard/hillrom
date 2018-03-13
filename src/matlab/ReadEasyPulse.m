function [t,ppg] = ReadEasyPulse(com,baud_rate)

if nargin < 1
  com = 'COM4';
end
if nargin < 2
  baud_rate = 19200;
end

global comport
% Code issued by Arduino that steraming has stopped.
STOP_CODE = -99;

if ~isempty(comport)
  try
    fclose(comport);
    delete(comport)
  catch err
    warning('Could not close comport')
    comport
  end
  clear comport
end

comport = serial(com,'BaudRate',baud_rate);
fopen(comport);

% % Throw out the first message -- could have leftover from last time.
% % If none leftover, wait for Setup message
% while ~comport.BytesAvailable
% end
flushinput(comport);
% disp(fscanf(comport));

figure; hold off; movegui('southwest')
t = zeros(1,10000);
ppg = zeros(1,10000);
bavail = zeros(1,10000);
tidx = 1;
h = msgbox('Press OK to stop recording.');
movegui(h,'northwest');

% uiwait(msgbox('Start'))
flushinput(comport);

% A = fscanf(comport, '%s\n');
% while ~strcmpi(A,'Setup')
%   disp(A)
%   A = fscanf(comport, '%s\n');
% end

tic
while ishandle(h)

if comport.BytesAvailable
  % Input as "t ppg"
  A = fscanf(comport, '%u %u\n');
  if numel(A) ~= 2
    continue;
  end
  ti = A(1);
  ppgi = A(2);
  
  % Look for stop code
  if ti == STOP_CODE && ppgi == STOP_CODE
    break;
  end

  if numel(ti) ~= 1 || numel(ppgi) ~= 1
    disp(ti)
    disp(ppgi)
    continue;
  end

  if tidx > numel(t)
%     tidx = 1;
    t = [t zeros(1,1000)];
    ppg = [ppg zeros(1,1000)];
    bavail = [bavail zeros(1,1000)];
  end
  
  if tidx > 1 && ti/1000 == t(tidx)
    continue;
  end

  t(tidx) = ti/1000;
  ppg(tidx) = ppgi;
  bavail(tidx) = comport.BytesAvailable;
  if toc > 0.2
    tic
    twindow = 10;
    if t(tidx) > twindow
      dt = mean(diff(t(1:tidx)));
      idx = floor(twindow/dt);
      plot(t(tidx-idx:tidx),ppg(tidx-idx:tidx),t(tidx-idx:tidx),bavail(tidx-idx:tidx))
    else
      plot(t(1:tidx),ppg(1:tidx),t(1:tidx),bavail(1:tidx))
    end
    
    drawnow
  end

  tidx = tidx + 1;
else
  % Wait for 1ms to offer CPU control to msgbox.
  pause(0.001);
end

end

fclose(comport);
delete(comport)
clear global comport

t = t(1:tidx-1);
ppg = ppg(1:tidx-1);
% end

end