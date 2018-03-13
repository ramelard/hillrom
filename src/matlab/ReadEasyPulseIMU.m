% [tppg,ppg,euler_angles,gyro] = ReadEasyPulseIMU('COM6');
% S = [];
% S.tppg = tppg;
% S.ppg = ppg;
% S.euler_angles = euler_angles;
% S.gyro = gyro;
% 
% save('X:/data/imu/ppg_imu.mat','S')

function [t,ppg,euler_angles,gyro] = ReadEasyPulseIMU(com,baud_rate)

if nargin < 1
  com = 'COM6';
end
if nargin < 2
  baud_rate = 115200;
end

global comport

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
euler_angles = zeros(3,10000);
gyro = zeros(3,10000);
tidx = 1;
h = msgbox('Press OK to stop recording.');
movegui(h,'northwest');

% uiwait(msgbox('Start'))
% flushinput(comport);

out = '';
while (~strncmp('Setup Complete', out, 14))
    out = fscanf(comport);
    display(sprintf('%s',out));
end

tic
% fprintf(comport, 'U');
while ishandle(h)

if comport.BytesAvailable
  % Input as "t ppg"
  ti = fread(comport, 1, 'uint32');
  ppgi = fread(comport, 1, 'int16');
  quaternion_raw = fread(comport, 4, 'int16');
  gyro_raw = fread(comport, 3, 'int16');
  
  [euler_angle, gyro_dps] = imu_raw_to_euler(quaternion_raw, gyro_raw);
  
%   A = fscanf(comport, '%u %u\n');
%   if numel(A) ~= 2
%     continue;
%   end
%   B = fscanf(comport, '%g %g %g\n');
%   C = fscanf(comport, '%g %g %g\n');
%   
%   ti = A(1);
%   ppgi = A(2);

  if numel(ti) ~= 1 || numel(ppgi) ~= 1
    disp(ti)
    disp(ppgi)
    continue;
  end

  if tidx > numel(t)
%     tidx = 1;
    t = [t zeros(1,1000)];
    ppg = [ppg zeros(1,1000)];
    euler_angles = [euler_angles zeros(3,1000)];
    gyro = [gyro zeros(3,1000)];
    bavail = [bavail zeros(1,1000)];
  end
  
  if tidx > 1 && ti/1000 == t(tidx)
    continue;
  end

  t(tidx) = ti/1000;
  ppg(tidx) = ppgi;
  euler_angles(1,tidx) = euler_angle(1);
  euler_angles(2,tidx) = euler_angle(2);
  euler_angles(3,tidx) = euler_angle(3);
  gyro(1,tidx) = gyro_dps(1);
  gyro(2,tidx) = gyro_dps(2);
  gyro(3,tidx) = gyro_dps(3);
  
  disp(sprintf('%g %g %g\n%g %g %g',euler_angles(1),euler_angles(2),euler_angles(3), ...
    gyro(1),gyro(2),gyro(3)));
  
  bavail(tidx) = comport.BytesAvailable;
  if toc > 0.2
    tic
    plot(t,ppg,t,bavail)
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
euler_angles = euler_angles(:,1:tidx-1);
gyro = gyro(:,1:tidx-1);
% end

end