%% Calibrate EasyPulse
user = 'RA';
ReadEasyPulse(9600);

%% Collect
fps = 50;
switch_rate = 3;

[tppg,ppg] = ReadEasyPulse;

savestr = sprintf('C:/Users/ramelard/Desktop/data/SPIE/%s/test-f%u-s%u/tppg_ppg.mat',user,fps,switch_rate);
save(savestr,'tppg','ppg');
