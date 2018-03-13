%% Calibrate EasyPulse
user = 'BL';
ReadEasyPulse(9600);

%% TEST 1 - AT REST
[tppg,ppg] = ReadEasyPulse;

save(sprintf('C:/Users/ramelard/Desktop/data/experiments/%s/test1/tppg_ppg.mat',user),'tppg','ppg');

%% TEST 2 - AT REST + CONTROLLED BREATHING
system('start controlled_breathing\ControlledBreathing\x64\Release\ControlledBreathing.exe');
[tppg,ppg] = ReadEasyPulse;
save(sprintf('C:/Users/ramelard/Desktop/data/experiments/%s/test2/tppg_ppg.mat',user),'tppg','ppg');

%% TEST 3 - AT REST + LAMP OCCLUSION (1 LIGHT)
[tppg,ppg] = ReadEasyPulse;

save(sprintf('C:/Users/ramelard/Desktop/data/experiments/%s/test3/tppg_ppg.mat',user),'tppg','ppg');

%% TEST 4 - AT REST + LAMP OCCLUSION (2 LIGHTS)
[tppg,ppg] = ReadEasyPulse;

save(sprintf('C:/Users/ramelard/Desktop/data/experiments/%s/test4/tppg_ppg.mat',user),'tppg','ppg');

%% TEST 5 - DISTANCE
[tppg,ppg] = ReadEasyPulse;

save(sprintf('C:/Users/ramelard/Desktop/data/experiments/%s/test5/tppg_ppg.mat',user),'tppg','ppg');
