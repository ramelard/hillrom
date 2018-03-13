
clear; clc; close all;

answer = inputdlg({'Participant id','Trial #'});
pid = answer{1};
trial = answer{2};
[tppg,ppg] = ReadEasyPulse;
save(sprintf('X:/heartfailure/%s/%s/tppg_ppg.mat',pid,trial),'tppg','ppg')
