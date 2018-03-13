 
% Script for JS presentation
%%
clear
clc

startup();

% ======================================================================= %
%% Example X: run Monte Carlo simulations accounting for absorption with
% analog and discrete absorption weighting with 1e6 photons and compare
% time and relative error

% create a default set of inputs
si = SimulationInput();
si.N = 1000000;
% simulation options initiation
options = SimulationOptions(); 
% seed of random number generator (-1=randomly selected seed, >=0 reproducible sequence)
options.Seed = 0;
si.Options = options;

% create a new 'instance' of the MultiLayerTissueInput class
tissueInput = MultiLayerTissueInput();
% assign the tissue layer regions struct
tissueInput.LayerRegions = struct(...
    'ZRange', ...
    {...
        [-Inf, 0], ... % air "z" range
        [0, 100], ... % tissue "z" range
        [100, +Inf] ... % air "z" range
    }, ...
    'RegionOP', ...
    {...
        [0.0, 1e-10, 1.0, 1.0], ... % air optical properties
        [0.01, 1.0, 0.8, 1.4], ... % tissue OPs [ua, us', g, n]
        [0.0, 1e-10, 1.0, 1.0] ... % air optical properties
    } ...
);
si.TissueInput = tissueInput;

% specify a single R(rho) detector by the endpoints of rho bins
si.DetectorInputs = { DetectorInput.ROfRho(linspace(0,10,101)) };
si.DetectorInputs{1}.TallySecondMoment = true;

% specify analog absorption and run simulation
si.Options.AbsorptionWeightingType = 'Analog';
disp('Analog results:');
output1 = VtsMonteCarlo.RunSimulation(si);

% specify discrete absorption weighting (DAW) and run simulation
si.Options.AbsorptionWeightingType = 'Discrete';
disp('Discrete absorption weighting results:');
output2 = VtsMonteCarlo.RunSimulation(si);

d1 = output1.Detectors(output1.DetectorNames{1});
d2 = output2.Detectors(output2.DetectorNames{1});
% determine standard deviation using 1st and 2nd moment results
d1SD = sqrt((d1.SecondMoment - (d1.Mean .* d1.Mean)) / si.N);
d2SD = sqrt((d2.SecondMoment - (d2.Mean .* d2.Mean)) / si.N);
% plot R(rho) using both methods with errorbars
f=figure; 
set(f,'Name','Spatially-resolved reflectance with 1-sigma error bars');
subplot(2,1,1);
errorbar(d1.Rho(1:end-1), d1.Mean(1:end-1), d1SD(1:end-1),'r-');
hold on;
errorbar(d2.Rho(1:end-1), d2.Mean(1:end-1), d2SD(1:end-1),'b-');
set(gca,'YScale','log');
axis([0 10, 1e-6 1]);
title('log(R(\rho)) [mm^-^2]'); %xlabel('Rho (mm)');
l=legend('analog','DAW');
set(l,'FontSize',10);

% plot analog relative error - DAW relative error
subplot(2,1,2);
plot(d1.Rho, (d1SD./d1.Mean-d2SD./d2.Mean),'g-',d1.Rho,zeros(length(d1.Rho),1),'k:');
axis([0 10, 0.0 max(d1SD./d1.Mean-d2SD./d2.Mean)]);
ax=gca;
%ax.YTick=[0,0.02,0.04];
%ax.YTickLabel=['0','0.02','0.04'];
title('analog relative error - DAW relative error'); xlabel('Rho (mm)');

