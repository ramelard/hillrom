%% Monte Carlo Demo
% Script for demoing use of VTS Monte Carlo tools within Matlab, to view
% the source code see vts_mc_demo.m
%%
clear all
clc

startup();
%%
% ======================================================================= %
% Example 6: run a Monte Carlo simulation with pMC post-processing enabled
% First run a simulation, then post-process the generated database with
% varying optical properties
si = SimulationInput();
si.N = 100000;
% output folder name
si.OutputName = 'results';
options = SimulationOptions();
options.AbsorptionWeightingType = 'Discrete';
% modify database generation to specifying creating pMC reflectance database
options.Databases = { Vts.MonteCarlo.DatabaseType.pMCDiffuseReflectance };
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
        [0.01, 1.0, 0.8, 1.4], ... % tissue optical properties
        [0.0, 1e-10, 1.0, 1.0] ... % air optical properties
        } ...
    );
si.Options = options;
si.TissueInput = tissueInput;
si.DetectorInputs = { DetectorInput.ROfRho(linspace(0,10,51)) };
output = VtsMonteCarlo.RunSimulation(si);
%% pMC
% specify post-processing of generated database 
ppi = PostProcessorInput();
% specify detector based on baseline infile tissue optical properties
di = DetectorInput.pMCROfRho(linspace(0,10,51));
di.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.01,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di.PerturbedRegions = [ 1 ];
% specify detector with perturbed mus = 0.5xbaseline
di0p5xmua = DetectorInput.pMCROfRho(linspace(0,10,51),'pMCROfRho_0p5xmua');
di0p5xmua.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.005,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di0p5xmua.PerturbedRegions = [ 1 ]; 
% specify detector with perturbed mus = 2xbaseline
di2xmua = DetectorInput.pMCROfRho(linspace(0,10,51),'pMCROfRho_2xmua');
di2xmua.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.02,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di2xmua.PerturbedRegions = [ 1 ];
ppi.InputFolder = si.OutputName;
ppi.DetectorInputs = { di, di0p5xmua, di2xmua} ;
ppoutput = VtsMonteCarlo.RunPostProcessor(ppi,si);
do = ppoutput.Detectors(ppoutput.DetectorNames{1});
do0p5xmua = ppoutput.Detectors(ppoutput.DetectorNames{2});
do2xmua = ppoutput.Detectors(ppoutput.DetectorNames{3});
figure; semilogy(do.Rho, do.Mean, 'r-',do0p5xmua.Rho, do0p5xmua.Mean,'b-', do2xmua.Rho, do2xmua.Mean, 'b--','LineWidth',2); 
set(gca,'FontName','Helvetica');
ylabel('pMC: R(rho)','FontSize',24); xlabel('rho (mm)','FontSize',24);
axis([0 10,1e-4 0.5]); 
legend('baseline','0.5x mua','2x mua','Location','Best');
saveas(gca,'pmcROfRhoMuaPert.eps','epsc');
%% dMC
ppi = PostProcessorInput();
% specify detector based on baseline infile tissue optical properties
di = DetectorInput.dMCdROfRhodMua(linspace(0,10,51));
di.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.01,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di.PerturbedRegions = [ 1 ];
% specify detector with perturbed mus = 0.5xbaseline
di0p5xmua = DetectorInput.dMCdROfRhodMua(linspace(0,10,51),'dMCROfRho_0p5xmua');
di0p5xmua.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.005,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di0p5xmua.PerturbedRegions = [ 1 ]; 
% specify detector with perturbed mus = 2xbaseline
di2xmua = DetectorInput.dMCdROfRhodMua(linspace(0,10,51),'dMCROfRho_2xmua');
di2xmua.PerturbedOps = ...
                [...
                [1e-10, 0.0, 0.0, 1.0]; ...
                [0.02,   1.0, 0.8, 1.4]; ...
                [1e-10, 0.0, 0.0, 1.0]; ...
                ];
di2xmua.PerturbedRegions = [ 1 ];
ppi.InputFolder = si.OutputName;
ppi.DetectorInputs = { di, di0p5xmua, di2xmua} ;
ppoutput = VtsMonteCarlo.RunPostProcessor(ppi,si);
do = ppoutput.Detectors(ppoutput.DetectorNames{1});
do0p5xmua = ppoutput.Detectors(ppoutput.DetectorNames{2});
do2xmua = ppoutput.Detectors(ppoutput.DetectorNames{3});
figure; plot(do.Rho, do.Mean, 'r-',do0p5xmua.Rho, do0p5xmua.Mean,'b-', do2xmua.Rho, do2xmua.Mean, 'b--','LineWidth',2); 
set(gca,'FontName','Helvetica');
ylabel('dR/dmua','FontSize',24); xlabel('rho (mm)','FontSize',24);
legend('baseline','0.5x mua','2x mua','Location','Best');
saveas(gca,'dmcROfRhoMuaPert.eps','epsc');
