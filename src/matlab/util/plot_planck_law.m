lambda = [0.1:.01:15];  % um
% T = 5800;  % the sun
% T = 310;  % body temperature
T = 2450;

plot(lambda, planck_law(lambda./10^6, T)./10^12)
ylabel('black body curve')
xlabel('\lambda (um)')
ylabel('Spectral radiance (kW sr^-1 m^-2 nm^-1)')