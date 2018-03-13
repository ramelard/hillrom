% lambda in [m], T in [K]
% plot(lambda, planck_law(lambda./10^6, 5000)./10^12), xlabel('\lambda (um)'), ylabel('Spectral radiance (kW sr^-1 m^-2 nm^-1)')
function R = planck_law(lambda, T)

h = 6.62606957E-34;  % Planck's constant [m^2 kg/s)
k = 1.3806488E-23;  % Boltzmann's constant ([m^2 kg s^-2 K^-1)
c = 299792458;  % speed of light in medium (m/s)

R = 2*h*c^2./lambda.^5 .* 1./(exp(h*c./(lambda*k*T))-1);