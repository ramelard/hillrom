%Script to convert ADU to power


h = 6.62606957e-34; %Plancks constant
c=3e8; %Speed of light
laser_wv_m = %Laser wavelength in metres
adu = %pixel value out of 2^16
qe = %Quantum efficiency at laser_wv_m
gain_adu_per_e = 
shutter_speed = %integration time in seconds
pixels_in_spot = %For single pixel, this is 1

E_p = h*c/laser_wv_m; %Energy per photon in Watts

photons_per_pixel = convertADU2Photon(adu,qe,gain_adu_per_e);


photons_in_spot = photons_per_pixel*pixels_in_spot;

energy = photons_in_spot*E_p; 

Power = energy/shutter_speed;

