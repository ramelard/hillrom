function [photons] = convertADU2Photon(adu,qe,gain)
gain_adu_per_e =gain;
ADU=adu;

elec_per_pixel = ADU/gain_adu_per_e;

photon_per_pixel=elec_per_pixel/qe;

photons = photon_per_pixel;
