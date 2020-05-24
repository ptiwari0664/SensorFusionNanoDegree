clc
clear all
close all

%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength
waveLength = c/fc;

%TODO : Measure the Maximum Range a Radar can see. 
numerator = Pt * (G^2) * (waveLength^2) * RCS;
denominator = Ps * (4*pi)^3;

Rmax = (numerator/denominator)^(1/4);
disp(Rmax);