clc
clear all
close all

%Speed of light
c = 3*10^8;
Rmax = 300;
delta_t = 1;
% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = (c/2)*delta_t;

% TODO : Calculate the chirp time based on the Radar's Max Range
chripTime = (5.5 * 2 * Rmax)/c;

% TODO : define the frequency shifts 
beatFrequencies = [0e+6 1.1e+6 13e+6 24e+6];

calculated_range = (c/2) * ((beatFrequencies * chripTime)/Bsweep);

% Display the calculated range
disp(calculated_range);