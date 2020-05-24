clc;
clear all;
close all;

Fs = 1e3;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal i.e. 1.5 secs
t = (0:L-1)*T;        % Time vector

%% 2D signal
S = 2*cos(2*pi*77*t) +  2*cos(2*pi*43*t);

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.
P = peaks(20);
X = repmat(P,[5 10]);
imagesc(X)

%% 2-D Transform
% TODO : Compute the 2-D Fourier transform of the data.  
signalFFT = fft2(X);
%Shift the zero-frequency component to the center of the output, and 
% plot the resulting 100-by-200 matrix, which is the same size as X.
imagesc(abs(fftshift(signalFFT)))

%% Pad X with zeros to compute a 128-by-256 transform.
Y = fft2(X,2^nextpow2(100),2^nextpow2(200));
imagesc(abs(fftshift(Y)));
