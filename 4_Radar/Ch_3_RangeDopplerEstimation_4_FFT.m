clc;
clear all;
close all;

Fs = 1e3;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal i.e. 1.5 secs
t = (0:L-1)*T;        % Time vector

%% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a
        % 43Hz sinusoid of amplitude 2.
        % signal = A*cos(2*pi*f*t)
S = 2*cos(2*pi*77*t) +  2*cos(2*pi*43*t);

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the 
    %frequency components by looking at the signal X(t). 
plot(1000*t(1:50) ,X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

%% TODO : Compute the Fourier transform of the signal. 
            %signal_fft = fft(signal,N);
S_fft = fft(X);

%% TODO : Compute the two-sided spectrum P2. Then compute the single-sided 
            %spectrum P1 based on P2 and the even-valued signal length L.
P2 = abs(S_fft/L);
P1 = P2(1:L/2+1);

%% Plotting
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
