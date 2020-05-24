# Radar Target Generation and Detection
| Date | Software  | Source  | Author |
| :---:   | :-: | :----: |  :-: |
| '2020-05-24' | 'MATLAB 2018a'  |'radar\_target\_generation\_and\_detection.m'  | Puneet Tiwari |

This project uses Matlab to introduce frequency modulated continuous-wave (FMCW) radar and related post-processing techniques. The topics covered include:

1. Fast Fourier transforms (FFT) and 2D FFT
2. Clutter v. target discrimination
3. Sizing chirp bandwith to meet system requirements for range resolution
4. Phased array beam steering to determine angle of arrival (AoA)
5. Constant false alarm rate (CFAR) noise suppression
6. Signal-to-noise ratio (SNR) and dynamic thresholding

---
## Project Summary Writeup
### Implementation steps for the 2D CFAR process
The 2D constant false alarm rate (CFAR), when applied to the results of the 2D FFT, uses a dynamic threshold set by the noise level in the vicinity of the cell under test (CUT). The key steps are as follows:

- Loop over all cells in the range and doppler dimensions, starting and ending at indices which leave appropriate margins
- Slice the training cells (and exclude the guard cells) surrounding the CUT
- Convert the training cell values from decibels (dB) to power, to linearize
- Find the mean noise level among the training cells
- Convert this average value back from power to dB
- Add the offset (in dB) to set the dynamic threshold
- Apply the threshold and store the result in a binary array of the same dimensions as the range doppler map (RDM)

There is potential room for performance improvement though parallelization. These sliding window type operations may be expressed as a convolution.

### Selection of training cells, guard cells, and offset
The values were hand selected and can be improved further as well. A rectangular window with the major dimension along the range cells will be selected. This window selection produces better filtered results from the given RDM. Choosing the right value for offset is the key to isolating the simulated target and avoiding false positives. Finally, no of training values can be precalculated to avoid a performance hit in the nested loop.

### Steps taken to suppress the non-thresholded cells at the edges
In 2D CFAR implementation, only CUT locations with sufficient margins to contain the entire window are considered. An empty array of zeros can be intantiated which is equivalent to the RDM array size. Set the indexed locations to `1` if and only if the threshold is exceeded by the CUT.

## Project Detailed Writeup
![](img/project.png)

1. Configure the FMCW waveform based on the system requirements.
2. Define the range and velocity of target and simulate its displacement.
3. For the same simulation loop process the transmit and receive signal to determine the beat signal
4. Perform Range FFT on the received signal to determine the Range
5. Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.

``` {.codeinput}
clear all;
clc;
```

## Radar Specifications
--------------------
- Frequency of operation = 77GHz
- Max Range = 200m
- Range Resolution = 1 m
- Max Velocity = 100 m/s

``` {.codeinput}
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
```

## User Defined Range and Velocity of target
-----------------------------------------

> **%TODO** : define the target's initial position and velocity. 
> Note : Velocity remains contant

``` {.codeinput}
target_range = 100;
target_velocity = 50;

radar_max_range = 200;
radar_range_resolution = 1;
radar_max_velocity = 100;
speed_of_light = 3e8;
```

## FMCW Waveform Generation
------------------------

> 1. Design the FMCW waveform by giving the specs of each of its parameters.
> 2. Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW chirp using the requirements above.
> 3. Bandwidth (B) = speed of light / (2 * radar range resolution)
> 4. The sweep time can be computed based on the time needed for the signal to travel the unambiguous maximum range. In general, for an FMCW radar system, the sweep time should be at least 5 to 6 times the round trip time. This example uses a factor of 5.5.
> 5. Chirp Time(t_chrip) = sweep factor * 2 * (radar maximum range / speed of light)
> 6. slope = bandwidth / ChripTime

``` {.codeinput}
%Operating carrier frequency of Radar
fc = 77e9;             %carrier freq Hz
B = speed_of_light /(2 * radar_range_resolution);

t_sweep = 5.5;
t_chirp = t_sweep * 2 * (radar_max_range/speed_of_light);
slope = B / t_chirp;
```
> The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT for Doppler Estimation.

``` {.codeinput}
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps
```
> The number of samples on each chirp.

``` {.codeinput}
Nr = 1024;                  %for length of time OR # of range cells
```
> Timestamp for running the displacement scenario for every sample on each chirp

``` {.codeinput}
t = linspace(0,Nd*t_chirp,Nr*Nd); %total time for samples
```
> Creating the vectors for Tx, Rx and Mix based on the total samples input.
``` {.codeinput}
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal
```
> Similar vectors for range_covered and time delay.
``` {.codeinput}
r_t = zeros(1, length(t));
td = zeros(1, length(t));
```

## Signal generation and Moving Target simulation
----------------------------------------------

> Running the radar scenario over the time.

``` {.codeinput}
for i = 1 : length(t)
  % *%TODO* :
  %For each time stamp update the Range of the Target for constant velocity.
  r_t(i) = target_range + (target_velocity*t(i));
  td(i) = (2 * r_t(i)) / speed_of_light;

  % *%TODO* :
  %For each time sample we need update the transmitted and
  %received signal.
  Tx(i) = cos(2 * pi * (fc * t(i) + 0.5 * slope * t(i)^2));
  Rx(i)  = cos(2 * pi * (fc * (t(i) - td(i)) + 0.5 * slope * (t(i) - td(i))^2));

  % *%TODO* :
  %Now by mixing the train_cellsansmit and Receive generate the beat signal
  %This is done by element wise matrix multiplication of train_cellsansmit and
  %Receiver Signal
  Mix(i) = Tx(i) .* Rx(i);
end
```

## RANGE MEASUREMENT
-----------------
> reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of Range and Doppler FFT respectively.
``` {.codeinput}
Mix = reshape(Mix, [Nr, Nd]);
```
> run the FFT on the beat signal along the range bins dimension (Nr) and normalize.
``` {.codeinput}
sig_fft = fft(Mix, Nr);
```
> Take the absolute value of FFT output
``` {.codeinput}
sig_fft = abs(sig_fft);
sig_fft = sig_fft ./ max(sig_fft); % Normalize
```
> Output of FFT is double sided signal, but we are interested in only one side of the spectrum. Hence we throw out half of the samples.
``` {.codeinput}
sig_fft = sig_fft(1 : Nr/2-1);
```
-----------------

![](img/radar_target_generation_and_detection_01.png)

## RANGE DOPPLER RESPONSE
----------------------
> 1. The 2D FFT implementation is already provided here.
> 2. This will run a 2DFFT on the mixed signal (beat signal) output and generate a range doppler map. 
> 3. You will implement CFAR on the generated RDM Range Doppler Map Generation. 
> 4. The output of the 2D FFT is an image that has reponse in the range and doppler FFT bins. So, it is important to convert the axis from bin sizes to range and doppler based on their Max values.

``` {.codeinput}
% Range Doppler Map Generation.
Mix = reshape(Mix, [Nr, Nd]);
```
> 2D FFT using the FFT size for both dimensions.
``` {.codeinput}
sig_fft2 = fft2(Mix, Nr, Nd);
```
> Taking just one side of signal from Range dimension.
``` {.codeinput}
sig_fft2 = sig_fft2(1 : Nr/2, 1 : Nd);
sig_fft2 = fftshift(sig_fft2);

range_doppler_map = abs(sig_fft2);
range_doppler_map = 10 * log10(range_doppler_map);
```
> use the surf function to plot the output of 2DFFT and to show axis in both dimensions
``` {.codeinput}
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr/2) * ((Nr/2) / 400);

% Additional views of the surface plot
figure ('Name', 'Amplitude and Range From FFT2');
surf(doppler_axis, range_axis, range_doppler_map);
title('Amplitude and Range From FFT2');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');
```
![](img/radar_target_generation_and_detection_02.png)
![](img/radar_target_generation_and_detection_03.png)

## CFAR implementation
-------------------
> Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.
> Slide Window through the complete Range Doppler Map
> Select the number of Training Cells in both the dimensions.
``` {.codeinput}
train_cells = 10;
train_band = 8;
```
> Select the number of Guard Cells in both dimensions around the Cell under test (CUT) for accurate estimation
``` {.codeinput}
guard_cells = 4;
guard_band = 4;
```
> offset the threshold by SNR value in dB
``` {.codeinput}
offset = 1.4;
```
> 1. Design a loop such that it slides the CUT across range doppler map by giving margins at the edges for train_cellsaining and Guard Cells.
> 2. For every iteration sum the signal level within all the training cells. To sum convert the value from logarithmic to linear using db2pow function.
> 3. Average the summed values for all of the training cells used. After averaging convert it back to logarithimic using pow2db.
> 4. Further add the offset to it to determine the threshold. 
> 5. Next, compare the signal under CUT with this threshold. 
> 6. If the CUT level > threshold assign, it a value of 1, else equate it to 0.
> 7. Use RDM[x,y] as the matrix from the output of 2D FFT for implementing CFAR

``` {.codeinput}
range_doppler_map = range_doppler_map / max(range_doppler_map(:));

for row1 = train_cells + guard_cells + 1 : (Nr/2) - (train_cells + guard_cells)
  for col1 = train_band + guard_band + 1 : (Nd) - (train_band + guard_band)
    %Create a vector to store noise_level for each iteration on training cells
    noise_level = zeros(1, 1);

    for row2 = row1 - (train_cells + guard_cells) : row1 + (train_cells + guard_cells)
      for col2 = col1 - (train_band + guard_band) : col1 + (train_band + guard_band)
        if (abs(row1 - row2) > guard_cells || abs(col1 - col2) > guard_band)
          noise_level = noise_level + db2pow(range_doppler_map(row2, col2));
        end
      end
    end

    % Calculate threshold from noise average then add the offset
    threshold = pow2db(noise_level / (2 * (train_band + guard_band + 1) * 2 * (train_cells + guard_cells + 1) - (guard_cells * guard_band) - 1));
    threshold = threshold + offset;

    cell_under_test = range_doppler_map(row1,col1);

    if (cell_under_test < threshold)
      range_doppler_map(row1, col1) = 0;
    else
      range_doppler_map(row1, col1) = 1;
    end

  end
end
```
> The process above will generate a thresholded block, which is smaller than the Range Doppler Map as the CUT cannot be located at the edges of matrix. Hence,few cells will not be thresholded. To keep the map size same set those values to 0.
``` {.codeinput}
range_doppler_map(range_doppler_map~=0 & range_doppler_map~=1) = 0;
```
> Display the CFAR output using the Surf function like we did for Range Doppler Response output.
``` {.codeinput}
figure('Name', 'CA-CFAR Filtered RDM')
surf(doppler_axis, range_axis, range_doppler_map);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
view(315, 45);
```

![](img/radar_target_generation_and_detection_04.png)
![](img/radar_target_generation_and_detection_05.png)
![](img/radar_target_generation_and_detection_06.png)


