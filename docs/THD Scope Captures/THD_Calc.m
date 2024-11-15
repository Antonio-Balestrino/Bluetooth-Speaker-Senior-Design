% MATLAB Script to Calculate THD from dBV Readings
% 11/13/24

% Clear terminal, close graphs, etc
clc;            
clear all;     
close all; 
format short;   

% Offset in dBV (add this to all readings)
offset_dBV = -40.625;

% Volume level percentages
volume_levels = [6, 25, 50, 75, 100];  % Define volume levels as percentages

% Input dBV readings for each volume level (rows = volume levels, columns = harmonics)
% Each row corresponds to a different volume level (e.g., 6%, 25%, 50%, etc.)
dBV_readings = [ ...  
    % Volume level 1 (6%)
    [-12 -52 -52 -64 -64],  
    
    % Volume level 2 (25%)
    [-4 -56 -48 -52 -60],  
    
    % Volume level 3 (50%)
    [8 -52 -56 -68 -52],  

    % Volume level 4 (75%)
    [16 -24 -28 -32 -36],  

    % Volume level 5 (100%)
    [20 -4 4 -16 -20],  
];

% Number of volume levels
num_levels = size(dBV_readings, 1);

% Initialize vector to store THD results for each volume level
THD_results = zeros(num_levels, 1);

for i = 1:num_levels
    % Get the dBV readings for the current volume level
    current_readings = dBV_readings(i, :);
    
    % Apply offset to get the corrected dBV values
    corrected_dBV = current_readings + offset_dBV;
    
    % Convert corrected dBV readings to linear scale (volts)
    linear_voltages = 10.^(corrected_dBV / 20);
    
    % Separate fundamental and harmonics
    A1 = linear_voltages(1);            % Fundamental amplitude
    harmonics = linear_voltages(2:end); % Harmonic amplitudes
    
    % Calculate THD for the current volume level
    THD_results(i) = (sqrt(sum(harmonics.^2)) / A1) * 100;
end

% Display results with actual volume percentages
for i = 1:num_levels
    fprintf('Total Harmonic Distortion (THD) for Volume Level %d%%: %.2f%%\n', volume_levels(i), THD_results(i));
end
