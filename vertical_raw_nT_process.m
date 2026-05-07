% vertical_raw_nT_process.m
% Input data format: [Bx_nT, By_nT, Bz_nT, roll_deg, pitch_deg]
%
% Usage option 1:
%   data = readmatrix('your_data.csv');
%   run('vertical_raw_nT_process.m');
%
% Usage option 2:
%   inputFile = 'your_data.csv';
%   run('vertical_raw_nT_process.m');

if exist('inputFile', 'var')
    rawData = readmatrix(inputFile);
elseif exist('data', 'var')
    rawData = data;
else
    [fileName, filePath] = uigetfile({'*.csv;*.txt;*.dat;*.xlsx', 'Data Files'}, ...
                                     'Select 5-column magnetic data');
    if isequal(fileName, 0)
        error('No input data selected.');
    end
    rawData = readmatrix(fullfile(filePath, fileName));
end

if size(rawData, 2) < 5
    error('Input data must contain at least 5 columns: Bx, By, Bz, roll, pitch.');
end

rawData = rawData(:, 1:5);
rawData = rawData(all(isfinite(rawData), 2), :);
if isempty(rawData)
    error('Input data contains no valid numeric rows.');
end

Bx_nT = rawData(:, 1);
By_nT = rawData(:, 2);
Bz_nT = rawData(:, 3);
roll_deg = rawData(:, 4);
pitch_deg = rawData(:, 5);

roll_rad = roll_deg * pi / 180.0;
pitch_rad = pitch_deg * pi / 180.0;

ez_x = -sin(pitch_rad);
ez_y = sin(roll_rad) .* cos(pitch_rad);
ez_z = cos(roll_rad) .* cos(pitch_rad);

vertical_raw_nT = Bx_nT .* ez_x + By_nT .* ez_y + Bz_nT .* ez_z;
vertical_raw_span_nT = max(vertical_raw_nT) - min(vertical_raw_nT);

vertical_raw_result = table(Bx_nT, By_nT, Bz_nT, roll_deg, pitch_deg, ...
                            vertical_raw_nT);

fprintf('vertical_raw_nT peak-to-peak = %.6f nT\n', vertical_raw_span_nT);

figure;
plot(vertical_raw_nT, 'LineWidth', 1.0);
grid on;
xlabel('Sample');
ylabel('nT');
title(sprintf('vertical\\_raw\\_nT, peak-to-peak = %.3f nT', ...
              vertical_raw_span_nT));
