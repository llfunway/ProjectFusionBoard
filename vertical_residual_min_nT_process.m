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

vertical_unit = zeros(size(rawData, 1), 3);
vertical_unit(:, 1) = -sin(pitch_rad);
vertical_unit(:, 2) = sin(roll_rad) .* cos(pitch_rad);
vertical_unit(:, 3) = cos(roll_rad) .* cos(pitch_rad);

mag_nT = [Bx_nT, By_nT, Bz_nT];
vertical_raw_nT = sum(mag_nT .* vertical_unit, 2);

baseline_alpha = 0.0020;
adapt_mu = 0.0300;
weight_decay = 0.0002;
min_horizontal_nT = 1000.0;
max_error_nT = 20000.0;
max_weight = 0.2500;
norm_floor = 1000000.0;

sampleCount = size(rawData, 1);
vertical_corr_nT = zeros(sampleCount, 1);
turn_residual_nT = zeros(sampleCount, 1);
turn_correction_nT = zeros(sampleCount, 1);
baseline_nT = zeros(sampleCount, 1);
weights_log = zeros(sampleCount, 3);

weights = [0.0, 0.0, 0.0];
baseline = vertical_raw_nT(1);
vertical_corr_nT(1) = vertical_raw_nT(1);
baseline_nT(1) = baseline;
weights_log(1, :) = weights;

for k = 2:sampleCount
    horizontal = mag_nT(k, :) - vertical_raw_nT(k) * vertical_unit(k, :);
    horizontal_norm_sq = sum(horizontal .* horizontal);

    correction = sum(weights .* horizontal);
    corrected = vertical_raw_nT(k) - correction;
    error_nT = corrected - baseline;

    turn_correction_nT(k) = correction;
    vertical_corr_nT(k) = corrected;
    turn_residual_nT(k) = error_nT;

    if horizontal_norm_sq > (min_horizontal_nT * min_horizontal_nT) && ...
       abs(error_nT) < max_error_nT
        adapt_gain = adapt_mu * error_nT / (horizontal_norm_sq + norm_floor);
        weights = weights * (1.0 - weight_decay) + adapt_gain * horizontal;
        weights = min(max(weights, -max_weight), max_weight);
    end

    baseline = baseline + baseline_alpha * (corrected - baseline);
    baseline_nT(k) = baseline;
    weights_log(k, :) = weights;
end

vertical_raw_span_nT = max(vertical_raw_nT) - min(vertical_raw_nT);
vertical_corr_span_nT = max(vertical_corr_nT) - min(vertical_corr_nT);

vertical_residual_min_result = table(Bx_nT, By_nT, Bz_nT, roll_deg, pitch_deg, ...
                                     vertical_raw_nT, vertical_corr_nT, ...
                                     turn_residual_nT, turn_correction_nT, ...
                                     baseline_nT);

fprintf('vertical_raw_nT  peak-to-peak = %.6f nT\n', vertical_raw_span_nT);
fprintf('vertical_corr_nT peak-to-peak = %.6f nT\n', vertical_corr_span_nT);
fprintf('final weights = [%.8f, %.8f, %.8f]\n', weights(1), weights(2), weights(3));

figure;
plot(vertical_raw_nT, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.0);
hold on;
plot(vertical_corr_nT, 'b', 'LineWidth', 1.0);
grid on;
xlabel('Sample');
ylabel('nT');
legend('vertical\_raw\_nT', 'vertical\_corr\_nT');
title(sprintf('Raw %.3f nT, corrected %.3f nT peak-to-peak', ...
              vertical_raw_span_nT, vertical_corr_span_nT));

figure;
plot(turn_correction_nT, 'LineWidth', 1.0);
grid on;
xlabel('Sample');
ylabel('nT');
title('turn\_correction\_nT');
