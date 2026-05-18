% Passive gimbal what-if simulation for ProjectFusionBoard logs.
%
% Usage:
%   simulate_passive_gimbal
%   simulate_passive_gimbal('fusiondata.txt', 50)
%
% Model:
% - Input is the board/body roll and pitch from the UART log.
% - A passive gravity-stabilized probe is approximated as a damped second
%   order residual coupling from body angle to probe angle.
% - static_coupling = 0 means an ideal frictionless gimbal keeps the probe
%   perfectly vertical for pure body rotations.
% - Larger static_coupling models cable torque, bearing friction, imbalance,
%   and imperfect gravity stabilization.

function result = simulate_passive_gimbal(log_file, sample_rate_hz)
if nargin < 1 || isempty(log_file)
    log_file = 'fusiondata.txt';
end

if nargin < 2 || isempty(sample_rate_hz)
    sample_rate_hz = read_sample_rate_hz(log_file, 50);
end

dt = 1 / sample_rate_hz;
data = read_fusion_log(log_file);
if height(data) < 5
    error('Not enough valid data rows in %s.', log_file);
end

b_nT = [data.b1_nT, data.b2_nT, data.b3_nT];
roll_body_deg = fill_missing_linear(data.stm32_roll_deg);
pitch_body_deg = fill_missing_linear(data.stm32_pitch_deg);
n = height(data);
t = (0:n-1)' * dt;

baseline_count = min(n, max(5, round(3 * sample_rate_hz)));
roll0 = median(roll_body_deg(1:baseline_count));
pitch0 = median(pitch_body_deg(1:baseline_count));
roll_disturbance = roll_body_deg - roll0;
pitch_disturbance = pitch_body_deg - pitch0;

default_params = struct();
default_params.fn_hz = 0.70;
default_params.zeta = 0.90;
default_params.static_coupling = 0.08;

[roll_probe_deg, pitch_probe_deg] = simulate_gimbal_angles( ...
    roll_disturbance, pitch_disturbance, roll0, pitch0, dt, default_params);

b_nav_nT = body_to_nav_field(b_nT, roll_body_deg, pitch_body_deg);
b_probe_nT = nav_to_body_field(b_nav_nT, roll_probe_deg, pitch_probe_deg);

vertical_body_nT = vertical_projection(b_nT, roll_body_deg, pitch_body_deg);
vertical_body_b3_nT = b_nT(:, 3);
vertical_gimbal_raw_nT = b_probe_nT(:, 3);
vertical_gimbal_projected_nT = vertical_projection(b_probe_nT, roll_probe_deg, pitch_probe_deg);

warmup_s = 3.0;
body_range = signal_range_after(t, vertical_body_nT, warmup_s);
body_b3_range = signal_range_after(t, vertical_body_b3_nT, warmup_s);
gimbal_raw_range = signal_range_after(t, vertical_gimbal_raw_nT, warmup_s);
gimbal_projected_range = signal_range_after(t, vertical_gimbal_projected_nT, warmup_s);

fn_grid = [0.25, 0.40, 0.70, 1.00, 1.50];
zeta_grid = [0.40, 0.70, 1.00, 1.40];
coupling_grid = [0.02, 0.05, 0.08, 0.12, 0.20];
sweep = sweep_gimbal_params(b_nav_nT, roll_disturbance, pitch_disturbance, ...
                            roll0, pitch0, t, dt, warmup_s, ...
                            fn_grid, zeta_grid, coupling_grid);

output_dir = 'fusion_figures';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
plot_passive_gimbal_result(output_dir, t, roll_body_deg, pitch_body_deg, ...
                           roll_probe_deg, pitch_probe_deg, ...
                           vertical_body_nT, vertical_body_b3_nT, ...
                           vertical_gimbal_raw_nT, vertical_gimbal_projected_nT, ...
                           sweep, default_params);

fprintf('\nPassive gimbal simulation\n');
fprintf('  log file: %s\n', log_file);
fprintf('  sample rate: %.3f Hz\n', sample_rate_hz);
fprintf('  default fn/zeta/coupling: %.2f Hz / %.2f / %.2f\n', ...
        default_params.fn_hz, default_params.zeta, default_params.static_coupling);
fprintf('  vertical range after %.1f s:\n', warmup_s);
fprintf('    body b3 raw:                %.3f nT\n', body_b3_range);
fprintf('    body attitude projection:   %.3f nT\n', body_range);
fprintf('    passive gimbal b3 raw:      %.3f nT\n', gimbal_raw_range);
fprintf('    passive gimbal projected:   %.3f nT\n', gimbal_projected_range);
fprintf('    raw b3 improvement factor:  %.3f x\n', body_b3_range / max(gimbal_raw_range, eps));
fprintf('  best sweep range: %.3f nT at fn=%.2f Hz, zeta=%.2f, coupling=%.2f\n', ...
        sweep.best_range_nT, sweep.best_fn_hz, sweep.best_zeta, sweep.best_coupling);
fprintf('  saved figure:\n    %s\n\n', fullfile(output_dir, '05_passive_gimbal_sim.png'));

result = struct();
result.time_s = t;
result.params = default_params;
result.roll_body_deg = roll_body_deg;
result.pitch_body_deg = pitch_body_deg;
result.roll_probe_deg = roll_probe_deg;
result.pitch_probe_deg = pitch_probe_deg;
result.vertical_body_nT = vertical_body_nT;
result.vertical_body_b3_nT = vertical_body_b3_nT;
result.vertical_gimbal_raw_nT = vertical_gimbal_raw_nT;
result.vertical_gimbal_projected_nT = vertical_gimbal_projected_nT;
result.body_range_nT = body_range;
result.body_b3_range_nT = body_b3_range;
result.gimbal_raw_range_nT = gimbal_raw_range;
result.gimbal_projected_range_nT = gimbal_projected_range;
result.sweep = sweep;
end

function data = read_fusion_log(log_file)
fid = fopen(log_file, 'r');
if fid < 0
    error('Cannot open log file: %s', log_file);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
values = zeros(0, 14);

while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end

    line = strtrim(line);
    if isempty(line)
        continue;
    end

    if strncmp(line, '{plotter}', 9)
        line = strtrim(line(10:end));
    elseif strncmp(line, '{text}', 6)
        continue;
    end

    if isempty(regexp(line, '^[\+\-0-9\.]', 'once'))
        continue;
    end

    number_tokens = regexp(line, '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');
    row = str2double(number_tokens);

    if (numel(row) == 11 || numel(row) == 13 || numel(row) == 14) && all(isfinite(row))
        parsed = nan(1, 14);
        parsed(1:numel(row)) = row(:).';
        values(end + 1, :) = parsed; %#ok<AGROW>
    end
end

data = array2table(values, ...
    'VariableNames', {'b1_nT', 'b2_nT', 'b3_nT', ...
                      'acc_x', 'acc_y', 'acc_z', ...
                      'gyro_x', 'gyro_y', 'gyro_z', ...
                      'stm32_roll_deg', 'stm32_pitch_deg', ...
                      'stm32_vertical_raw_nT', 'stm32_vertical_corr_nT', ...
                      'stm32_spike_flag'});
end

function sample_rate_hz = read_sample_rate_hz(log_file, fallback_hz)
sample_rate_hz = fallback_hz;
fid = fopen(log_file, 'r');
if fid < 0
    return;
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
for i = 1:200
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end

    token = regexp(line, 'frequency\s*=\s*([0-9.]+)\s*Hz', 'tokens', 'once');
    if ~isempty(token)
        value = str2double(token{1});
        if isfinite(value) && value > 0
            sample_rate_hz = value;
        end
        break;
    end
end
end

function [roll_probe_deg, pitch_probe_deg] = simulate_gimbal_angles( ...
    roll_disturbance, pitch_disturbance, roll0, pitch0, dt, params)
roll_residual = second_order_coupling(roll_disturbance, dt, params.fn_hz, ...
                                      params.zeta, params.static_coupling);
pitch_residual = second_order_coupling(pitch_disturbance, dt, params.fn_hz, ...
                                       params.zeta, params.static_coupling);
roll_probe_deg = roll0 + roll_residual;
pitch_probe_deg = pitch0 + pitch_residual;
end

function y = second_order_coupling(u, dt, fn_hz, zeta, static_coupling)
u = u(:);
n = numel(u);
y = zeros(n, 1);
rate = 0.0;
wn = 2 * pi * fn_hz;
y(1) = static_coupling * u(1);

for k = 2:n
    target = static_coupling * u(k);
    accel = (wn * wn) * (target - y(k - 1)) - (2 * zeta * wn) * rate;
    rate = rate + accel * dt;
    y(k) = y(k - 1) + rate * dt;
end
end

function vertical_nT = vertical_projection(b_nT, roll_deg, pitch_deg)
roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);
vertical_unit = [-sin(pitch), sin(roll) .* cos(pitch), cos(roll) .* cos(pitch)];
vertical_nT = sum(b_nT .* vertical_unit, 2);
end

function b_nav_nT = body_to_nav_field(b_body_nT, roll_deg, pitch_deg)
n = size(b_body_nT, 1);
b_nav_nT = zeros(n, 3);
for k = 1:n
    R_body_to_nav = body_to_nav_matrix(roll_deg(k), pitch_deg(k));
    b_nav_nT(k, :) = (R_body_to_nav * b_body_nT(k, :).').';
end
end

function b_body_nT = nav_to_body_field(b_nav_nT, roll_deg, pitch_deg)
n = size(b_nav_nT, 1);
b_body_nT = zeros(n, 3);
for k = 1:n
    R_body_to_nav = body_to_nav_matrix(roll_deg(k), pitch_deg(k));
    b_body_nT(k, :) = (R_body_to_nav.' * b_nav_nT(k, :).').';
end
end

function R = body_to_nav_matrix(roll_deg, pitch_deg)
r = deg2rad(roll_deg);
p = deg2rad(pitch_deg);
cr = cos(r);
sr = sin(r);
cp = cos(p);
sp = sin(p);

R = [ cp,       sr * sp,  cr * sp; ...
      0.0,      cr,      -sr; ...
     -sp,       sr * cp,  cr * cp];
end

function sweep = sweep_gimbal_params(b_nav_nT, roll_disturbance, pitch_disturbance, ...
                                     roll0, pitch0, t, dt, warmup_s, ...
                                     fn_grid, zeta_grid, coupling_grid)
range_map = nan(numel(fn_grid), numel(zeta_grid), numel(coupling_grid));
best_range = Inf;
best_fn = fn_grid(1);
best_zeta = zeta_grid(1);
best_coupling = coupling_grid(1);

for i = 1:numel(fn_grid)
    for j = 1:numel(zeta_grid)
        for k = 1:numel(coupling_grid)
            params = struct('fn_hz', fn_grid(i), ...
                            'zeta', zeta_grid(j), ...
                            'static_coupling', coupling_grid(k));
            [roll_probe, pitch_probe] = simulate_gimbal_angles( ...
                roll_disturbance, pitch_disturbance, roll0, pitch0, dt, params);
            b_probe = nav_to_body_field(b_nav_nT, roll_probe, pitch_probe);
            vertical_probe_raw = b_probe(:, 3);
            r = signal_range_after(t, vertical_probe_raw, warmup_s);
            range_map(i, j, k) = r;
            if r < best_range
                best_range = r;
                best_fn = fn_grid(i);
                best_zeta = zeta_grid(j);
                best_coupling = coupling_grid(k);
            end
        end
    end
end

sweep = struct();
sweep.fn_grid = fn_grid;
sweep.zeta_grid = zeta_grid;
sweep.coupling_grid = coupling_grid;
sweep.range_map_nT = range_map;
sweep.best_range_nT = best_range;
sweep.best_fn_hz = best_fn;
sweep.best_zeta = best_zeta;
sweep.best_coupling = best_coupling;
end

function r = signal_range_after(t, x, warmup_s)
valid = isfinite(x) & (t >= warmup_s);
if ~any(valid)
    r = NaN;
    return;
end
r = max(x(valid)) - min(x(valid));
end

function y = fill_missing_linear(x)
x = x(:);
t = (1:numel(x))';
valid = isfinite(x);
if all(valid)
    y = x;
elseif any(valid)
    y = interp1(t(valid), x(valid), t, 'linear', 'extrap');
else
    y = zeros(size(x));
end
end

function plot_passive_gimbal_result(output_dir, t, roll_body, pitch_body, ...
                                    roll_probe, pitch_probe, ...
                                    vertical_body, vertical_body_b3, ...
                                    vertical_gimbal_raw, vertical_gimbal_projected, ...
                                    sweep, params)
fig = figure('Color', 'w', 'Name', 'Passive gimbal simulation', ...
             'Position', [80, 80, 1500, 950]);

subplot(3, 1, 1);
plot(t, roll_body, 'Color', [0.0 0.45 0.74]); hold on;
plot(t, roll_probe, 'Color', [0.45 0.0 0.65], 'LineWidth', 1.5);
grid on;
xlabel('s');
ylabel('deg');
title('Roll: body vs passive-gimbal probe');
legend('body', 'probe residual', 'Location', 'best');

subplot(3, 1, 2);
plot(t, pitch_body, 'Color', [0.85 0.33 0.10]); hold on;
plot(t, pitch_probe, 'Color', [0.45 0.0 0.65], 'LineWidth', 1.5);
grid on;
xlabel('s');
ylabel('deg');
title('Pitch: body vs passive-gimbal probe');
legend('body', 'probe residual', 'Location', 'best');

subplot(3, 1, 3);
plot(t, vertical_body_b3, 'Color', [0.55 0.55 0.55]); hold on;
plot(t, vertical_body, 'Color', [0.0 0.45 0.74]);
plot(t, vertical_gimbal_raw, 'Color', [0.45 0.0 0.65], 'LineWidth', 1.5);
plot(t, vertical_gimbal_projected, 'Color', [0.0 0.55 0.0], 'LineStyle', '--');
grid on;
xlabel('s');
ylabel('nT');
title(sprintf('Vertical projection, fn=%.2f Hz, zeta=%.2f, coupling=%.2f', ...
      params.fn_hz, params.zeta, params.static_coupling));
legend('body b3 raw', 'body projection', 'gimbal b3 raw', 'gimbal projected', 'Location', 'best');

exportgraphics(fig, fullfile(output_dir, '05_passive_gimbal_sim.png'), 'Resolution', 160);
close(fig);

fig = figure('Color', 'w', 'Name', 'Passive gimbal parameter sweep', ...
             'Position', [120, 120, 1100, 700]);
[~, best_coupling_idx] = min(abs(sweep.coupling_grid - sweep.best_coupling));
imagesc(sweep.zeta_grid, sweep.fn_grid, sweep.range_map_nT(:, :, best_coupling_idx));
set(gca, 'YDir', 'normal');
colorbar;
grid on;
xlabel('damping ratio');
ylabel('natural frequency (Hz)');
title(sprintf('Vertical range sweep at static coupling %.2f', sweep.best_coupling));
exportgraphics(fig, fullfile(output_dir, '06_passive_gimbal_sweep.png'), 'Resolution', 160);
close(fig);
end
