% Offline processing for ProjectFusionBoard UART logs.
%
% Expected columns:
% b1_nT b2_nT b3_nT acc_x acc_y acc_z gyro_x gyro_y gyro_z
% Optional columns:
% stm32_roll_deg stm32_pitch_deg
%
% Usage:
%   process_fusion_log
%   process_fusion_log('your_log.txt')
%   process_fusion_log('your_log.txt', 50)
%
% Notes:
% - acc_* is expected in g.
% - gyro_* is expected in deg/s.
% - If the log has no timestamp, the script uses the frequency=xxHz line if
%   present, otherwise falls back to 50 Hz.

function result = process_fusion_log(log_file, sample_rate_hz)
if nargin < 1
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

b_raw_columns = [data.b1_nT, data.b2_nT, data.b3_nT];
[b, mag_voltage_calibration] = apply_voltage_calibration_if_needed(b_raw_columns);
acc = [data.acc_x, data.acc_y, data.acc_z];
gyro_dps = [data.gyro_x, data.gyro_y, data.gyro_z];
validate_raw_imu_log(acc, gyro_dps, log_file);
stm32_roll_deg = data.stm32_roll_deg;
stm32_pitch_deg = data.stm32_pitch_deg;
stm32_vertical_raw_nT = data.stm32_vertical_raw_nT;
stm32_vertical_corr_nT = data.stm32_vertical_corr_nT;

n = height(data);
t = (0:n-1)' * dt;

gyro_bias_samples = min(n, max(10, round(3 * sample_rate_hz)));
gyro_bias_dps = median(gyro_dps(1:gyro_bias_samples, :), 1);
gyro_dps_bias_removed = gyro_dps - gyro_bias_dps;

[roll_acc_deg, pitch_acc_deg] = accel_roll_pitch(acc);
[roll_kalman_deg, pitch_kalman_deg] = kalman_roll_pitch(acc, gyro_dps_bias_removed, dt);
[roll_mahony_deg, pitch_mahony_deg, q_mahony] = mahony_stm32_roll_pitch(acc, gyro_dps_bias_removed, dt);
roll_acc_deg = align_angle_branch(roll_acc_deg, roll_kalman_deg);
roll_mahony_deg = align_angle_branch(roll_mahony_deg, roll_kalman_deg);
stm32_roll_deg = align_optional_angle_branch(stm32_roll_deg, roll_kalman_deg);
pitch_acc_deg = align_angle_branch(pitch_acc_deg, pitch_kalman_deg);
pitch_mahony_deg = align_angle_branch(pitch_mahony_deg, pitch_kalman_deg);
stm32_pitch_deg = align_optional_angle_branch(stm32_pitch_deg, pitch_kalman_deg);

vertical_acc = vertical_projection(b, roll_acc_deg, pitch_acc_deg);
vertical_kalman = vertical_projection(b, roll_kalman_deg, pitch_kalman_deg);
vertical_mahony = vertical_projection(b, roll_mahony_deg, pitch_mahony_deg);
vertical_stm32_mahony = vertical_projection_optional(b, stm32_roll_deg, stm32_pitch_deg);

[roll_deploy_src_deg, pitch_deploy_src_deg, deploy_attitude_source] = select_deployable_attitude( ...
    stm32_roll_deg, stm32_pitch_deg, roll_mahony_deg, pitch_mahony_deg);
[deploy_delay_s, roll_deploy_delay_deg, pitch_deploy_delay_deg] = optimize_attitude_delay( ...
    b, roll_deploy_src_deg, pitch_deploy_src_deg, t);
vertical_deploy_delay = vertical_projection(b, roll_deploy_delay_deg, pitch_deploy_delay_deg);
[vertical_deploy_nlms, vertical_deploy_rls, vertical_deploy_final, deploy_model] = ...
    deployable_vertical_pipeline(b, roll_deploy_delay_deg, pitch_deploy_delay_deg, ...
                                 vertical_deploy_delay, dt);
deploy_final_lag_s = estimate_signal_lag(vertical_deploy_rls, vertical_deploy_final, dt, 2.0);

[R_mag_to_imu, align_euler_deg, b_aligned] = estimate_mag_to_imu_alignment(b, roll_mahony_deg, pitch_mahony_deg);
vertical_mahony_aligned = vertical_projection(b_aligned, roll_mahony_deg, pitch_mahony_deg);
[b_calibrated, mag_calibration] = calibrate_magnetic_axes(b);
[best_delay_s, roll_delay_deg, pitch_delay_deg] = optimize_attitude_delay( ...
    b_calibrated, roll_mahony_deg, pitch_mahony_deg, t);
[R_cal_to_imu, align_cal_euler_deg, b_cal_aligned] = estimate_mag_to_imu_alignment( ...
    b_calibrated, roll_delay_deg, pitch_delay_deg);
vertical_cal_aligned = vertical_projection(b_cal_aligned, roll_delay_deg, pitch_delay_deg);
[vertical_pipeline, residual_model] = residual_regression_compensate( ...
    b_cal_aligned, roll_delay_deg, pitch_delay_deg, vertical_cal_aligned, dt);

result = struct();
result.sample_rate_hz = sample_rate_hz;
result.time_s = t;
result.b_nT = b;
result.b_raw_columns = b_raw_columns;
result.mag_voltage_calibration = mag_voltage_calibration;
result.acc_g = acc;
result.gyro_dps = gyro_dps;
result.gyro_bias_dps = gyro_bias_dps;
result.gyro_dps_bias_removed = gyro_dps_bias_removed;
result.roll_acc_deg = roll_acc_deg;
result.pitch_acc_deg = pitch_acc_deg;
result.roll_kalman_deg = roll_kalman_deg;
result.pitch_kalman_deg = pitch_kalman_deg;
result.roll_mahony_deg = roll_mahony_deg;
result.pitch_mahony_deg = pitch_mahony_deg;
result.stm32_roll_deg = stm32_roll_deg;
result.stm32_pitch_deg = stm32_pitch_deg;
result.stm32_vertical_raw_nT = stm32_vertical_raw_nT;
result.stm32_vertical_corr_nT = stm32_vertical_corr_nT;
result.q_mahony = q_mahony;
result.R_mag_to_imu = R_mag_to_imu;
result.align_euler_deg = align_euler_deg;
result.b_aligned_nT = b_aligned;
result.mag_calibration = mag_calibration;
result.best_delay_s = best_delay_s;
result.R_cal_to_imu = R_cal_to_imu;
result.align_cal_euler_deg = align_cal_euler_deg;
result.b_calibrated_nT = b_calibrated;
result.b_cal_aligned_nT = b_cal_aligned;
result.residual_model = residual_model;
result.vertical_acc_nT = vertical_acc;
result.vertical_kalman_nT = vertical_kalman;
result.vertical_mahony_nT = vertical_mahony;
result.vertical_stm32_mahony_nT = vertical_stm32_mahony;
result.deploy_attitude_source = deploy_attitude_source;
result.deploy_delay_s = deploy_delay_s;
result.roll_deploy_delay_deg = roll_deploy_delay_deg;
result.pitch_deploy_delay_deg = pitch_deploy_delay_deg;
result.vertical_deploy_delay_nT = vertical_deploy_delay;
result.vertical_deploy_nlms_nT = vertical_deploy_nlms;
result.vertical_deploy_rls_nT = vertical_deploy_rls;
result.vertical_deploy_final_nT = vertical_deploy_final;
result.deploy_final_lag_s = deploy_final_lag_s;
result.deploy_model = deploy_model;
result.vertical_mahony_aligned_nT = vertical_mahony_aligned;
result.vertical_cal_aligned_nT = vertical_cal_aligned;
result.vertical_pipeline_nT = vertical_pipeline;

plot_results(result);
print_summary(result);
end

function data = read_fusion_log(log_file)
fid = fopen(log_file, 'r');
if fid < 0
    error('Cannot open log file: %s', log_file);
end

cleanup_obj = onCleanup(@() fclose(fid));
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

    if (numel(row) == 9 || numel(row) == 11 || numel(row) == 13 || numel(row) == 14) && all(isfinite(row))
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

function [b_nT, calibration] = apply_voltage_calibration_if_needed(b_input)
% New STM32 firmware already prints calibrated nT. For older logs or manual
% voltage captures, auto-convert when the first three columns look like volts.
frontend_scale = 4.0;
sensitivity_v_per_oe = [10.3904, 10.5235, 10.9590];
zero_offset_v = [-0.1350, -0.1949, -0.1795];
nt_per_oe = 100000.0;

calibration = struct();
calibration.frontend_scale = frontend_scale;
calibration.sensitivity_v_per_oe = sensitivity_v_per_oe;
calibration.zero_offset_v = zero_offset_v;
calibration.nt_per_oe = nt_per_oe;
calibration.input_was_voltage = false;

if isempty(b_input)
    b_nT = b_input;
    return;
end

if median(abs(b_input(:))) < 100
    sensor_voltage = b_input * frontend_scale;
    b_nT = bsxfun(@rdivide, bsxfun(@minus, sensor_voltage, zero_offset_v), ...
                  sensitivity_v_per_oe) * nt_per_oe;
    calibration.input_was_voltage = true;
else
    b_nT = b_input;
end
end

function validate_raw_imu_log(acc, gyro_dps, log_file)
acc_norm = sqrt(sum(acc.^2, 2));
acc_norm_med = median(acc_norm);
gyro_abs_med = median(abs(gyro_dps), 1);

if acc_norm_med > 5 || acc_norm_med < 0.2
    error(['The file %s does not look like the new 9-column raw IMU log.\n' ...
           'Median |acc| is %.3f, but acc_x/acc_y/acc_z should be in g and close to 1.\n' ...
           'You are probably using the old format: b1 b2 b3 roll pitch vertical_raw vertical_corr residual correction.\n' ...
           'Please collect a new log after flashing the firmware that prints:\n' ...
           'b1_nT b2_nT b3_nT acc_x acc_y acc_z gyro_x gyro_y gyro_z'], ...
           log_file, acc_norm_med);
end

if acc_norm_med < 0.75 || acc_norm_med > 1.25
    warning(['Median |acc| is %.3f g, not close to 1 g. ', ...
             'This suggests the MPU6050 accelerometer range/sensitivity scale is still mismatched. ', ...
             'The current firmware gates correction by the startup |acc| reference, but the scale should still be checked.'], ...
             acc_norm_med);
end

if any(gyro_abs_med > 2000)
    warning(['Gyro median magnitude is very large: [%.3f %.3f %.3f] deg/s.\n' ...
             'Check whether gyro columns are really deg/s raw IMU data.'], ...
             gyro_abs_med(1), gyro_abs_med(2), gyro_abs_med(3));
end
end

function sample_rate_hz = read_sample_rate_hz(log_file, fallback_hz)
sample_rate_hz = fallback_hz;
fid = fopen(log_file, 'r');
if fid < 0
    return;
end

cleanup_obj = onCleanup(@() fclose(fid));
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

function [roll_deg, pitch_deg] = accel_roll_pitch(acc)
ax = acc(:, 1);
ay = acc(:, 2);
az = acc(:, 3);

roll_deg = atan2d(ay, az);
pitch_deg = atan2d(-ax, sqrt(ay.^2 + az.^2));
end

function [roll_deg, pitch_deg] = kalman_roll_pitch(acc, gyro_dps, dt)
n = size(acc, 1);
acc_filt = lowpass_vector(acc, dt, 5.0);
[roll_acc, pitch_acc] = accel_roll_pitch(acc_filt);

roll_filter = init_angle_kalman(roll_acc(1));
pitch_filter = init_angle_kalman(pitch_acc(1));

roll_deg = zeros(n, 1);
pitch_deg = zeros(n, 1);
roll_deg(1) = roll_acc(1);
pitch_deg(1) = pitch_acc(1);

for k = 2:n
    prev_roll = roll_deg(k - 1);
    prev_pitch = pitch_deg(k - 1);
    [roll_rate, pitch_rate] = gyro_to_roll_pitch_rate(gyro_dps(k, :), prev_roll, prev_pitch);

    acc_norm = norm(acc_filt(k, :));
    gyro_norm = norm(gyro_dps(k, :));

    r_measure = roll_acc(k);
    p_measure = pitch_acc(k);

    % Trust accelerometer less during strong linear acceleration or fast motion.
    r_scale = 1 + 300 * abs(acc_norm - 1) + 0.05 * gyro_norm;

    [roll_filter, roll_deg(k)] = update_angle_kalman(roll_filter, r_measure, roll_rate, dt, r_scale);
    [pitch_filter, pitch_deg(k)] = update_angle_kalman(pitch_filter, p_measure, pitch_rate, dt, r_scale);
end
end

function y = lowpass_vector(x, dt, cutoff_hz)
if cutoff_hz <= 0
    y = x;
    return;
end

tau = 1 / (2 * pi * cutoff_hz);
alpha = dt / (tau + dt);
y = zeros(size(x));
y(1, :) = x(1, :);

for k = 2:size(x, 1)
    y(k, :) = y(k - 1, :) + alpha * (x(k, :) - y(k - 1, :));
end
end

function [roll_rate, pitch_rate] = gyro_to_roll_pitch_rate(gyro_dps, roll_deg, pitch_deg)
gx = gyro_dps(1);
gy = gyro_dps(2);
gz = gyro_dps(3);
roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);
cos_pitch = cos(pitch);

if abs(cos_pitch) < 0.1
    cos_pitch = sign(cos_pitch) * 0.1;
    if cos_pitch == 0
        cos_pitch = 0.1;
    end
end

roll_rate = gx + sin(roll) * tan(pitch) * gy + cos(roll) * tan(pitch) * gz;
pitch_rate = cos(roll) * gy - sin(roll) * gz;

roll_rate = max(-720, min(720, roll_rate));
pitch_rate = max(-720, min(720, pitch_rate));
end

function filter = init_angle_kalman(initial_angle)
filter.angle = initial_angle;
filter.bias = 0;
filter.P = eye(2) * 0.01;
filter.Q_angle = 0.001;
filter.Q_bias = 0.003;
filter.R_measure = 0.08;
end

function [filter, angle] = update_angle_kalman(filter, measured_angle, measured_rate, dt, r_scale)
rate = measured_rate - filter.bias;
filter.angle = filter.angle + dt * rate;

F = [1, -dt; 0, 1];
Q = [filter.Q_angle, 0; 0, filter.Q_bias] * dt;
filter.P = F * filter.P * F' + Q;

H = [1, 0];
R = filter.R_measure * r_scale;
S = H * filter.P * H' + R;
K = filter.P * H' / S;

y = wrap_to_180(measured_angle - filter.angle);
x = [filter.angle; filter.bias] + K * y;
filter.angle = x(1);
filter.bias = x(2);

filter.P = (eye(2) - K * H) * filter.P;
angle = filter.angle;
end

function [roll_deg, pitch_deg, q] = mahony_stm32_roll_pitch(acc, gyro_dps, dt)
n = size(acc, 1);
q = zeros(n, 4);
roll_deg = zeros(n, 1);
pitch_deg = zeros(n, 1);

[roll0, pitch0] = accel_roll_pitch(acc(1:min(n, 32), :));
roll0 = median(roll0);
pitch0 = median(pitch0);
qk = quat_from_roll_pitch(deg2rad(roll0), deg2rad(pitch0));

nominalTwoKp = 2.0;
nominalTwoKi = 0.0;
twoKp = nominalTwoKp;
twoKi = nominalTwoKi;
integral = [0, 0, 0];
runtime_bias = [0, 0, 0];

accel_full_trust_delta_g = 0.05;
accel_zero_trust_delta_g = 0.25;
accel_gate_min_g = 0.85;
accel_gate_max_g = 1.15;
static_accel_tolerance_g = 0.05;
static_gyro_tolerance_dps = 1.5;
gyro_bias_adapt_alpha = 0.01;
static_tilt_recover_tau_s = 0.35;
dynamic_tilt_recover_tau_s = 1.20;

for k = 1:n
    ax = acc(k, 1);
    ay = acc(k, 2);
    az = acc(k, 3);

    if ~isfinite(ax), ax = 0; end
    if ~isfinite(ay), ay = 0; end
    if ~isfinite(az), az = 1; end

    gyro_sample = gyro_dps(k, :);
    gyro_sample(~isfinite(gyro_sample)) = 0;

    gx = deg2rad(gyro_sample(1) - runtime_bias(1));
    gy = deg2rad(gyro_sample(2) - runtime_bias(2));
    gz = deg2rad(gyro_sample(3) - runtime_bias(3));

    accel_norm_sq = ax * ax + ay * ay + az * az;
    accel_norm = sqrt(accel_norm_sq);
    accel_delta = abs(accel_norm - 1.0);
    gyro_mag_dps = norm(gyro_sample);

    if accel_delta <= accel_full_trust_delta_g
        accel_weight = 1.0;
    elseif accel_delta < accel_zero_trust_delta_g
        accel_weight = 1.0 - ((accel_delta - accel_full_trust_delta_g) / ...
                              (accel_zero_trust_delta_g - accel_full_trust_delta_g));
    else
        accel_weight = 0.0;
    end

    if nominalTwoKi > 0 && accel_delta <= static_accel_tolerance_g && gyro_mag_dps <= static_gyro_tolerance_dps
        runtime_bias = (1 - gyro_bias_adapt_alpha) * runtime_bias + gyro_bias_adapt_alpha * gyro_sample;
        gx = deg2rad(gyro_sample(1) - runtime_bias(1));
        gy = deg2rad(gyro_sample(2) - runtime_bias(2));
        gz = deg2rad(gyro_sample(3) - runtime_bias(3));
    end

    if accel_norm_sq > 0 && ...
       accel_norm >= accel_gate_min_g && ...
       accel_norm <= accel_gate_max_g && ...
       accel_weight > 0

        recip_norm = 1.0 / sqrt(accel_norm_sq);
        ax = ax * recip_norm;
        ay = ay * recip_norm;
        az = az * recip_norm;

        halfvx = qk(2) * qk(4) - qk(1) * qk(3);
        halfvy = qk(1) * qk(2) + qk(3) * qk(4);
        halfvz = qk(1) * qk(1) - 0.5 + qk(4) * qk(4);

        halfex = (ay * halfvz) - (az * halfvy);
        halfey = (az * halfvx) - (ax * halfvz);
        halfez = (ax * halfvy) - (ay * halfvx);
        halfex = halfex * accel_weight;
        halfey = halfey * accel_weight;
        halfez = halfez * accel_weight;

        if twoKi > 0 && accel_weight >= 0.2
            integral = integral + twoKi * [halfex, halfey, halfez] * dt;
            gx = gx + integral(1);
            gy = gy + integral(2);
            gz = gz + integral(3);
        else
            integral = [0, 0, 0];
        end

        gx = gx + twoKp * halfex;
        gy = gy + twoKp * halfey;
        gz = gz + twoKp * halfez;
    end

    gx = gx * 0.5 * dt;
    gy = gy * 0.5 * dt;
    gz = gz * 0.5 * dt;

    qa = qk(1);
    qb = qk(2);
    qc = qk(3);

    qk(1) = qk(1) + (-qb * gx - qc * gy - qk(4) * gz);
    qk(2) = qk(2) + ( qa * gx + qc * gz - qk(4) * gy);
    qk(3) = qk(3) + ( qa * gy - qb * gz + qk(4) * gx);
    qk(4) = qk(4) + ( qa * gz + qb * gy - qc * gx);
    qk = qk / norm(qk);

    if accel_norm_sq > 0 && ...
       accel_norm >= accel_gate_min_g && ...
       accel_norm <= accel_gate_max_g && ...
       accel_weight > 0

        [roll_q, pitch_q] = quat_to_roll_pitch(qk);
        [roll_acc_now, pitch_acc_now] = accel_roll_pitch(acc(k, :));
        roll_acc_now = align_scalar_angle(roll_acc_now, roll_q);
        pitch_acc_now = align_scalar_angle(pitch_acc_now, pitch_q);

        if gyro_mag_dps <= static_gyro_tolerance_dps
            recover_tau = static_tilt_recover_tau_s;
        else
            recover_tau = dynamic_tilt_recover_tau_s;
        end

        tilt_blend = (1 - exp(-dt / recover_tau)) * accel_weight;
        roll_q = roll_q + tilt_blend * wrap_to_180(roll_acc_now - roll_q);
        pitch_q = pitch_q + tilt_blend * wrap_to_180(pitch_acc_now - pitch_q);
        qk = quat_from_roll_pitch(deg2rad(roll_q), deg2rad(pitch_q));
    end

    q(k, :) = qk;
    [roll_deg(k), pitch_deg(k)] = quat_to_roll_pitch(qk);
end
end

function q = quat_from_roll_pitch(roll_rad, pitch_rad)
cr = cos(roll_rad / 2);
sr = sin(roll_rad / 2);
cp = cos(pitch_rad / 2);
sp = sin(pitch_rad / 2);
q = [cr * cp, sr * cp, cr * sp, -sr * sp];
q = q / norm(q);
end

function out = quat_multiply(a, b)
out = [ ...
    a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
    a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
    a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
    a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

function [roll_deg, pitch_deg] = quat_to_roll_pitch(q)
roll_rad = atan2(2 * (q(1)*q(2) + q(3)*q(4)), ...
                 1 - 2 * (q(2)^2 + q(3)^2));
sinp = 2 * (q(1)*q(3) - q(4)*q(2));
sinp = max(-1, min(1, sinp));
pitch_rad = asin(sinp);

roll_deg = rad2deg(roll_rad);
pitch_deg = rad2deg(pitch_rad);
end

function vertical_nT = vertical_projection(b_nT, roll_deg, pitch_deg)
roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);

vx = -sin(pitch);
vy = sin(roll) .* cos(pitch);
vz = cos(roll) .* cos(pitch);

vertical_nT = b_nT(:, 1) .* vx + b_nT(:, 2) .* vy + b_nT(:, 3) .* vz;
end

function vertical_nT = vertical_projection_optional(b_nT, roll_deg, pitch_deg)
vertical_nT = nan(size(b_nT, 1), 1);
valid = isfinite(roll_deg) & isfinite(pitch_deg);
if any(valid)
    vertical_nT(valid) = vertical_projection(b_nT(valid, :), roll_deg(valid), pitch_deg(valid));
end
end

function [R_mag_to_imu, euler_deg, b_aligned] = estimate_mag_to_imu_alignment(b_nT, roll_deg, pitch_deg)
% Estimate a pure rotation from magnetometer axes to IMU axes.
% The objective assumes the log was collected in a roughly uniform magnetic
% field while the board was rotated, so the vertical projection should vary
% as little as possible after the axes are aligned.
initial_guesses_deg = [
      0,   0,   0;
     90,   0,   0;
    -90,   0,   0;
      0,  90,   0;
      0, -90,   0;
      0,   0,  90;
      0,   0, -90;
    180,   0,   0;
      0, 180,   0;
      0,   0, 180];

best_score = Inf;
best_angles = [0, 0, 0];
options = optimset('Display', 'off', 'MaxIter', 1000, 'MaxFunEvals', 4000);

for i = 1:size(initial_guesses_deg, 1)
    x0 = deg2rad(initial_guesses_deg(i, :));
    objective = @(x) alignment_objective(x, b_nT, roll_deg, pitch_deg);
    [x, score] = fminsearch(objective, x0, options);
    if score < best_score
        best_score = score;
        best_angles = x;
    end
end

R_mag_to_imu = euler_xyz_to_rotation(best_angles);
euler_deg = rad2deg(wrap_to_pi(best_angles));
b_aligned = apply_rotation(b_nT, R_mag_to_imu);
end

function [b_calibrated, calibration] = calibrate_magnetic_axes(b_raw)
% Practical offline magnetic calibration:
% 1) estimate hard-iron bias by algebraic sphere fitting;
% 2) normalize axis scale with covariance whitening.
% This is intentionally toolbox-free and works as a first-pass diagnostic.
bias = estimate_sphere_bias(b_raw);
b_centered = bsxfun(@minus, b_raw, bias);

C = cov(b_centered);
[V, D] = eig(C);
d = diag(D);
d(d <= 0) = min(d(d > 0));
target_var = mean(d);
A = V * diag(sqrt(target_var ./ d)) * V';

b_calibrated = (A * b_centered')';

raw_norm = sqrt(sum(b_raw.^2, 2));
cal_norm = sqrt(sum(b_calibrated.^2, 2));
scale = median(raw_norm) / max(median(cal_norm), eps);
b_calibrated = b_calibrated * scale;
A = A * scale;

calibration = struct();
calibration.bias_nT = bias;
calibration.A = A;
calibration.raw_norm_std_nT = std(raw_norm);
calibration.calibrated_norm_std_nT = std(sqrt(sum(b_calibrated.^2, 2)));
end

function bias = estimate_sphere_bias(b_raw)
x = b_raw(:, 1);
y = b_raw(:, 2);
z = b_raw(:, 3);
M = [x, y, z, ones(size(x))];
rhs = -(x.^2 + y.^2 + z.^2);
p = M \ rhs;
bias = -0.5 * p(1:3).';

if any(~isfinite(bias))
    bias = median(b_raw, 1);
end
end

function [best_delay_s, roll_shifted, pitch_shifted] = optimize_attitude_delay(b_nT, roll_deg, pitch_deg, t)
if numel(t) < 5
    best_delay_s = 0;
    roll_shifted = roll_deg;
    pitch_shifted = pitch_deg;
    return;
end

delay_grid = -0.30:0.005:0.30;
best_score = Inf;
best_delay_s = 0;
roll_shifted = roll_deg;
pitch_shifted = pitch_deg;

for i = 1:numel(delay_grid)
    delay_s = delay_grid(i);
    r = interp1(t, roll_deg, t - delay_s, 'linear', 'extrap');
    p = interp1(t, pitch_deg, t - delay_s, 'linear', 'extrap');
    vertical = vertical_projection(b_nT, r, p);
    score = robust_spread(vertical);

    if score < best_score
        best_score = score;
        best_delay_s = delay_s;
        roll_shifted = r;
        pitch_shifted = p;
    end
end
end

function [roll_deg, pitch_deg, source_name] = select_deployable_attitude( ...
    stm32_roll_deg, stm32_pitch_deg, roll_fallback_deg, pitch_fallback_deg)
valid_stm32 = isfinite(stm32_roll_deg) & isfinite(stm32_pitch_deg);
if nnz(valid_stm32) >= max(5, 0.8 * numel(stm32_roll_deg))
    roll_deg = fill_missing_linear(stm32_roll_deg);
    pitch_deg = fill_missing_linear(stm32_pitch_deg);
    source_name = 'stm32 mahony';
else
    roll_deg = roll_fallback_deg;
    pitch_deg = pitch_fallback_deg;
    source_name = 'offline mahony';
end
end

function y = fill_missing_linear(x)
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

function [vertical_nlms, vertical_rls, vertical_final, model] = deployable_vertical_pipeline( ...
    b_nT, roll_deg, pitch_deg, vertical_nT, dt)
features = residual_features(b_nT, roll_deg, pitch_deg, vertical_nT);
trend = lowpass_vector(vertical_nT, dt, 0.20);
target_residual = vertical_nT - trend;

[vertical_nlms, nlms_model] = nlms_residual_compensate(vertical_nT, features, target_residual);
[vertical_rls, rls_model] = rls_residual_compensate(vertical_nT, features(:, 1:3), target_residual);
[vertical_rate_kf, rate_model] = vertical_rate_kalman_filter(vertical_rls, dt, 900.0, 180.0, 0.12);
vertical_final = vertical_rate_kf;

model = struct();
model.trend_nT = trend;
model.features = features;
model.nlms = nlms_model;
model.rls = rls_model;
model.vertical_rate_kalman_nT = vertical_rate_kf;
model.rate_kalman = rate_model;
end

function X = residual_features(b_nT, roll_deg, pitch_deg, vertical_nT)
roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);
v = [-sin(pitch), sin(roll) .* cos(pitch), cos(roll) .* cos(pitch)];
vertical_vec = bsxfun(@times, vertical_nT, v);
horizontal = b_nT - vertical_vec;

field_scale_nT = 50000;
hx = horizontal(:, 1) / field_scale_nT;
hy = horizontal(:, 2) / field_scale_nT;
hz = horizontal(:, 3) / field_scale_nT;
X = [hx, hy, hz, hx.^2, hy.^2, hz.^2, hx .* hy, hx .* hz, hy .* hz];
end

function [vertical_corrected, model] = nlms_residual_compensate(vertical_nT, X, target_residual)
n = size(X, 1);
p = size(X, 2);
w = zeros(p, 1);
residual_hat = zeros(n, 1);
mu = 0.08;
leak = 0.0001;
eps_norm = 0.05;

for k = 1:n
    x = X(k, :)';
    yhat = w' * x;
    e = target_residual(k) - yhat;
    residual_hat(k) = yhat;
    w = (1 - leak) * w + (mu * e / (eps_norm + x' * x)) * x;
    w = max(-5000, min(5000, w));
end

vertical_corrected = vertical_nT - residual_hat;
model.weights = w;
model.residual_hat_nT = residual_hat;
end

function [vertical_corrected, model] = rls_residual_compensate(vertical_nT, X, target_residual)
n = size(X, 1);
p = size(X, 2);
w = zeros(p, 1);
P = eye(p) * 20;
lambda = 0.999;
residual_hat = zeros(n, 1);

for k = 1:n
    x = X(k, :)';
    yhat = w' * x;
    e = target_residual(k) - yhat;
    Px = P * x;
    gain_den = lambda + x' * Px;
    K = Px / gain_den;
    residual_hat(k) = yhat;
    w = w + K * e;
    w = max(-2500, min(2500, w));
    P = (P - K * x' * P) / lambda;
end

vertical_corrected = vertical_nT - residual_hat;
model.weights = w;
model.P = P;
model.residual_hat_nT = residual_hat;
model.lambda = lambda;
end

function [y, model] = vertical_rate_kalman_filter(z, dt, process_accel_nT_s2, measurement_noise_nT, lead_s)
n = numel(z);
y = zeros(size(z));
rate = zeros(size(z));
state = [z(1); 0];
P = diag([measurement_noise_nT * measurement_noise_nT, 1000 * 1000]);
R = measurement_noise_nT * measurement_noise_nT;
q = process_accel_nT_s2 * process_accel_nT_s2;
F = [1, dt; 0, 1];
H = [1, 0];
Q = q * [0.25 * dt^4, 0.5 * dt^3; 0.5 * dt^3, dt^2];
y(1) = state(1);

for k = 2:n
    state = F * state;
    P = F * P * F' + Q;

    innovation = z(k) - H * state;
    S = H * P * H' + R;
    K = P * H' / S;
    state = state + K * innovation;
    P = (eye(2) - K * H) * P;

    rate(k) = state(2);
    y(k) = state(1) + lead_s * state(2);
end

model = struct();
model.rate_nT_per_s = rate;
model.process_accel_nT_s2 = process_accel_nT_s2;
model.measurement_noise_nT = measurement_noise_nT;
model.lead_s = lead_s;
end

function lag_s = estimate_signal_lag(reference, candidate, dt, max_lag_s)
reference = reference(:);
candidate = candidate(:);
valid = isfinite(reference) & isfinite(candidate);
reference = reference(valid);
candidate = candidate(valid);
if numel(reference) < 5
    lag_s = 0;
    return;
end

reference = reference - mean(reference);
candidate = candidate - mean(candidate);
max_lag = min(round(max_lag_s / dt), numel(reference) - 2);
best_score = -Inf;
best_lag = 0;

for lag = -max_lag:max_lag
    if lag < 0
        a = reference(1:end + lag);
        b = candidate(1 - lag:end);
    elseif lag > 0
        a = reference(1 + lag:end);
        b = candidate(1:end - lag);
    else
        a = reference;
        b = candidate;
    end

    denom = sqrt(sum(a.^2) * sum(b.^2));
    if denom > 0
        score = sum(a .* b) / denom;
        if score > best_score
            best_score = score;
            best_lag = lag;
        end
    end
end

lag_s = best_lag * dt;
end

function [vertical_corrected, model] = residual_regression_compensate(b_imu, roll_deg, pitch_deg, vertical_nT, dt)
roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);

v = [-sin(pitch), sin(roll) .* cos(pitch), cos(roll) .* cos(pitch)];
vertical_vec = bsxfun(@times, vertical_nT, v);
horizontal = b_imu - vertical_vec;
hx = horizontal(:, 1);
hy = horizontal(:, 2);
hz = horizontal(:, 3);

trend = lowpass_vector(vertical_nT, dt, 0.25);
residual = vertical_nT - trend;

X = [hx, hy, hz, hx.^2, hy.^2, hz.^2, hx .* hy, hx .* hz, hy .* hz];
X = normalize_columns(X);
lambda = 1e-3;
w = (X' * X + lambda * eye(size(X, 2))) \ (X' * residual);
residual_hat = X * w;
vertical_corrected = vertical_nT - residual_hat;

model = struct();
model.weights = w;
model.trend_nT = trend;
model.residual_hat_nT = residual_hat;
end

function Xn = normalize_columns(X)
mu = mean(X, 1);
sigma = std(X, 0, 1);
sigma(sigma < eps) = 1;
Xn = bsxfun(@rdivide, bsxfun(@minus, X, mu), sigma);
end

function spread = robust_spread(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    spread = Inf;
    return;
end
xc = x - median(x);
spread = median(abs(xc)) + 0.15 * std(xc);
end

function score = alignment_objective(angles_rad, b_nT, roll_deg, pitch_deg)
R = euler_xyz_to_rotation(angles_rad);
b_rot = apply_rotation(b_nT, R);
vertical = vertical_projection(b_rot, roll_deg, pitch_deg);
score = robust_spread(vertical);
end

function b_rot = apply_rotation(b_nT, R)
b_rot = (R * b_nT')';
end

function R = euler_xyz_to_rotation(angles_rad)
rx = angles_rad(1);
ry = angles_rad(2);
rz = angles_rad(3);

cx = cos(rx); sx = sin(rx);
cy = cos(ry); sy = sin(ry);
cz = cos(rz); sz = sin(rz);

Rx = [1, 0, 0; 0, cx, -sx; 0, sx, cx];
Ry = [cy, 0, sy; 0, 1, 0; -sy, 0, cy];
Rz = [cz, -sz, 0; sz, cz, 0; 0, 0, 1];

R = Rz * Ry * Rx;
end

function y = wrap_to_pi(x)
y = mod(x + pi, 2 * pi) - pi;
end

function y = wrap_to_180(x)
y = mod(x + 180, 360) - 180;
end

function aligned = align_angle_branch(angle_deg, reference_deg)
aligned = rad2deg(unwrap(deg2rad(angle_deg)));
reference = rad2deg(unwrap(deg2rad(reference_deg)));

offset_turns = round((median(reference - aligned)) / 360);
aligned = aligned + 360 * offset_turns;

for k = 1:numel(aligned)
    delta_turns = round((reference(k) - aligned(k)) / 360);
    aligned(k) = aligned(k) + 360 * delta_turns;
end
end

function aligned = align_optional_angle_branch(angle_deg, reference_deg)
aligned = angle_deg;
valid = isfinite(angle_deg);
if any(valid)
    aligned(valid) = align_angle_branch(angle_deg(valid), reference_deg(valid));
end
end

function angle_deg = align_scalar_angle(angle_deg, reference_deg)
angle_deg = reference_deg + wrap_to_180(angle_deg - reference_deg);
end

function plot_results(r)
output_dir = fullfile(pwd, 'fusion_figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fig = create_report_figure('Magnetic field');
plot(r.time_s, r.b_nT);
legend('b1', 'b2', 'b3', 'Location', 'best');
xlabel('Time (s)');
ylabel('nT');
title('Magnetic field');
format_report_axes(gca);
export_report_figure(fig, fullfile(output_dir, '01_magnetic_field.png'));

fig = create_report_figure('STM32 Mahony attitude');
subplot(2, 1, 1);
plot(r.time_s, r.stm32_roll_deg, 'Color', [0.0 0.45 0.74]);
format_report_axes(gca);
xlabel('s');
ylabel('deg');
title('STM32 Mahony roll');

subplot(2, 1, 2);
plot(r.time_s, r.stm32_pitch_deg, 'Color', [0.85 0.33 0.10]);
format_report_axes(gca);
xlabel('s');
ylabel('deg');
title('STM32 Mahony pitch');
export_report_figure(fig, fullfile(output_dir, '02_stm32_mahony_attitude.png'));

fig = create_report_figure('Vertical magnetic projection');
plot(r.time_s, r.vertical_kalman_nT, 'b'); hold on;
legend_entries = {'kalman'};
if any(isfinite(r.vertical_stm32_mahony_nT))
    plot(r.time_s, r.vertical_stm32_mahony_nT, 'Color', [0.0 0.55 0.0], 'LineStyle', '--');
    legend_entries{end + 1} = 'stm32 mahony'; %#ok<AGROW>
end
if any(isfinite(r.stm32_vertical_corr_nT))
    plot(r.time_s, r.stm32_vertical_corr_nT, 'Color', [0.45 0.0 0.65], 'LineStyle', '-');
    legend_entries{end + 1} = 'stm32 deployed'; %#ok<AGROW>
end
legend(legend_entries, 'Location', 'best');
xlabel('s');
ylabel('nT');
title('Vertical magnetic projection');
format_report_axes(gca);
export_report_figure(fig, fullfile(output_dir, '04_vertical_projection.png'));

fig = create_report_figure('B3 and STM32 vertical outputs');
plot(r.time_s, r.b_nT(:, 3), 'Color', [0.25 0.25 0.25]); hold on;
legend_entries = {'b3'};
if any(isfinite(r.vertical_stm32_mahony_nT))
    plot(r.time_s, r.vertical_stm32_mahony_nT, 'Color', [0.0 0.55 0.0], 'LineStyle', '--');
    legend_entries{end + 1} = 'stm32 mahony'; %#ok<AGROW>
end
if any(isfinite(r.stm32_vertical_corr_nT))
    plot(r.time_s, r.stm32_vertical_corr_nT, 'Color', [0.45 0.0 0.65], 'LineStyle', '-');
    legend_entries{end + 1} = 'stm32 deployed'; %#ok<AGROW>
end
legend(legend_entries, 'Location', 'best');
xlabel('s');
ylabel('nT');
title('B3 and STM32 vertical outputs');
format_report_axes(gca);
export_report_figure(fig, fullfile(output_dir, '03_b3_stm32_vertical.png'));

fprintf('\nSaved report figures to:\n  %s\n', output_dir);
end

function fig = create_report_figure(name)
fig = figure('Name', name, 'Color', 'w', 'Units', 'centimeters', ...
             'Position', [2, 2, 18, 11]);
set(fig, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 18, 11]);
end

function format_report_axes(ax)
grid(ax, 'on');
box(ax, 'on');
set(ax, 'FontName', 'Arial', 'FontSize', 12, 'LineWidth', 1.0);
lines = findobj(ax, 'Type', 'Line');
set(lines, 'LineWidth', 1.6);
legend_obj = findobj(ancestor(ax, 'figure'), 'Type', 'Legend');
if ~isempty(legend_obj)
    set(legend_obj, 'FontSize', 10, 'Box', 'off');
end
end

function export_report_figure(fig, file_path)
if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, file_path, 'Resolution', 300);
else
    print(fig, file_path, '-dpng', '-r300');
end
end

function print_summary(r)
acc_norm = sqrt(sum(r.acc_g.^2, 2));
gyro_norm = sqrt(sum(r.gyro_dps.^2, 2));
gyro_norm_br = sqrt(sum(r.gyro_dps_bias_removed.^2, 2));

fprintf('\nInput diagnostics:\n');
fprintf('  acc norm median: %.4f g, range: %.4f g\n', ...
    median(acc_norm), range(acc_norm));
fprintf('  gyro bias removed from first samples: [%.4f %.4f %.4f] deg/s\n', ...
    r.gyro_bias_dps(1), r.gyro_bias_dps(2), r.gyro_bias_dps(3));
fprintf('  gyro norm median before/after bias removal: %.4f / %.4f deg/s\n', ...
    median(gyro_norm), median(gyro_norm_br));
fprintf('  magnetic voltage calibration applied: %d\n', ...
    r.mag_voltage_calibration.input_was_voltage);
if any(isfinite(r.stm32_roll_deg))
    fprintf('  STM32 Mahony roll median/range: %.3f / %.3f deg\n', ...
        median(r.stm32_roll_deg(isfinite(r.stm32_roll_deg))), ...
        range(r.stm32_roll_deg(isfinite(r.stm32_roll_deg))));
    fprintf('  STM32 Mahony pitch median/range: %.3f / %.3f deg\n', ...
        median(r.stm32_pitch_deg(isfinite(r.stm32_pitch_deg))), ...
        range(r.stm32_pitch_deg(isfinite(r.stm32_pitch_deg))));
end
fprintf('\nMag-to-IMU alignment:\n');
fprintf('  Euler XYZ [deg]: [%.3f %.3f %.3f]\n', ...
    r.align_euler_deg(1), r.align_euler_deg(2), r.align_euler_deg(3));
fprintf('  R_mag_to_imu:\n');
fprintf('    %.8f  %.8f  %.8f\n', r.R_mag_to_imu(1, 1), r.R_mag_to_imu(1, 2), r.R_mag_to_imu(1, 3));
fprintf('    %.8f  %.8f  %.8f\n', r.R_mag_to_imu(2, 1), r.R_mag_to_imu(2, 2), r.R_mag_to_imu(2, 3));
fprintf('    %.8f  %.8f  %.8f\n', r.R_mag_to_imu(3, 1), r.R_mag_to_imu(3, 2), r.R_mag_to_imu(3, 3));
fprintf('\nEnhanced pipeline:\n');
fprintf('  magnetic bias [nT]: [%.3f %.3f %.3f]\n', ...
    r.mag_calibration.bias_nT(1), r.mag_calibration.bias_nT(2), r.mag_calibration.bias_nT(3));
fprintf('  |B| std raw/calibrated: %.3f / %.3f nT\n', ...
    r.mag_calibration.raw_norm_std_nT, r.mag_calibration.calibrated_norm_std_nT);
fprintf('  best attitude delay: %.1f ms\n', r.best_delay_s * 1000);
fprintf('  calibrated alignment Euler XYZ [deg]: [%.3f %.3f %.3f]\n', ...
    r.align_cal_euler_deg(1), r.align_cal_euler_deg(2), r.align_cal_euler_deg(3));

fprintf('\nVertical magnetic projection summary:\n');
fprintf('  accel-only std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_acc_nT), range(r.vertical_acc_nT));
fprintf('  kalman     std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_kalman_nT), range(r.vertical_kalman_nT));
fprintf('  mahony     std: %.3f nT, range: %.3f nT\n\n', ...
    std(r.vertical_mahony_nT), range(r.vertical_mahony_nT));
if any(isfinite(r.vertical_stm32_mahony_nT))
    valid_stm32_vertical = r.vertical_stm32_mahony_nT(isfinite(r.vertical_stm32_mahony_nT));
    fprintf('  stm32 mahony std: %.3f nT, range: %.3f nT\n\n', ...
        std(valid_stm32_vertical), range(valid_stm32_vertical));
end

fprintf('\nDeployable pipeline without magnetic A/bias or axis R:\n');
fprintf('  attitude source: %s\n', r.deploy_attitude_source);
fprintf('  best attitude delay: %.1f ms\n', r.deploy_delay_s * 1000);
fprintf('  delay only      std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_deploy_delay_nT), range(r.vertical_deploy_delay_nT));
fprintf('  delay + NLMS    std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_deploy_nlms_nT), range(r.vertical_deploy_nlms_nT));
fprintf('  delay + RLS     std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_deploy_rls_nT), range(r.vertical_deploy_rls_nT));
fprintf('  RLS + rate KF std: %.3f nT, range: %.3f nT\n', ...
    std(r.vertical_deploy_final_nT), range(r.vertical_deploy_final_nT));
fprintf('  final lag vs RLS estimate: %.1f ms\n', r.deploy_final_lag_s * 1000);
fprintf('  delay/final linear slope: %.3f / %.3f nT/s\n', ...
    linear_slope_nT_per_s(r.time_s, r.vertical_deploy_delay_nT), ...
    linear_slope_nT_per_s(r.time_s, r.vertical_deploy_final_nT));
if any(isfinite(r.stm32_vertical_corr_nT))
    valid_stm32_corr = r.stm32_vertical_corr_nT(isfinite(r.stm32_vertical_corr_nT));
    fprintf('  stm32 deployed std: %.3f nT, range: %.3f nT\n', ...
        std(valid_stm32_corr), range(valid_stm32_corr));
end
end

function slope = linear_slope_nT_per_s(t, x)
valid = isfinite(t) & isfinite(x);
if nnz(valid) < 2
    slope = NaN;
    return;
end
p = polyfit(t(valid), x(valid), 1);
slope = p(1);
end
