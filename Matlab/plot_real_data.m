clc
clear
close all

%% Select measured logs
project_log_dir = '/Users/a123/Documents/Unicycle/unicycle_project_umich/BLE/LOGS/ValidLogs';

mode_name = 'position';  % 'position' or 'velocity'
target_value = 1.0;      % 1.0 for position, 0.5 for velocity
time_limit = 12.5;

switch lower(mode_name)
    case 'position'
        file_paths = {
            fullfile(project_log_dir, 'P1.0x3.csv')
            fullfile(project_log_dir, 'P1.0x3(2).csv')
        };
        min_position_range = 0.05;
        min_velocity_peak = 0.02;
    case 'velocity'
        file_paths = {
            fullfile(project_log_dir, 'V0.5x3.csv')
            fullfile(project_log_dir, 'V0.5x3(2).csv')
        };
        min_position_range = 1.0;
        min_velocity_peak = 0.05;
    otherwise
        error('plot_real_data:BadMode', ...
            'mode_name must be position or velocity.');
end

segments = read_real_data( ...
    file_paths, ...
    mode_name, ...
    target_value, ...
    'TimeLimit', time_limit, ...
    'ZeroSignals', true, ...
    'MinPositionRange', min_position_range, ...
    'MinVelocityPeak', min_velocity_peak);

fprintf('Loaded %d measured %s segment(s), target %.3g.\n', ...
    numel(segments), mode_name, target_value);

%% Plot measured data
figure

subplot(4, 1, 1)
box on
hold on
grid on
ylabel('$x \, (\mathrm{m})$', 'Interpreter', 'latex')
for i = 1:numel(segments)
    plot(segments(i).time, segments(i).position, 'LineWidth', 1.1)
end
if strcmpi(mode_name, 'position')
    plot([0 time_limit], [target_value target_value], ':', 'LineWidth', 1.2)
end
title(sprintf('Measured %s command intervals', mode_name))
xlim([0 time_limit])

subplot(4, 1, 2)
box on
hold on
grid on
ylabel('$\dot{x} \, (\mathrm{m/s})$', 'Interpreter', 'latex')
for i = 1:numel(segments)
    plot(segments(i).time, segments(i).velocity, 'LineWidth', 1.1)
end
if strcmpi(mode_name, 'velocity')
    plot([0 time_limit], [target_value target_value], ':', 'LineWidth', 1.2)
else
    plot([0 time_limit], [0 0], ':', 'LineWidth', 1.2)
end
xlim([0 time_limit])

subplot(4, 1, 3)
box on
hold on
grid on
ylabel('$\gamma \, (\mathrm{deg})$', 'Interpreter', 'latex')
for i = 1:numel(segments)
    plot(segments(i).time, segments(i).gamma_deg, 'LineWidth', 1.1)
end
plot([0 time_limit], [0 0], ':', 'LineWidth', 1.2)
xlim([0 time_limit])

subplot(4, 1, 4)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{\gamma} \, (\mathrm{deg/s})$', 'Interpreter', 'latex')
for i = 1:numel(segments)
    plot(segments(i).time, segments(i).dgamma_degps, 'LineWidth', 1.1)
end
plot([0 time_limit], [0 0], ':', 'LineWidth', 1.2)
xlim([0 time_limit])
