clc
clear
close all
clear segway_controller segway_rolling_resistance_model

%% Parameters
% Values mirror the latest Python-vs-MuJoCo comparison in
% Derivation/Segway/Tuner.ipynb.
scenario = 'position'; % 'velocity' or 'position'
Tend = 12.5;

par.m = 0.7443479658510976;          % pendulum mass (kg)
par.m_w = 6.555652034148903;       % wheel mass (kg)
par.h = 0.115;        % pendulum COM height from wheel COM (m)
par.R = 0.2527;       % wheel radius (m)
par.I = 0.104;        % wheel inertia used by the latest comparison (kg*m^2)
par.g = 9.81;         % gravity (m/s^2)

% Controller gains from Derivation/Segway/Parameters.py.
par.K_gamma = 3.0;
par.K_dgamma = 0.8;
par.K_velocity = 3.3;
par.K_position = 2.0;
par.K_vi = 0.0;
par.K_pi = 0.00;
par.posDeadZone = 0.005;
par.clip_integral = 1.0;

switch lower(scenario)
    case 'velocity'
        % Match Tuner.ipynb latest velocity Python simulation.
        par.B = 0.29534719835953205;
        par.B_0 = 0.012007420032393943;
        par.mu_rolling = 0.010143346358048193;
        par.smooth_factor = 1.2;
        par.control_mode = 'velocity';
        par.control_strategy = 'pd';
        par.desired_gamma = 0.0;
        par.desired_velocity = 0.5;
        par.desired_position = 0.0;
    case 'position'
        % Match Tuner.ipynb latest position Python simulation.
        par.B = 0.29534719835953205;
        par.B_0 = 0.012007420032393943;
        par.mu_rolling = 0.010143346358048193;
        par.smooth_factor = 1.2;
        par.control_mode = 'position';
        par.control_strategy = 'pd';
        par.desired_gamma = 0.0;
        par.desired_velocity = 0.0;
        par.desired_position = 1.0;
    otherwise
        error('simulation:BadScenario', ...
            'scenario must be velocity or position.');
end

fprintf('Running simulation file: %s\n', which('simulation'));
fprintf(['scenario=%s, Tend=%.3g, K_velocity=%.3g, smooth_factor=%.3g, ', ...
    'B=%.6g, B_0=%.6g, mu=%.6g\n'], ...
    scenario, Tend, par.K_velocity, par.smooth_factor, ...
    par.B, par.B_0, par.mu_rolling);

%% Initial conditions
% z = [x; x_dot; gamma; gamma_dot]
z0 = [0.0; 0.0; 0.0; 0.0];

%% Simulation
t_eval = linspace(0, Tend, 1000);
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);

[t, Z] = ode45(@(t, z) segway_rolling_resistance_model( ...
    t, z, par, @segway_controller), t_eval, z0, options);

X = Z(:, 1);
X_dot = Z(:, 2);
gamma = Z(:, 3);
gamma_dot = Z(:, 4);

fprintf(['MATLAB result: final x=%.6g, peak x=%.6g, peak x_dot=%.6g, ', ...
    'peak gamma=%.6g deg, peak gamma_dot=%.6g deg/s\n'], ...
    X(end), max(X), max(X_dot), max(gamma*180/pi), ...
    max(gamma_dot*180/pi));
fprintf(['Python latest reference should be approximately: final x=0.9837, ', ...
    'peak x=1.1095, peak x_dot=0.2893, peak gamma=40.14 deg, ', ...
    'peak gamma_dot=100.95 deg/s.\n']);

%% Plotting
figure(1)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$x \, (\mathrm{m})$', 'Interpreter', 'latex')
plot(t, X, 'LineWidth', 1.5)
xlim([0 Tend])
if strcmpi(par.control_mode, 'position')
    plot(t, par.desired_position*ones(size(t)), ':', 'LineWidth', 1.2)
end
if strcmpi(par.control_mode, 'position')
    legend('$x$', '$x_d$', 'Interpreter', 'latex', 'Location', 'best')
else
    legend('$x$', 'Interpreter', 'latex', 'Location', 'best')
end
title('Longitudinal Segway Rolling Resistance Model')

figure(2)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\gamma \, (\mathrm{deg})$', 'Interpreter', 'latex')
plot(t, gamma*180/pi, 'LineWidth', 1.5)
plot(t, par.desired_gamma*180/pi*ones(size(t)), ':', 'LineWidth', 1.2)
xlim([0 Tend])
legend('$\gamma$', '$\gamma_d$', 'Interpreter', 'latex', 'Location', 'best')

figure(3)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{\gamma} \, (\mathrm{deg/s})$', 'Interpreter', 'latex')
plot(t, gamma_dot*180/pi, 'LineWidth', 1.5)
xlim([0 Tend])
legend('$\dot{\gamma}$', ...
    'Interpreter', 'latex', 'Location', 'best')

figure(2)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{x} \, (\mathrm{m/s})$', 'Interpreter', 'latex')
plot(t, X_dot, 'LineWidth', 1.5)
plot(t, par.desired_velocity*ones(size(t)), ':', 'LineWidth', 1.2)
xlim([0 Tend])
legend('$\dot{x}$', '$\dot{x}_d$', ...
    'Interpreter', 'latex', 'Location', 'best')
