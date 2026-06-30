clear; clc; close all;

%% 1. Build symbolic model (3-Mass Nonlinear Oscillator)
syms z1 z2 z3 z4 z5 z6 F1_sym F2_sym real
z_sym = [z1; z2; z3; z4; z5; z6];
F_sym = [F1_sym; F2_sym]; % Two control inputs

% System parameters
m1 = 1.0; 
m2 = 0.5; 
m3 = 10.0; 
k1 = 5.0; % Spring 1 stiffness (between m1 & m2)
k2 = 3.0; % Spring 2 stiffness (between m2 & m3)
c1 = 0.2; % Damping 1
c2 = 0.3; % Damping 2

% Nonlinear forces between the masses (Pure cubic springs)
F_spring1 = k1 * (z1 - z3)^3 + c1 * (z2 - z4);
F_spring2 = k2 * (z3 - z5)^3 + c2 * (z4 - z6);

% Dynamics (F1 applied to m1, F2 applied to m3)
dz_sym = [
    z2;
    (-F_spring1 + F1_sym) / m1;
    z4;
    (F_spring1 - F_spring2) / m2;
    z6;
    (F_spring2 + F2_sym) / m3
];

%% 2. Solve symbolic Jacobian matrices
disp('Calculating Jacobian matrices...');
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

disp('Generating dynamic functions...');
A_func = matlabFunction(A_sym, 'Vars', {z_sym});
B_func = matlabFunction(B_sym, 'Vars', {z_sym});
model_func = matlabFunction(dz_sym, 'Vars', {z_sym, F_sym});

%% 3. Design Controllers
% Shared Controller parameters
Kp = [10, 10, 10];  % Proportional gains for m1, m2, m3
Kd = [5, 5, 5];     % Derivative gains for m1, m2, m3

% --- Controller 1: Coupleness Controller ---
gamma = 0.1;               
W = [1.0, 0.1, 1.0, 0.1, 1.0, 0.1];  
coupleness_controller = @(t, z) calc_coupleness_control(z, A_func, B_func, gamma, Kp, Kd, W);

% --- Controller 2: Standard Collocated PD Controller ---
pd_controller = @(t, z) calc_pd_control(z, Kp, Kd);

%% 4. Run Fixed-Step Simulations
disp('Starting fixed-step Euler simulations...');
z0 = [0.5; 0; 0.2; 0; -0.5; 0]; 
T_end = 10; 
dt = 0.001; 

% Run Coupleness Simulation
disp('Running Coupleness Controller...');
[t_c, z_c] = timeConstantSimu(z0, T_end, dt, model_func, coupleness_controller);

% Run PD Simulation
disp('Running Pure PD Controller...');
[t_pd, z_pd] = timeConstantSimu(z0, T_end, dt, model_func, pd_controller);

% Recalculate Forces for plotting
F_c = zeros(length(t_c), 2);
F_pd = zeros(length(t_pd), 2);
for i = 1:length(t_c)
    [F_c(i,:), ~, ~] = coupleness_controller(t_c(i), z_c(i,:)');
    [F_pd(i,:), ~, ~] = pd_controller(t_pd(i), z_pd(i,:)');
end

%% 5. Plot Results (Side-by-Side Comparison)
disp('Plotting comparison results...');
figure('Position', [100, 100, 1200, 600]);

% --- Coupleness: Positions ---
subplot(2,2,1);
plot(t_c, z_c(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_c, z_c(:,3), 'r', 'LineWidth', 1.5);
plot(t_c, z_c(:,5), 'k', 'LineWidth', 1.5);
ylabel('Positions (m)');
title('Coupleness Control (No Delta, Pure Pseudo-Inverse)');
legend('Mass 1 (z_1)', 'Mass 2 (z_3)', 'Mass 3 (z_5)', 'Location', 'best');
grid on;

% --- Pure PD: Positions ---
subplot(2,2,2);
plot(t_pd, z_pd(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_pd, z_pd(:,3), 'r', 'LineWidth', 1.5);
plot(t_pd, z_pd(:,5), 'k', 'LineWidth', 1.5);
ylabel('Positions (m)');
title('Pure PD Control (Local Collocated Only)');
legend('Mass 1 (z_1)', 'Mass 2 (z_3)', 'Mass 3 (z_5)', 'Location', 'best');
grid on;

% --- Coupleness: Forces ---
subplot(2,2,3);
plot(t_c, F_c(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_c, F_c(:,2), 'k', 'LineWidth', 1.5);
ylabel('Force F (N)');
xlabel('Time (s)');
legend('F_1 (on Mass 1)', 'F_2 (on Mass 3)', 'Location', 'best');
grid on;

% --- Pure PD: Forces ---
subplot(2,2,4);
plot(t_pd, F_pd(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_pd, F_pd(:,2), 'k', 'LineWidth', 1.5);
ylabel('Force F (N)');
xlabel('Time (s)');
legend('F_1 (on Mass 1)', 'F_2 (on Mass 3)', 'Location', 'best');
grid on;

%% ================== Helper Functions ================== %%

% Custom Fixed-Step Simulator
function [t, Z] = timeConstantSimu(z0, T_end, dt, model, controller)
    t = 0:dt:T_end;
    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';
    for k = 1:length(t)-1
        zk = Z(k, :)';
        [F, ~, ~] = controller(t(k), zk);
        dz = model(zk, F(:));
        zk_next = zk + dt * dz;
        Z(k+1, :) = zk_next';
    end
end

% Core Controller Implementation: Coupleness (Optimized, Delta Removed)
function [F, priorities, cu_norm] = calc_coupleness_control(z, A_func, B_func, gamma, Kp, Kd, W)
    A = A_func(z);
    B = B_func(z); 
    
    spectral_radius = max(abs(eig(A)));
    if gamma * spectral_radius >= 1
        gamma = 0.99 / spectral_radius; 
    end
    
    I = eye(6);
    Cz = (I - gamma * A) \ I - I; 
    Cu = gamma * (Cz + I) * B; 
    
    scores = zeros(3, 1);
    G_val = W(:) .* abs(z); 
    for m = 1:3
        idx = (m-1)*2 + 1; 
        scores(m) = sum(abs(Cz(idx, :))' .* G_val);
    end
    
    [~, sorted_idx] = sort(scores, 'descend');
    priorities = sorted_idx(1:2); 
    
    v = zeros(2, 1);
    state_idx = zeros(2, 1);
    for i = 1:2
        m = priorities(i);
        s_idx = (m-1)*2 + 1; 
        state_idx(i) = s_idx;
        v(i) = -Kp(m) * z(s_idx) - Kd(m) * z(s_idx+1); 
    end
    
    Cu_sel = Cu(state_idx, :);
    cu_norm = norm(Cu_sel);
    
    F = pinv(Cu_sel) * v;
end

% Core Controller Implementation: Pure PD (Baseline)
function [F, priorities, cu_norm] = calc_pd_control(z, Kp, Kd)
    F1 = -Kp(1) * z(1) - Kd(1) * z(2);
    F2 = -Kp(3) * z(5) - Kd(3) * z(6);
    F = [F1; F2];
    priorities = [1; 3]; 
    cu_norm = 0;
end