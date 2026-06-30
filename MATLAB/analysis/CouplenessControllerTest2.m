clear; clc; close all;

%% 1. Build symbolic model (5-Mass Mixed Nonlinear Oscillator)
syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 F1_sym F2_sym real
z_sym = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10];
F_sym = [F1_sym; F2_sym]; % Two control inputs on edges

% System parameters
m1 = 1.0; m2 = 0.8; m3 = 2.0; m4 = 0.8; m5 = 1.0;
k_cub = 8.0;  % Cubic spring coefficient
k_sin = 5.0;  % Sinusoidal spring coefficient
c = 0.2;      % Uniform damping

% Complex Mixed Nonlinear Forces
F12 = k_cub * (z1 - z3)^3 + c * (z2 - z4);
F23 = k_sin * sin(z3 - z5) + c * (z4 - z6);
F34 = k_cub * (z5 - z7)^3 + c * (z6 - z8);
F45 = k_sin * sin(z7 - z9) + c * (z8 - z10);

% Dynamics (F1 on m1, F2 on m5)
dz_sym = [
    z2;
    (-F12 + F1_sym) / m1;
    z4;
    (F12 - F23) / m2;
    z6;
    (F23 - F34) / m3;    % Deepest node, totally unactuated
    z8;
    (F34 - F45) / m4;
    z10;
    (F45 + F2_sym) / m5
];

%% 2. Solve symbolic Jacobian matrices
disp('Calculating 10x10 and 10x2 Jacobian matrices...');
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

disp('Generating dynamic functions...');
A_func = matlabFunction(A_sym, 'Vars', {z_sym});
B_func = matlabFunction(B_sym, 'Vars', {z_sym});
model_func = matlabFunction(dz_sym, 'Vars', {z_sym, F_sym});

%% 3. Design Controllers
Kp = [15, 15, 20, 15, 15]; 
Kd = [5, 5, 8, 5, 5];

% --- Smooth Coupleness Controller (No Delta, SVD Protected & Actuator Saturated) ---
gamma = 0.05; 
W = ones(1, 10);     % Uniform weights to prevent state attention monopolization
alpha_softmax = 0.0; % Lower intensity to allow smooth resource distribution among edge nodes
smooth_coupleness_controller = @(t, z) calc_smooth_coupleness_control_5mass(z, A_func, B_func, gamma, Kp, Kd, W, alpha_softmax);

% --- Standard Collocated PD Controller (Baseline) ---
pd_controller = @(t, z) calc_pd_control_5mass(z, Kp, Kd);

%% 4. Run Fixed-Step Simulations
disp('Starting fixed-step simulations...');
z0 = [2.0; 0;  1.5; 0;  -0.8; 0;  -0.35; 0;  -1.0; 0]; 
T_end = 40; 
dt = 0.001; 

disp('Running Smooth Coupleness Controller...');
[t_c, z_c] = timeConstantSimu(z0, T_end, dt, model_func, smooth_coupleness_controller);

disp('Running Pure PD Controller...');
[t_pd, z_pd] = timeConstantSimu(z0, T_end, dt, model_func, pd_controller);

% Extract metrics for Smooth Coupleness Control
F_c = zeros(length(t_c), 2);
Priority_out = zeros(length(t_c), 2);
for i = 1:length(t_c)
    [F_c(i,:), Priority_out(i,:), ~] = smooth_coupleness_controller(t_c(i), z_c(i,:)');
end

%% 5. Plot Results
disp('Plotting comparison results...');
figure('Position', [50, 50, 1400, 800]);

% 1. Coupleness Position Tracking
subplot(2,3,1);
plot(t_c, z_c(:,1), 'LineWidth', 1.5); hold on;
plot(t_c, z_c(:,3), 'LineWidth', 1.5);
plot(t_c, z_c(:,5), 'k', 'LineWidth', 2); % Mass 3 (Center)
plot(t_c, z_c(:,7), 'LineWidth', 1.5);
plot(t_c, z_c(:,9), 'LineWidth', 1.5);
ylabel('Positions (m)'); title('Smooth Coupleness: Active Convergence (No Delta)');
legend('m_1', 'm_2', 'm_3 (Center)', 'm_4', 'm_5', 'Location', 'best'); grid on;

% 2. PD Position Tracking
subplot(2,3,2);
plot(t_pd, z_pd(:,1), 'LineWidth', 1.5); hold on;
plot(t_pd, z_pd(:,3), 'LineWidth', 1.5);
plot(t_pd, z_pd(:,5), 'k', 'LineWidth', 2);
plot(t_pd, z_pd(:,7), 'LineWidth', 1.5);
plot(t_pd, z_pd(:,9), 'LineWidth', 1.5);
ylabel('Positions (m)'); title('PD: Slow Passive Dissipation'); grid on;

% 3. Coupleness Forces
subplot(2,3,3);
plot(t_c, F_c(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_c, F_c(:,2), 'r', 'LineWidth', 1.5);
ylabel('Force (N)'); title('Smooth Coupleness Control Forces');
legend('F_1 (on m_1)', 'F_2 (on m_5)'); grid on;

% 4. Attention Focus (Highest Score States)
subplot(2,3,[4,5,6]);
plot(t_c, Priority_out(:,1), 'g.', 'MarkerSize', 6); hold on;
plot(t_c, Priority_out(:,2), 'm.', 'MarkerSize', 6);
yticks(1:5); yticklabels({'m_1', 'm_2', 'm_3', 'm_4', 'm_5'});
ylabel('Top Attention Targets'); xlabel('Time (s)');
title('Continuous Resource Allocation Focus');
legend('Highest Score', '2nd Highest Score'); grid on;

%% ================== Helper Functions ================== %%

function [t, Z] = timeConstantSimu(z0, T_end, dt, model, controller)
    t = 0:dt:T_end;
    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';
    for k = 1:length(t)-1
        zk = Z(k, :)';
        [F, ~, ~] = controller(t(k), zk);
        Z(k+1, :) = (zk + dt * model(zk, F(:)))';
    end
end

% Core Controller Implementation: Smooth Coupleness (Delta Removed, Protected by SVD Truncation & Saturation)
function [F, priorities, cu_norm] = calc_smooth_coupleness_control_5mass(z, A_func, B_func, gamma, Kp, Kd, W, alpha)
    % Safety guard: Check if numerical integration has already exploded to NaN
    if any(isnan(z)) || any(isinf(z))
        F = [0; 0]; priorities = [1; 5]; cu_norm = 0; return;
    end

    A = A_func(z);
    B = B_func(z); 
    
    spectral_radius = max(abs(eig(A)));
    if gamma * spectral_radius >= 1
        gamma = 0.99 / spectral_radius; 
    end
    
    % Upgrade 1: Use pinv for global network matrix calculation to prevent early singularity warnings
    I = eye(10);
    Cz = pinv(I - gamma * A) - I; 
    Cu = gamma * (Cz + I) * B; 
    
    % 1. Calculate scores for all 5 masses
    scores = zeros(5, 1);
    G_val = W(:) .* abs(z); 
    for m = 1:5
        idx = (m-1)*2 + 1; 
        scores(m) = sum(abs(Cz(idx, :))' .* G_val);
    end
    
    % 2. Softmax weight transformation
    shifted_scores = alpha * (scores - max(scores)); 
    weights = exp(shifted_scores) / sum(exp(shifted_scores));
    W_soft = diag(weights); 
    
    % 3. Extract position-level Coupleness matrix (5x2)
    pos_indices = [1, 3, 5, 7, 9];
    Cu_pos = Cu(pos_indices, :);
    
    % 4. Collect full virtual control demands (5x1)
    v_all = zeros(5, 1);
    for m = 1:5
        s_idx = (m-1)*2 + 1;
        v_all(m) = -Kp(m) * z(s_idx) - Kd(m) * z(s_idx+1); 
    end
    
% 5. --- UPGRADED: Boundary Layer Smooth Pseudo-Inverse (Eliminates Red Chattering Block) ---
    W_sqrt = sqrt(W_soft);
    M = W_sqrt * Cu_pos;    % Target weighted matrix
    y = W_sqrt * v_all;     % Target weighted virtual control vector
    
    [U, S, V] = svd(M, 'econ');
    s_vals = diag(S);
    
    % Define a smooth boundary layer epsilon. 
    % Singular values below this boundary will be smoothly regularized instead of hard cut-off.
    epsilon = 1e-3; 
    
    s_inv = zeros(size(s_vals));
    for i = 1:length(s_vals)
        if s_vals(i) >= epsilon
            % Outside the boundary layer: Pure aggressive inverse for maximum network authority
            s_inv(i) = 1 / s_vals(i);
        else
            % Inside the boundary layer: Smoothly taper down the gain using Tikhonov-like smoothing
            % This prevents the gain from blowing up to infinity and cures the actuator chattering.
            s_inv(i) = s_vals(i) / (s_vals(i)^2 + epsilon^2);
        end
    end
    
    % Compute the smooth and beautiful control input F
    F = V * diag(s_inv) * (U' * y);
    
    % Actuator Physical Saturation Limiter (Now acts only as a safety jacket, not a constant wall)
    max_force = 1000000; 
    F = max(min(F, max_force), -max_force);

    
    % Outputs for legacy plotting compatibility
    [~, sorted_idx] = sort(scores, 'descend');
    priorities = sorted_idx(1:2); 
    cu_norm = norm(Cu_pos);
end

function [F, priorities, cu_norm] = calc_pd_control_5mass(z, Kp, Kd)
    F1 = -Kp(1) * z(1) - Kd(1) * z(2);
    F2 = -Kp(5) * z(9) - Kd(5) * z(10);
    F = [F1; F2];
    
    % Upgrade 3: Keep baseline PD bounded as well for fair comparison
    max_force = 10000;
    F = max(min(F, max_force), -max_force);
    
    priorities = [1; 5]; 
    cu_norm = 0;
end