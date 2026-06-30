clear; clc; close all;

%% 1. Build symbolic model (Standard Stable 5-Mass)
syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 F1_sym F2_sym real
z_sym = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10];
F_sym = [F1_sym; F2_sym]; 

m1 = 1.0; m2 = 0.8; m3 = 2.0; m4 = 0.8; m5 = 10.0;
k_cub = 8.0;  
k_sin = 5.0;  
c = 0.2;      

F12 = k_cub * (z1 - z3)^3 + c * (z2 - z4);
F23 = k_sin * sin(z3 - z5) + c * (z4 - z6);
F34 = k_cub * (z5 - z7)^3 + c * (z6 - z8);
F45 = k_sin * sin(z7 - z9) + c * (z8 - z10);

dz_sym = [
    z2;
    (-F12 + F1_sym) / m1;
    z4;
    (F12 - F23) / m2;
    z6;
    (F23 - F34) / m3; % Disturbance will be injected dynamically here during simulation
    z8;
    (F34 - F45) / m4;
    z10;
    (F45 + F2_sym) / m5
];

%% 2. Solve symbolic Jacobian matrices
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_func = matlabFunction(A_sym, 'Vars', {z_sym});
B_func = matlabFunction(B_sym, 'Vars', {z_sym});
model_func = matlabFunction(dz_sym, 'Vars', {z_sym, F_sym});

%% 3. Design Controllers
Kp = [15, 15, 20, 15, 15]; 
Kd = [5, 5, 8, 5, 5];

% --- Coupleness Tracking Error Controller ---
gamma = 0.05; 
W = ones(1, 10);     
alpha_softmax = 2.0; 
K_cu_error = 2.5; 

smooth_coupleness_controller = @(t, z) calc_coupleness_error_control_5mass(z, A_func, B_func, gamma, Kp, Kd, W, alpha_softmax, K_cu_error);
pd_controller = @(t, z) calc_pd_control_5mass(z, Kp, Kd);

%% 4. Run Fixed-Step Simulations with Hammer Disturbance
disp('Starting simulations with mid-node disturbance...');
z0 = zeros(10, 1); % Start completely at rest
T_end = 20; 
dt = 0.002; 

[t_c, z_c] = timeConstantSimu(z0, T_end, dt, model_func, smooth_coupleness_controller, true);
[t_pd, z_pd] = timeConstantSimu(z0, T_end, dt, model_func, pd_controller, true);

%% 5. Plot Results
figure('Position', [50, 50, 1400, 500]);

% 1. Coupleness Position Tracking
subplot(1,2,1);
plot(t_c, z_c(:,1), 'LineWidth', 1.5); hold on;
plot(t_c, z_c(:,3), 'LineWidth', 1.5);
plot(t_c, z_c(:,5), 'k', 'LineWidth', 2); % Disturbed Mass
plot(t_c, z_c(:,7), 'LineWidth', 1.5);
plot(t_c, z_c(:,9), 'LineWidth', 1.5);
xline(5.0, 'r--', 'Hammer Impact', 'LabelVerticalAlignment', 'bottom');
ylabel('Positions (m)'); title('Coupleness: Active Instant Suppression');
legend('m_1', 'm_2', 'm_3 (Hammered)', 'm_4', 'm_5', 'Location', 'best'); grid on;
ylim([-1, 1]);

% 2. PD Position Tracking
subplot(1,2,2);
plot(t_pd, z_pd(:,1), 'LineWidth', 1.5); hold on;
plot(t_pd, z_pd(:,3), 'LineWidth', 1.5);
plot(t_pd, z_pd(:,5), 'k', 'LineWidth', 2);
plot(t_pd, z_pd(:,7), 'LineWidth', 1.5);
plot(t_pd, z_pd(:,9), 'LineWidth', 1.5);
xline(5.0, 'r--', 'Hammer Impact', 'LabelVerticalAlignment', 'bottom');
ylabel('Positions (m)'); title('PD: Sluggish Delayed Reaction'); grid on;
ylim([-1, 1]);

%% ================== Helper Functions ================== %%

function [t, Z] = timeConstantSimu(z0, T_end, dt, model, controller, apply_dist)
    t = 0:dt:T_end;
    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';
    
    for k = 1:length(t)-1
        zk = Z(k, :)';
        time = t(k);
        
        [F, ~, ~] = controller(time, zk);
        
        % Calculate nominal dynamics
        dz = model(zk, F(:));
        
        % --- THE HAMMER IMPACT ON M3 ---
        % Inject a massive 50N force on the 3rd mass (6th index in dz is m3 velocity derivative)
        if apply_dist && time >= 5.0 && time <= 5.1
            dz(6) = dz(6) + 50.0 / 2.0; % Force / m3
        end
        
        Z(k+1, :) = (zk + dt * dz)';
    end
end

function [F, priorities, cu_norm] = calc_coupleness_error_control_5mass(z, A_func, B_func, gamma, Kp, Kd, W, alpha, K_cu)
    A = A_func(z); B = B_func(z); 
    spectral_radius = max(abs(eig(A)));
    if gamma * spectral_radius >= 1
        gamma = 0.99 / spectral_radius; 
    end
    I = eye(10);
    Cz = pinv(I - gamma * A) - I; 
    Cu = gamma * (Cz + I) * B; 
    
    scores = zeros(5, 1);
    G_val = W(:) .* abs(z); 
    for m = 1:5
        idx = (m-1)*2 + 1; 
        scores(m) = sum(abs(Cz(idx, :))' .* G_val);
    end
    shifted_scores = alpha * (scores - max(scores)); 
    weights = exp(shifted_scores) / sum(exp(shifted_scores));
    W_soft = diag(weights); 
    
    pos_indices = [1, 3, 5, 7, 9];
    Cu_pos = Cu(pos_indices, :);
    
    v_error = zeros(5, 1);
    for m = 1:5
        s_idx = (m-1)*2 + 1;
        v_error(m) = -Kp(m) * z(s_idx) - Kd(m) * z(s_idx+1); 
    end
    
    W_sqrt = sqrt(W_soft);
    M = W_sqrt * Cu_pos;               
    y = W_sqrt * (K_cu * v_error);     
    
    [U, S, V] = svd(M, 'econ');
    s_vals = diag(S);
    epsilon = 5e-3; 
    
    s_inv = zeros(size(s_vals));
    for i = 1:length(s_vals)
        if s_vals(i) >= epsilon
            s_inv(i) = 1 / s_vals(i); 
        else
            s_inv(i) = s_vals(i) / (s_vals(i)^2 + epsilon^2); 
        end
    end
    F = V * diag(s_inv) * (U' * y);
    max_force = 10000; 
    F = max(min(F, max_force), -max_force);
    
    [~, sorted_idx] = sort(scores, 'descend');
    priorities = sorted_idx(1:2); 
    cu_norm = norm(Cu_pos);
end

function [F, priorities, cu_norm] = calc_pd_control_5mass(z, Kp, Kd)
    F1 = -Kp(1) * z(1) - Kd(1) * z(2);
    F2 = -Kp(5) * z(9) - Kd(5) * z(10);
    F = [F1; F2];
    priorities = [1; 5]; 
    cu_norm = 0;
end