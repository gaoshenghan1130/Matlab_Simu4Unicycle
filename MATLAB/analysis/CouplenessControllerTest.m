clear; clc; close all;

%% 1. Build symbolic model (3-Mass Nonlinear Oscillator)
syms z1 z2 z3 z4 z5 z6 F1_sym F2_sym real
z_sym = [z1; z2; z3; z4; z5; z6];
F_sym = [F1_sym; F2_sym]; % Two control inputs

% System parameters
m1 = 1.0; 
m2 = 0.5; 
m3 = 3.0; 
k1 = 5.0; % Spring 1 stiffness (between m1 & m2)
k2 = 5.0; % Spring 2 stiffness (between m2 & m3)
c1 = 0.2; % Damping 1
c2 = 0.3; % Damping 2

% Nonlinear forces between the masses
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

%% 3. Design Coupleness Controller
% Controller parameters
gamma = 0.1;               
Kp = [10, 10, 10];  % Proportional gains for m1, m2, m3
Kd = [5, 5, 5];     % Derivative gains for m1, m2, m3
% Error weights (weighing masses differently to observe priority shifts)
W = [1.0, 0.1, 1.5, 0.1, 1.0, 0.1];  

% Define controller handle
coupleness_controller = @(t, z) calc_coupleness_control(z, A_func, B_func, gamma, Kp, Kd, W);

%% 4. Run Fixed-Step Simulation (timeConstantSimu)
disp('Starting fixed-step Euler simulation...');
% Initial conditions: m1 and m3 are stretched outwards, m2 at center
z0 = [0.5; 0; 0.; 0; -0.5; 0]; 
T_end = 10; % Extended simulation time slightly to observe settling
dt = 0.001; 

% Run custom solver
[t_out, z_out] = timeConstantSimu(z0, T_end, dt, model_func, coupleness_controller);

% Recalculate Control Force, Priorities, and c_u norm for plotting
F_out = zeros(length(t_out), 2);
Priority_out = zeros(length(t_out), 2);
cu_out = zeros(length(t_out), 1);

for i = 1:length(t_out)
    [F_out(i,:), Priority_out(i,:), cu_out(i)] = coupleness_controller(t_out(i), z_out(i,:)');
end

%% 5. Plot Results
disp('Plotting results...');
figure('Position', [100, 50, 800, 900]);

subplot(4,1,1);
plot(t_out, z_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_out, z_out(:,3), 'r', 'LineWidth', 1.5);
plot(t_out, z_out(:,5), 'k', 'LineWidth', 1.5);
ylabel('Positions (m)');
legend('Mass 1 (z_1)', 'Mass 2 (z_3)', 'Mass 3 (z_5)');
title('Coupleness Analysis Control on 3-Mass Nonlinear Oscillator');
grid on;

subplot(4,1,2);
plot(t_out, F_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_out, F_out(:,2), 'k', 'LineWidth', 1.5);
ylabel('Force F (N)');
legend('F_1 (on Mass 1)', 'F_2 (on Mass 3)');
grid on;

subplot(4,1,3);
% Plot the two targeted priority masses at each time step
plot(t_out, Priority_out(:,1), 'g.', 'MarkerSize', 5); hold on;
plot(t_out, Priority_out(:,2), 'm.', 'MarkerSize', 5);
ylabel('Priority Targets');
yticks([1, 2, 3]);
yticklabels({'Mass 1', 'Mass 2', 'Mass 3'});
legend('Priority 1', 'Priority 2', 'Location', 'best');
grid on;

subplot(4,1,4);
plot(t_out, cu_out, 'm', 'LineWidth', 1.5);
ylabel('||C_{u, sel}|| (Coupling Norm)');
xlabel('Time (s)');
grid on;


%% ================== Helper Functions ================== %%

% Custom Fixed-Step Simulator (Adapted for MIMO)
function [t, Z] = timeConstantSimu(z0, T_end, dt, model, controller)
    t = 0:dt:T_end;
    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';
    for k = 1:length(t)-1
        zk = Z(k, :)';
        % 1. Get control input (MIMO)
        [F, ~, ~] = controller(t(k), zk);
        % 2. Calculate derivatives
        dz = model(zk, F(:));
        % 3. Euler forward integration
        zk_next = zk + dt * dz;
        Z(k+1, :) = zk_next';
    end
end

% Core Controller Implementation (MIMO Extension)
function [F, priorities, cu_norm] = calc_coupleness_control(z, A_func, B_func, gamma, Kp, Kd, W)
    % 1. Evaluate A (6x6) and B (6x2)
    A = A_func(z);
    B = B_func(z);
    
    % 2. Calculate Coupleness Matrices
    I = eye(6);
    Cz = inv(I - gamma * A) - I;
    Cu = Cz * B; % Cu is now 6x2
    
    % 3. Evaluate strongest coupled state for each of the 3 masses
    scores = zeros(3, 1);
    G_val = W(:) .* abs(z); 
    for m = 1:3
        idx = (m-1)*2 + 1; % Position indices: 1, 3, 5
        % Score calculation integrating over all state couplings
        scores(m) = sum(abs(Cz(idx, :))' .* G_val);
    end
    
    % 4. Determine priorities: Select the top 2 masses with highest scores
    [~, sorted_idx] = sort(scores, 'descend');
    priorities = sorted_idx(1:2); % Indices in [1, 2, 3]
    
    % 5. Build virtual control vector 'v' for the 2 priority masses
    v = zeros(2, 1);
    state_idx = zeros(2, 1);
    for i = 1:2
        m = priorities(i);
        s_idx = (m-1)*2 + 1; % Map mass [1,2,3] to state [1,3,5]
        state_idx(i) = s_idx;
        v(i) = -Kp(m) * z(s_idx) - Kd(m) * z(s_idx+1); 
    end
    
    % 6. Extract the 2x2 MIMO input coupleness submatrix
    Cu_sel = Cu(state_idx, :);
    cu_norm = norm(Cu_sel);
    
    % 7. Calculate Control Forces using MIMO Tikhonov Regularization 
    delta = 1e-3; % Damping factor to prevent singularity
    F = Cu_sel' * ((Cu_sel * Cu_sel' + delta * eye(2)) \ v);
end