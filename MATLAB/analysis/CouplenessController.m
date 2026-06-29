clear; clc;
addpath("param/", "model/");
par = LatParam();

%% 1. Build symbolic model
syms theta theta_dot r r_dot F_sym real
syms m_L m_B m_W h g R real
syms Friction I_b I_w I_rod real

z_sym = [theta; theta_dot; r; r_dot];

M_matrix = [2*m_L * R^2 + m_B*(R+h)^2 + m_W*R^2 + 2*m_L*r^2 + par.I_b + par.I_w + par.I_rod, -2*m_L*R;
            -2*m_L*R, 2*m_L];
            
M_rightside = [
    2*m_L*g* R*sin(theta) - 2*m_L*g*r*cos(theta) + m_B* g*(R+h)*sin(theta) + m_W*g*R*sin(theta)  - 4  * m_L*r*r_dot* theta_dot;
    - 2*m_L*g*sin(theta) + F_sym + 2*m_L * r * theta_dot ^2 
];

accel = M_matrix \ M_rightside;
dz_sym = [
    theta_dot;
    accel(1);
    r_dot;
    accel(2)
];

%% 2. Solve symbolic Jacobian matrices and generate dynamic functions
disp('Calculating Jacobian matrices...');
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

disp('Generating dynamic functions for the control loop (this may take a few seconds)...');
A_func = matlabFunction(A_sym, 'Vars', {z_sym, F_sym, m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod});
B_func = matlabFunction(B_sym, 'Vars', {z_sym, m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod});

%% 3. Design Coupleness Controller
% Controller tuning section
gamma = 0.1;              % Step parameter for coupleness calculation
Kp = [10, 5];           % Proportional gains [Kp_theta, Kp_r]
Kd = [3, 3];             % Derivative gains [Kd_theta, Kd_r]
W = [1.0, 0.1, 1.0, 0.1];  % Error weights [W_theta, W_thetadot, W_r, W_rdot]

% Define anonymous controller
coupleness_controller = @(t, z, par) calc_coupleness_control(z, par, A_func, B_func, gamma, Kp, Kd, W);

%% 4. Run Fixed-Step Simulation
disp('Starting fixed-step Euler simulation...');

% Call your custom simulation function
[t_out, z_out] = timeConstantSimu(par, @LatModel_SignCorrection, coupleness_controller);

% Recalculate control input F over the entire process for plotting
F_out = zeros(length(t_out), 1);
for i = 1:length(t_out)
    F_out(i) = coupleness_controller(t_out(i), z_out(i,:)', par);
end

%% 5. Plot Results
disp('Plotting results...');
figure;
subplot(3,1,1);
plot(t_out, z_out(:,1) * 180/pi, 'LineWidth', 1.5);
ylabel('Theta (deg)');
title('Coupleness Analysis Controlled System (Fixed-Step)');
grid on;

subplot(3,1,2);
plot(t_out, z_out(:,3), 'LineWidth', 1.5);
ylabel('Position r (m)');
grid on;

subplot(3,1,3);
plot(t_out, F_out, 'r', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Force F (N)');
grid on;


%% ================== Helper Functions ================== %%

% Custom Fixed-Step Simulator
function [t, Z] = timeConstantSimu(par, model, controller)
    % init
    z0 = [10 * pi/180; 0; 0; 0.0];
    % Simulation settings
    dt = 0.001; 
    T  = 1;           
    t  = 0:dt:T;
    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';
    for k = 1:length(t)-1
        zk = Z(k, :)';
        % Call the model dynamically
        dz = model(t(k), zk, par, controller);
        zk_next = zk + dt * dz;
        Z(k+1, :) = zk_next';
    end
end

% Core Controller Implementation
function F = calc_coupleness_control(z, par, A_func, B_func, gamma, Kp, Kd, W)
    % 1. Dynamically evaluate A(z) and B(z) at the current state
    A = A_func(z, 0, par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod);
    B = B_func(z, par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod);
    
    % 2. Calculate Coupleness Matrices
    I = eye(4);
    Cz = inv(I - gamma * A) - I;
    Cu = Cz * B;
    
    % 3. Evaluate the strongest coupled state
    score_theta = 0;
    score_r = 0;
    
    for j = 1:4
        G_val = W(j) * abs(z(j)); 
        score_theta = score_theta + abs(Cz(1, j)) * G_val;
        score_r     = score_r     + abs(Cz(3, j)) * G_val;
    end
    
    % 4. Determine priority and calculate virtual control
    if score_theta >= score_r
        pi_1 = 1; % Prioritize theta
        v = -Kp(1) * z(1) - Kd(1) * z(2); 
    else
        pi_1 = 3; % Prioritize r
        v = -Kp(2) * z(3) - Kd(2) * z(4);
    end
    
    % 5. Extract input coupleness and calculate final force
    c_u = Cu(pi_1);
    
    if abs(c_u) < 1e-3
        c_u = sign(c_u + eps) * 1e-3;
    end

    delta = 1e-4;
    
    F = v * (c_u / (c_u^2 + delta));
    
end

