% State-Space Linearization + Longitudinal/Lateral-Directional 
% Builds Delta_xdot = A*Delta_x + B*Delta_u  (12 states, 4 controls) by
% numerically differentiating C172NonlinearModel.m around the trim point
% saved in trim_results.mat, then splits the coupled 12-state model into
% a 5-state Longitudinal subsystem and a 5-state Lateral-Directional
% ready for transfer function.
clc
clear

run("initializeParameters_C172.m");
load("trim_results.mat", "x_trim", "U_trim");

fprintf('---------------------------------------------------------------------\n');
fprintf('                STATE-SPACE LINEARIZATION (A, B, C, D)               \n');
fprintf('---------------------------------------------------------------------\n');

n_x = 12; n_u = 4;
eps_x = 1e-6;   % perturbation size for states
eps_u = 1e-6;   % perturbation size for controls

%% Build A = df/dx at trim (central difference) 
A = zeros(n_x, n_x);
for j = 1:n_x
    xp = x_trim; xp(j) = xp(j) + eps_x;
    xm = x_trim; xm(j) = xm(j) - eps_x;
    A(:, j) = (C172NonlinearModel(0, xp, U_trim) - C172NonlinearModel(0, xm, U_trim)) / (2*eps_x);
end

%% Build B = df/du at trim (central difference) 
B = zeros(n_x, n_u);
for j = 1:n_u
    up = U_trim; up(j) = up(j) + eps_u;
    um = U_trim; um(j) = um(j) - eps_u;
    B(:, j) = (C172NonlinearModel(0, x_trim, up) - C172NonlinearModel(0, x_trim, um)) / (2*eps_u);
end

fprintf('linear model built: A is 12x12, B is 12x4.\n');

%% Full 12-state state-space object (C = I, D = 0: every state is an output)
C = eye(n_x);
D = zeros(n_x, n_u);

state_names = {'V','alpha','beta','p','q','r','phi','theta','psi','xe','ye','ze'};
input_names = {'del_e','del_a','del_r','del_t'};

sys_full = ss(A, B, C, D);
sys_full.StateName = state_names;
sys_full.InputName = input_names;

%%  Decouple into Longitudinal and Lateral-Directional subsystems
% x = [V(1) alpha(2) beta(3) p(4) q(5) r(6) phi(7) theta(8) psi(9) xe(10) ye(11) ze(12)]
% U = [del_e(1) del_a(2) del_r(3) del_t(4)]

lon_idx   = [1 2 5 8 12];   % V, alpha, q, theta, ze
lon_u_idx = [1 4];          % del_e, del_t

lat_idx   = [3 4 6 7 9];    % beta, p, r, phi, psi
lat_u_idx = [2 3];          % del_a, del_r

A_lon_raw = A(lon_idx, lon_idx);
B_lon_raw = B(lon_idx, lon_u_idx);

A_lat = A(lat_idx, lat_idx);
B_lat = B(lat_idx, lat_u_idx);

% Replace ze with altitude h = -ze via a sign-flip similarity transform.
% T is its own inverse (T*T = I), so A_h = T*A_raw*T and B_h = T*B_raw.
T = diag([1 1 1 1 -1]);
A_lon = T * A_lon_raw * T;
B_lon = T * B_lon_raw;

C_lon = eye(5); D_lon = zeros(5,2);
C_lat = eye(5); D_lat = zeros(5,2);

sys_lon = ss(A_lon, B_lon, C_lon, D_lon);
sys_lon.StateName = {'V','alpha','q','theta','h'};
sys_lon.InputName = {'del_e','del_t'};

sys_lat = ss(A_lat, B_lat, C_lat, D_lat);
sys_lat.StateName = {'beta','p','r','phi','psi'};
sys_lat.InputName = {'del_a','del_r'};

%% eigenvalues per subsystem 
fprintf('\n---------------------------------------------------------------------\n');
fprintf('        LONGITUDINAL EIGENVALUES (V, alpha, q, theta, h)             \n');
fprintf('---------------------------------------------------------------------\n');
disp(eig(A_lon));

fprintf('\n---------------------------------------------------------------------\n');
fprintf('     LATERAL-DIRECTIONAL EIGENVALUES (beta, p, r, phi, psi)          \n');
fprintf('---------------------------------------------------------------------\n');
disp(eig(A_lat));

%% Save everything for FCS design
save('linearized_model.mat', 'A', 'B', 'C', 'D', 'sys_full', ...
     'A_lon', 'B_lon', 'C_lon', 'D_lon', 'sys_lon', ...
     'A_lat', 'B_lat', 'C_lat', 'D_lat', 'sys_lat');

fprintf('\nSaved full and decoupled state-space models to linearized_model.mat\n');