% Full 12-State & 4-Control Trim Calculation
% Aircraft: Cessna 172 (6-DOF Nonlinear EOM)
% Target Flight Condition: V = 65 m/s, Altitude h = 500 m (ze = -500 m)
clc;
clear;
close all;

% Load aircraft parameters and stability derivatives
run("initializeParameters_C172.m");

% Flight Conditions
V_target = 65.0;     % Airspeed (m/s)
h_target = 500.0;    % Altitude above sea level (m) 

fprintf('---------------------------------------------------------------------\n');
fprintf('            FULL 12-STATE AIRCRAFT TRIM (Cessna 172)                 \n');
fprintf('---------------------------------------------------------------------\n');
fprintf(' Target Airspeed (V)  : %.2f m/s\n', V_target);
fprintf(' Target Altitude (h)  : %.2f m  (ze = -%.2f m in NED frame)\n\n', h_target, h_target);

% Initial Guess for Free Variables:
% z = [alpha, beta, p, q, r, phi, theta, del_e, del_a, del_r, del_t] (11 variables)
z0 = [0.02; 0.0; 0.0; 0.0; 0.0; 0.0; 0.02; 0.0; 0.0; 0.0; 0.5];

% Solver Options for fsolve
options = optimoptions('fsolve', 'Display', 'iter','TolFun', 1e-12,'TolX', 1e-12, 'MaxFunEvals', 2000, 'MaxIter', 1000);

% Execute fsolve for Full 12-State Trim
disp('Running fsolve for Full 12-State & 4-Control Equilibrium...');
[z_trim, fval, exitflag, output] = fsolve(@(z) full_trim_objective(z, V_target, h_target), z0, options);

if exitflag > 0
    alpha_trim = z_trim(1);
    beta_trim  = z_trim(2);
    p_trim     = z_trim(3);
    q_trim     = z_trim(4);
    r_trim     = z_trim(5);
    phi_trim   = z_trim(6);
    theta_trim = z_trim(7);
    del_e_trim = z_trim(8);
    del_a_trim = z_trim(9);
    del_r_trim = z_trim(10);
    del_t_trim = z_trim(11);

    % Construct Full 12x1 State Vector and 4x1 Control Vector
    x_trim = [V_target; alpha_trim; beta_trim; p_trim; q_trim; r_trim; ...
              phi_trim; theta_trim; 0.0; 0.0; 0.0; -h_target];
    U_trim = [del_e_trim; del_a_trim; del_r_trim; del_t_trim];

    % Evaluate exact 12-state derivatives at trim
    xdot_trim = C172NonlinearModel(0, x_trim, U_trim);

    fprintf('\n---------------------------------------------------------------------\n');
    fprintf('                     TRIMMED STATE VECTOR x_trim                       \n');
    fprintf('-----------------------------------------------------------------------\n');
    fprintf('  x(1)  Airspeed (V)        : %10.4f m/s\n', x_trim(1));
    fprintf('  x(2)  Angle of Attack (a) : %10.4f deg  (%10.6f rad)\n', alpha_trim*180/pi, alpha_trim);
    fprintf('  x(3)  Sideslip Angle (b)  : %10.4f deg  (%10.6f rad)\n', beta_trim*180/pi, beta_trim);
    fprintf('  x(4)  Roll Rate (p)       : %10.4f deg/s (%10.6f rad/s)\n', p_trim*180/pi, p_trim);
    fprintf('  x(5)  Pitch Rate (q)      : %10.4f deg/s (%10.6f rad/s)\n', q_trim*180/pi, q_trim);
    fprintf('  x(6)  Yaw Rate (r)        : %10.4f deg/s (%10.6f rad/s)\n', r_trim*180/pi, r_trim);
    fprintf('  x(7)  Roll Angle (phi)    : %10.4f deg  (%10.6f rad)\n', phi_trim*180/pi, phi_trim);
    fprintf('  x(8)  Pitch Angle (theta) : %10.4f deg  (%10.6f rad)\n', theta_trim*180/pi, theta_trim);
    fprintf('  x(9)  Yaw Angle (psi)     : %10.4f deg  (%10.6f rad)\n', x_trim(9)*180/pi, x_trim(9));
    fprintf('  x(10) Position Xe         : %10.4f m\n', x_trim(10));
    fprintf('  x(11) Position Ye         : %10.4f m\n', x_trim(11));
    fprintf('  x(12) Position Ze         : %10.4f m   (Altitude h = %.2f m)\n', x_trim(12), -x_trim(12));

    fprintf('\n---------------------------------------------------------------------\n');
    fprintf('                    TRIMMED CONTROL INPUTS U_trim                    \n');
    fprintf('---------------------------------------------------------------------\n');
    fprintf('  U(1) Elevator Defl (del_e): %10.4f deg  (%10.6f rad)\n', del_e_trim*180/pi, del_e_trim);
    fprintf('  U(2) Aileron Defl  (del_a): %10.4f deg  (%10.6f rad)\n', del_a_trim*180/pi, del_a_trim);
    fprintf('  U(3) Rudder Defl   (del_r): %10.4f deg  (%10.6f rad)\n', del_r_trim*180/pi, del_r_trim);
    fprintf('  U(4) Throttle Setting(del_t): %10.4f       (%.2f%%)\n', del_t_trim, del_t_trim*100);

    fprintf('\n---------------------------------------------------------------------\n');
    fprintf('            FULL 12-STATE DERIVATIVES xdot AT TRIM (Residuals)         \n');
    fprintf('-----------------------------------------------------------------------\n');
    state_names = {'V_dot', 'alpha_dot', 'beta_dot', 'p_dot', 'q_dot', 'r_dot', ...
                   'phi_dot', 'theta_dot', 'psi_dot', 'xe_dot', 'ye_dot', 'ze_dot'};
    for i = 1:12
        fprintf('  xdot(%2d) %-12s = %e\n', i, state_names{i}, xdot_trim(i));
    end
    fprintf('---------------------------------------------------------------------\n');

    % Stability Analysis via System Linearization A Matrix
    disp('Computing System Linearization Matrix A and Stability Eigenvalues...');
    A_sys = zeros(12, 12);
    eps_val = 1e-6;
    for j = 1:12
        x_p = x_trim; x_p(j) = x_p(j) + eps_val;
        x_m = x_trim; x_m(j) = x_m(j) - eps_val;
        A_sys(:, j) = (C172NonlinearModel(0, x_p, U_trim) - C172NonlinearModel(0, x_m, U_trim)) / (2 * eps_val);
    end

    eig_vals = eig(A_sys);

    fprintf('\n---------------------------------------------------------------------\n');
    fprintf('                    DYNAMIC STABILITY EIGENVALUES                    \n');
    fprintf('---------------------------------------------------------------------\n');
    for k = 1:12
        fprintf('  Eigenvalue %2d: %12.6f + %12.6fi\n', k, real(eig_vals(k)), imag(eig_vals(k)));
    end
    fprintf('---------------------------------------------------------------------\n');
    
    % Verify Small-Disturbance Stability
    non_zero_reals = real(eig_vals(abs(real(eig_vals)) > 1e-5));
    if all(non_zero_reals < 0)
        fprintf('  STATUS: AIRCRAFT IS STABLE AT THIS TRIM POINT!\n');
        fprintf('  (All non-zero dynamic eigenvalues have negative real parts).\n');
    else
        fprintf('  STATUS: AIRCRAFT HAS UNSTABLE DYNAMIC MODES AT THIS TRIM POINT.\n');
    end
    fprintf('---------------------------------------------------------------------\n');

    
    % Save results
    save('trim_results.mat', 'x_trim', 'U_trim', 'z_trim', 'A_sys', 'eig_vals');
    fprintf('Saved trim results and linearization matrix to trim_results.mat\n\n');
else
    fprintf('\n fsolve failed to find trim solution. Exitflag: %d\n', exitflag);
end


% Full 12-State Objective Function for fsolve

function F = full_trim_objective(z, V_target, h_target)
    alpha = z(1);
    beta  = z(2);
    p     = z(3);
    q     = z(4);
    r     = z(5);
    phi   = z(6);
    theta = z(7);
    del_e = z(8);
    del_a = z(9);
    del_r = z(10);
    del_t = z(11);

    % Assemble Full 12x1 State Vector x and 4x1 Control Vector U
    % ze = -h_target (NED height coordinate)
    x = [V_target; alpha; beta; p; q; r; phi; theta; 0.0; 0.0; 0.0; -h_target];
    U = [del_e; del_a; del_r; del_t];

    % Evaluate 6DOF nonlinear EOM
    xdot = C172NonlinearModel(0, x, U);

    % Residual rates that must be zero for steady straight & level flight:
    % [V_dot, alpha_dot, beta_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, ye_dot, ze_dot]
    F = [xdot(1);   % V_dot = 0
         xdot(2);   % alpha_dot = 0
         xdot(3);   % beta_dot = 0
         xdot(4);   % p_dot = 0
         xdot(5);   % q_dot = 0
         xdot(6);   % r_dot = 0
         xdot(7);   % phi_dot = 0
         xdot(8);   % theta_dot = 0
         xdot(9);   % psi_dot = 0
         xdot(11);  % ye_dot = 0 (no lateral drift)
         xdot(12)]; % ze_dot = 0 (no climb/descent, h = 500m constant)
end
