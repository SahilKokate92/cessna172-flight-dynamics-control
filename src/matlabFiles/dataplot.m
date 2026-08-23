% Cessna 172 6-DOF Nonlinear Simulation Analysis

if ~exist('t', 'var') || ~exist('x', 'var')
    disp('Simulation data not found in workspace. Executing RunSimulation.m...');
    run('RunSimulation.m');
end

% Set global graphics defaults for publication-quality plots
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.8);
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultAxesGridLineStyle', ':');

% Color Palette (Modern Vibrant Theme)
c_blue   = [0.00, 0.45, 0.74];
c_orange = [0.85, 0.33, 0.10];
c_yellow = [0.93, 0.69, 0.13];
c_purple = [0.49, 0.18, 0.56];
c_green  = [0.47, 0.67, 0.19];
c_red    = [0.64, 0.08, 0.18];
c_dark   = [0.15, 0.15, 0.15];

% Extract States from Matrix
V     = x(:, 1);         % Airspeed (m/s)
alpha = rad2deg(x(:, 2)); % AoA (deg)
beta  = rad2deg(x(:, 3)); % Sideslip (deg)

p     = rad2deg(x(:, 4)); % Roll rate (deg/s)
q     = rad2deg(x(:, 5)); % Pitch rate (deg/s)
r     = rad2deg(x(:, 6)); % Yaw rate (deg/s)

phi   = rad2deg(x(:, 7)); % Roll angle (deg)
theta = rad2deg(x(:, 8)); % Pitch angle (deg)
psi   = rad2deg(x(:, 9)); % Yaw angle (deg)

xe    = x(:, 10);        % Position North (m)
ye    = x(:, 11);        % Position East (m)
ze    = x(:, 12);        % Position Down (m)
alt   = -ze;             % Altitude (m)

% Plot State visualization

fig1 = figure('Name', 'Cessna 172 Flight State', ...
              'Color', 'w', 'Units', 'normalized', 'Position', [0.05, 0.08, 0.88, 0.82]);

sgtitle('\bf\fontsize{15}Cessna 172 Nonlinear Flight State', 'Color', c_dark);

% 1. Airspeed
subplot(2, 3, 1);
plot(t, V, 'Color', c_blue, 'DisplayName', 'Airspeed V');
hold on;
if exist('V_cruise', 'var')
    yline(V_cruise, '--', 'Color', c_green, 'LineWidth', 1.2, 'DisplayName', sprintf('V_{cruise} (%.0f m/s)', V_cruise));
end
if exist('V_stall', 'var')
    yline(V_stall, ':', 'Color', c_red, 'LineWidth', 1.2, 'DisplayName', sprintf('V_{stall} (%.0f m/s)', V_stall));
end

grid on; grid minor;
title('\bfAirspeed (V)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Airspeed (m/s)');
legend('Location', 'best');

% 2. Aerodynamic Angles (alpha & beta)
subplot(2, 3, 2);
plot(t, alpha, 'Color', c_orange, 'DisplayName', '\alpha (AoA)');
hold on;
plot(t, beta, 'Color', c_yellow, 'DisplayName', '\beta (Sideslip)');
grid on; grid minor;
title('\bfAerodynamic Angles (\alpha, \beta)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Location', 'best');

% 3. Angular Rates (p, q, r)
subplot(2, 3, 3);
plot(t, p, 'Color', c_blue, 'DisplayName', 'p (Roll rate)');
hold on;
plot(t, q, 'Color', c_orange, 'DisplayName', 'q (Pitch rate)');
plot(t, r, 'Color', c_purple, 'DisplayName', 'r (Yaw rate)');
grid on; grid minor;
title('\bfBody Angular Rates (p, q, r)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Rate (deg/s)');
legend('Location', 'best');

% 4. Euler Attitude Angles (phi, theta, psi)
subplot(2, 3, 4);
plot(t, phi, 'Color', c_blue, 'DisplayName', '\phi (Roll)');
hold on;
plot(t, theta, 'Color', c_orange, 'DisplayName', '\theta (Pitch)');
plot(t, psi, 'Color', c_green, 'DisplayName', '\psi (Yaw)');
grid on; grid minor;
title('\bfEuler Attitude Angles (\phi, \theta, \psi)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Attitude (deg)');
legend('Location', 'best');

% 5. Altitude Profile
subplot(2, 3, 5);
plot(t, alt, 'Color', c_green, 'DisplayName', 'Altitude (h)');
grid on; grid minor;
title('\bfAltitude (-Z_e)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Altitude (m)');
legend('Location', 'best');

% 6. Position (Xe & Ye)
subplot(2, 3, 6);
plot(ye, xe, 'Color', c_purple, 'LineWidth', 2, 'DisplayName', 'Flight Path');
hold on;
plot(ye(1), xe(1), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Start');
plot(ye(end), xe(end), 's', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'End');
grid on; grid minor;
axis equal;
title('\bf2D Position (East vs North)', 'FontSize', 12);
xlabel('East Position Y_e (m)'); ylabel('North Position X_e (m)');
legend('Location', 'best');

% 3D Flight Trajectory visualization

fig2 = figure('Name', '3D Flight Trajectory', ...
              'Color', 'w', 'Units', 'normalized', 'Position', [0.15, 0.12, 0.70, 0.75]);

% 3D Flight Path Curve
p3d = plot3(ye, xe, alt, 'Color', c_blue, 'LineWidth', 2.5, 'DisplayName', '3D Trajectory');
hold on;

% Ground Shadow Projection
plot3(ye, xe, zeros(size(alt)), '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2, 'DisplayName', 'Ground Projection');

% Start and End Markers
plot3(ye(1), xe(1), alt(1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', c_green, 'MarkerEdgeColor', 'k', 'DisplayName', 'Start Point');
plot3(ye(end), xe(end), alt(end), 'o', 'MarkerSize', 8, 'MarkerFaceColor', c_red, 'MarkerEdgeColor', 'k', 'DisplayName', 'Current Position');


grid on; grid minor;
view(37.5, 30); % 3D Isometric View Angle
axis tight;

title('\bf\fontsize{14}Cessna 172 3D Flight Path & Trajectory Projection', 'Color', c_dark);
xlabel('\bfEast Position Y_e (m)');
ylabel('\bfNorth Position X_e (m)');
zlabel('\bfAltitude h (m)');
legend('Location', 'northeast');

% Add aesthetic grid shading
ax = gca;
ax.GridColor = [0.3 0.3 0.3];
ax.GridAlpha = 0.3;

% Force MATLAB to render and display combined figures immediately on screen
drawnow;

disp('---------------------------------------------------------');
disp('✔ Combined Plots Rendered & Displayed');

% Separate Individual Figure Windows
plot_sep = input('\n📊 Do you want to generate each plot in a separate individual figure window? (y/n) [n]: ', 's');

if strcmpi(strtrim(plot_sep), 'y') || strcmpi(strtrim(plot_sep), 'yes')
    disp('⏳ Generating individual separate figures for each state/plot...');
    
    % 1. Airspeed
    fig_v = figure('Name', 'Separate Plot - Airspeed (V)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.10, 0.50, 0.40, 0.40]);
    plot(t, V, 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', 'Airspeed V');
    hold on;
    if exist('V_cruise', 'var')
        yline(V_cruise, '--', 'Color', c_green, 'LineWidth', 1.2, 'DisplayName', sprintf('V_{cruise} (%.0f m/s)', V_cruise));
    end
    if exist('V_stall', 'var')
        yline(V_stall, ':', 'Color', c_red, 'LineWidth', 1.2, 'DisplayName', sprintf('V_{stall} (%.0f m/s)', V_stall));
    end
    grid on; grid minor;
    title('\bfAirspeed Profile (V)', 'FontSize', 13);
    xlabel('Time (s)', 'FontSize', 11); ylabel('Airspeed (m/s)', 'FontSize', 11);
    legend('Location', 'best');

    % 2. Aerodynamic Angles (alpha & beta)
    fig_alpha = figure('Name', 'Separate Plot - Aerodynamic Angles (alpha, beta)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.52, 0.50, 0.40, 0.40]);
    plot(t, alpha, 'Color', c_orange, 'LineWidth', 2.0, 'DisplayName', '\alpha (AoA)');
    hold on;
    plot(t, beta, 'Color', c_yellow, 'LineWidth', 2.0, 'DisplayName', '\beta (Sideslip)');
    grid on; grid minor;
    title('\bfAerodynamic Angles (\alpha, \beta)', 'FontSize', 13);
    xlabel('Time (s)', 'FontSize', 11); ylabel('Angle (deg)', 'FontSize', 11);
    legend('Location', 'best');

    % 3. Body Angular Rates (p, q, r)
    fig_rates = figure('Name', 'Separate Plot - Angular Rates (p, q, r)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.10, 0.05, 0.40, 0.40]);
    plot(t, p, 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', 'p (Roll rate)');
    hold on;
    plot(t, q, 'Color', c_orange, 'LineWidth', 2.0, 'DisplayName', 'q (Pitch rate)');
    plot(t, r, 'Color', c_purple, 'LineWidth', 2.0, 'DisplayName', 'r (Yaw rate)');
    grid on; grid minor;
    title('\bfBody Angular Rates (p, q, r)', 'FontSize', 13);
    xlabel('Time (s)', 'FontSize', 11); ylabel('Rate (deg/s)', 'FontSize', 11);
    legend('Location', 'best');

    % 4. Euler Attitude Angles (phi, theta, psi)
    fig_euler = figure('Name', 'Separate Plot - Euler Angles (phi, theta, psi)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.52, 0.05, 0.40, 0.40]);
    plot(t, phi, 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', '\phi (Roll)');
    hold on;
    plot(t, theta, 'Color', c_orange, 'LineWidth', 2.0, 'DisplayName', '\theta (Pitch)');
    plot(t, psi, 'Color', c_green, 'LineWidth', 2.0, 'DisplayName', '\psi (Yaw)');
    grid on; grid minor;
    title('\bfEuler Attitude Angles (\phi, \theta, \psi)', 'FontSize', 13);
    xlabel('Time (s)', 'FontSize', 11); ylabel('Attitude (deg)', 'FontSize', 11);
    legend('Location', 'best');

    % 5. Altitude Profile
    fig_alt = figure('Name', 'Separate Plot - Altitude (h)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.20, 0.25, 0.40, 0.40]);
    plot(t, alt, 'Color', c_green, 'LineWidth', 2.0, 'DisplayName', 'Altitude h');
    grid on; grid minor;
    title('\bfAltitude Profile (-Z_e)', 'FontSize', 13);
    xlabel('Time (s)', 'FontSize', 11); ylabel('Altitude (m)', 'FontSize', 11);
    legend('Location', 'best');

    % 6. 2D Position Ground Track
    fig_pos = figure('Name', 'Separate Plot - 2D Position Ground Track', 'Color', 'w', 'Units', 'normalized', 'Position', [0.40, 0.25, 0.40, 0.40]);
    plot(ye, xe, 'Color', c_purple, 'LineWidth', 2.5, 'DisplayName', 'Flight Path');
    hold on;
    plot(ye(1), xe(1), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Start');
    plot(ye(end), xe(end), 's', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'End');
    grid on; grid minor;
    axis equal;
    title('\bf2D Ground Position Track (East vs North)', 'FontSize', 13);
    xlabel('East Position Y_e (m)', 'FontSize', 11); ylabel('North Position X_e (m)', 'FontSize', 11);
    legend('Location', 'best');

    disp('✔ Separate Individual Figure Windows Generated Successfully');
    drawnow;
end

% Saving Generated Plots
save_choice = input('\n💾 Do you want to save all generated plots to results folder? (y/n) [n]: ', 's');

if strcmpi(strtrim(save_choice), 'y') || strcmpi(strtrim(save_choice), 'yes')
    % Define absolute target results directory
    mfile_dir = fileparts(mfilename('fullpath'));
    results_dir = fullfile(mfile_dir, '..', '..', 'results');
    
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    disp('⏳ Saving generated figures to results directory...');
    
    % Save Combined Figures
    if exist('fig1', 'var') && ishandle(fig1)
        saveas(fig1, fullfile(results_dir, 'Flight_State_Combined.png'));
    end
    if exist('fig2', 'var') && ishandle(fig2)
        saveas(fig2, fullfile(results_dir, '3D_Trajectory.png'));
    end
    
    % Save Separate Figures if generated
    if exist('fig_v', 'var') && ishandle(fig_v)
        saveas(fig_v, fullfile(results_dir, 'Airspeed_V.png'));
    end
    if exist('fig_alpha', 'var') && ishandle(fig_alpha)
        saveas(fig_alpha, fullfile(results_dir, 'Aerodynamic_Angles.png'));
    end
    if exist('fig_rates', 'var') && ishandle(fig_rates)
        saveas(fig_rates, fullfile(results_dir, 'Angular_Rates.png'));
    end
    if exist('fig_euler', 'var') && ishandle(fig_euler)
        saveas(fig_euler, fullfile(results_dir, 'Euler_Attitude_Angles.png'));
    end
    if exist('fig_alt', 'var') && ishandle(fig_alt)
        saveas(fig_alt, fullfile(results_dir, 'Altitude_Profile.png'));
    end
    if exist('fig_pos', 'var') && ishandle(fig_pos)
        saveas(fig_pos, fullfile(results_dir, 'Ground_Track_2D.png'));
    end
    
    fprintf('💾 All generated plots successfully saved to:\n   %s\n\n', results_dir);
end
