clc
clear
close all

run("initializeParameters_C172.m")
run("trimAndStability.m")
ts = 60;     % simulation time (s)
t_span = [0 ts];

% Initial conditions
x0 = x_trim;

% Control input
U = U_trim;

%% Solving by using ODE45 function

options = odeset('OutputFcn', @odeplot);
disp('Running Simulation...');

[t, x] = ode45(@(t,x) C172NonlinearModel(t, x, U), t_span, x0);

disp('Simulation Completed!');

