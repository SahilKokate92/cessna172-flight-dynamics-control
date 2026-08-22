clc
clear
close all

run("initializeParameters_C172.m")

ts = 60;     % simulation time (s)
t_span = [0 ts];

% Initial conditions
x0 = [V_cruise; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0;];

% Control input
U = [0; 0; 0; 0.7];

%% Solving by using ODE45 function

options = odeset('OutputFcn', @odeplot);
disp('⏳ Running Simulation...');

[t, x] = ode45(@(t,x) C172NonlinearModel(t, x, U), t_span, x0);

disp('Simulation Completed!');

