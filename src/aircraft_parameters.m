% Cessna 172:  mass, geometry parameters and performance specifications
% All data is taken from the Research article posted in docs folder
c_bar = 1.4935; % m, mean aerodynamic chord
b = 10.9118;    % m, wing span
S = 16.1651;    % m2, wing area
inertia.Ixx = 1285.3; % kg.m2 
inertia.Iyy = 1824.9; % kg.m2
inertia.Izz = 2666.9; % kg.m2
inertia.Jxy = 0;      % kg.m2
inertia.Jxz = 0;      % kg.m2
inertia.Jyz = 0;      % kg.m2
inertia_matrix = [inertia.Ixx, -inertia.Jxy, -inertia.Jxz;
                 -inertia.Jxy,  inertia.Iyy, -inertia.Jyz;
                 -inertia.Jxz, -inertia.Jyz,  inertia.Izz];
m = 1043.3;           % kg, mass
V_cruise = 65;        % m/s, cruise speed
V_stall  = 24;        % m/s, stall speed
V_ne     = 84;        % m/s, Never exceed speed
mcw      = 7.7;       % m/s, Max cross wind
serCeiling = 4100;    % m, Service ceiling

save("aircraft_parameters.mat");
