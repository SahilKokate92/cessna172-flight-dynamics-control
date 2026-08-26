% function for 12 ODE


function xdot = C172NonlinearModel(t, x, U)

run("initializeParameters_C172.m");

% Extract States
V     = x(1);   % Airspeed (m/s)
alpha = x(2);   % Angle of Attack (rad)
beta  = x(3);   % Sideslip Angle (rad)

p     = x(4);   % roll rate (rad/s)
q     = x(5);   % pitch rate (rad/s)
r     = x(6);   % yaw rate (rad/s)

phi   = x(7);   % roll Angle (rad)
theta = x(8);   % pitch Angle (rad)
psi   = x(9);   % yaw/Heading Angle (rad)

xe    = x(10);  % position Xe (m) 
ye    = x(11);  % position Ye (m)
ze    = x(12);  % position Ze (m)

% Control Inputs
del_e = U(1);   % elevator deflection (rad)
del_a = U(2);   % aileron deflection (rad)
del_r = U(3);   % rudder deflection (rad)
del_t = U(4);   % throttle [0-1]

% Non Dimensional angular rates
V_safe = max(V, 0.1);
p_hat = p*b/(2*V_safe);
q_hat = q*c_bar/(2*V_safe);
r_hat = r*b/(2*V_safe);

% Aerodynamic forces coefficients
CD = CD0 + CD_alpha*alpha + CD_q*q_hat + CD_del_e*del_e;
CL = CL0 + CL_alpha*alpha + CL_q*q_hat + CL_del_e*del_e;
CY = CY_beta*beta + CY_p*p_hat + CY_r*r_hat + CY_del_a*del_a + CY_del_r*del_r;

% Aerodynamic moment coefficients
Cl = Cl_beta*beta + Cl_p*p_hat + Cl_r*r_hat + Cl_del_a*del_a + Cl_del_r*del_r;
Cm = Cm0 + Cm_alpha*alpha + Cm_q*q_hat + Cm_del_e*del_e;
Cn = Cn_beta*beta + Cn_p*p_hat + Cn_r*r_hat + Cn_del_a*del_a + Cn_del_r*del_r;

% Dynamic pressure
rho = 1.225; % density sea level (kg/m3)
qbar = 0.5*rho*V^2;

% Forces and Moments
Drag = qbar*S*CD;         % Drag force (N)
Lift = qbar*S*CL;         % Lift Force (N)
Y    = qbar*S*CY;         % Side force (N)

L = qbar*S*b*Cl;       % Rolling Moment (Nm)
M = qbar*S*c_bar*Cm;   % Pitching Moment (Nm)
N = qbar*S*b*Cn;   % Yawing Moment (Nm)

% Converting into body axis Aerodynamics forces
X_aero = -Drag*cos(alpha)*cos(beta) + Y*cos(alpha)*sin(beta) + Lift*sin(alpha);
Y_aero = -Drag*sin(beta) + Y*cos(beta);
Z_aero = -Drag*sin(alpha)*cos(beta) + Y*sin(alpha)*sin(beta) - Lift*cos(alpha);

% Propulsion model
% assuming max thrust = 2500 N
T_max = 2500; % N
del_t = min(max(del_t, 0), 1);
T = del_t * T_max;
X_thrust = T;
Y_thrust = 0;
Z_thrust = 0;

% Gravity forces in body axis
g = 9.81;
X_gravity = -m*g*sin(theta);
Y_gravity =  m*g*sin(phi)*cos(theta);
Z_gravity =  m*g*cos(phi)*cos(theta);

% Total forces
Fx = X_aero + X_thrust + X_gravity;
Fy = Y_aero + Y_thrust + Y_gravity;
Fz = Z_aero + Z_thrust + Z_gravity;

%% Translational EOM
V_dot = (1/m)*(Fx*cos(alpha)*cos(beta) + Fy*sin(beta) + Fz*sin(alpha)*cos(beta) );

alpha_dot = (1/(V*cos(beta))) * ( (1/m) * (-Fx*sin(alpha) + Fz*cos(alpha)) ) ...
               + q - (p*cos(alpha) + r*sin(alpha)) * tan(beta);

beta_dot = (1/V)* ( (1/m) * (-Fx*cos(alpha)*sin(beta) + Fy*cos(beta) - Fz*sin(alpha)*sin(beta)) ) ...
               + p*sin(alpha) - r*cos(alpha);

%% Rotational EOM
Gamma = inertia.Ixx*inertia.Izz - inertia.Jxz^2;

p_dot =  ( ( inertia.Izz*L + inertia.Jxz*N + inertia.Jxz * (inertia.Ixx - inertia.Iyy + inertia.Izz) * p*q ...
               - (inertia.Izz*(inertia.Izz - inertia.Iyy) + (inertia.Jxz)^2 )* q*r ) ) / Gamma ; 

q_dot =  ( M + (inertia.Izz - inertia.Ixx)* p*r + inertia.Jxz * (r^2 - p^2) ) / inertia.Iyy ;

r_dot =  ( inertia.Jxz*L + inertia.Ixx*N + ( (inertia.Ixx - inertia.Iyy) * inertia.Ixx + (inertia.Jxz)^2 ) * p*q ...
               + inertia.Jxz * (- inertia.Ixx + inertia.Iyy - inertia.Izz) * q*r ) / Gamma ; 

%% Euler Angles
phi_dot = p + (q*sin(phi) + r*cos(phi)) * tan(theta);

theta_dot = q*cos(phi) - r*sin(phi);

psi_dot = (q*sin(phi) + r*cos(phi))/cos(theta);

%% body axis velocity component uvw
u = V*cos(alpha)*cos(beta);
v = V*sin(beta);
w = V*sin(alpha)*cos(beta);

%% Positions
xe_dot = ( u*cos(theta) + ( v*sin(phi)+w*cos(phi) ) * sin(theta) ) * cos(psi) ...
             - ( v*cos(phi) - w*sin(phi) ) * sin(psi) ;

ye_dot = (u*cos(theta) + (v*sin(phi) + w*cos(phi))*sin(theta))*sin(psi) ...
             + (v*cos(phi) - w*sin(phi))*cos(psi);

ze_dot = -u*sin(theta) + ( v*sin(phi) + w*cos(phi) ) * cos(theta) ;

%% body velocities u_dot, v_dot, w_dot
% u_dot = (Fx/m) - q*w + r*v;
% v_dot = (Fy/m) + p*w - r*u;
% w-dot = (Fz/m) - p*v + q*u;


xdot = [V_dot; alpha_dot; beta_dot; p_dot; q_dot; r_dot; phi_dot; theta_dot; psi_dot; xe_dot; ye_dot; ze_dot ];

end 