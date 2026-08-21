% Stability and Control derivatives for Cessna 172
CD0 = 0.031;
CD_alpha = 0.13;
CD_q = 0;
CD_del_e = 0.06;
CD_jh = 0;

CL0 = 0.31;
CL_alpha = 5.143;
CL_q = 3.9;
Cl_del_e = 0.43;
CL_jh = 0;

CY_beta = -0.31;
CY_p = -0.037;
CY_r = 0.21;
CY_del_a = 0.0;
CY_del_r = 0.187;

Cl0 = 0;
Cl_beta = -0.089;
Cl_p = -0.47;
Cl_r = 0.096;
CL_del_a = -0.178;
Cl_del_r = 0.0147;

Cm0 = -0.015;
Cm_alpha = -0.89;
Cm_q = -12.4;
Cm_del_e = -1.28;
Cm_jh = 0;

Cn0 = 0;
Cn_beta = 0.065;
Cn_p = -0.03;
Cn_r = -0.099;
Cn_del_a = -0.053;
Cn_del_r = -0.0657;

save("Stability&ControlDerivative.mat")

