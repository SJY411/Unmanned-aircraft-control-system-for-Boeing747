%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 255840;
P.gravity = 9.81;
P.Jx   = 1.93e7;
P.Jy   = 4.38e7;
P.Jz   = 6.16e7;
P.Jxz  = 1.18e6;
% aerodynamic coefficients
P.S_wing        = 511;
P.b             = 59.65;
P.AR            = P.b^2/P.S_wing;
P.c             = 8.3232;
P.S_prop        = 18.52;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 1.11;
P.C_L_alpha     = 5.70;
P.C_L_q         = 5.4;
P.C_L_delta_e   = 0.338;
P.C_D_0         = 0.102;
P.C_D_alpha     = 0.66;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = 0.0;
P.C_m_alpha     = -1.26;
P.C_m_q         = -20.8;
P.C_m_delta_e   = -1.34;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.96;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = 0.175;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.221;
P.C_ell_p       = -0.45;
P.C_ell_r       = 0.101;
P.C_ell_delta_a = 0.0461;
P.C_ell_delta_r = 0.007;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.150;
P.C_n_p         = -0.121;
P.C_n_r         = -0.30;
P.C_n_delta_a   = 0.0064;
P.C_n_delta_r   = -0.109;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
