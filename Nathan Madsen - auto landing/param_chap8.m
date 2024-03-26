P.altitude_take_off_zone=20;
P.altitude_hold_zone=30;
P.climb_pitch=20*pi/180;
P.theta_max=30*pi/180;
P.phi_max=30*pi/180;

P.gravity = 9.8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 5*pi/180;  % desired flight path angle (radians)
R     = 150;         % desired radius (m) - use (+) for right handed orbit, 
gamma=0;
R=inf;

% autopilot sample rate
P.Ts = 0.01;
% gps sample rate
P.Ts_gps=1;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -1;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate


% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -10;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
% 
% % initial conditions
% P.pn0    = 0;  % initial North position
% P.pe0    = 0;  % initial East position
% P.pd0    = -.7;  % initial Down position (negative altitude)
% P.u0     = .01;  % initial velocity along body x-axis
% P.v0     = 0;  % initial velocity along body y-axis
% P.w0     = 0;  % initial velocity along body z-axis
% P.phi0   = 0;  % initial roll angle
% P.theta0 = 10*pi/180;  % initial pitch angle
% P.psi0   = 0;  % initial yaw angle
% P.p0     = 0;  % initial body frame roll rate
% P.q0     = 0;  % initial body frame pitch rate
% P.r0     = 0;  % initial body frame yaw rate

% compute different transfer functions
% [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
%     = compute_tf_model(x_trim,u_trim,P);



% % % linearize the equations of motion around trim conditions
% [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% GAINS

P.u_r=x_trim(4);
P.v_r=x_trim(5);
P.w_r=x_trim(6);
P.theta_trim=x_trim(8);
P.delta_e_trim=u_trim(1);
P.delta_t_trim=u_trim(4);
P.Va_trim=sqrt(P.u_r^2+P.v_r^2+P.w_r^2);
P.alpha_trim=atan(P.w_r/P.u_r);

P.gamma=P.Jx*P.Jz-P.Jxz^2;
P.gam3=P.Jz/P.gamma;
P.gam4=P.Jxz/P.gamma;

P.C_p_p=P.gam3*P.C_ell_p+P.gam4*P.C_n_p;
P.C_p_delta_a=P.gam3*P.C_ell_delta_a+P.gam4*P.C_n_delta_a;


P.a_phi1=-.5*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_p*P.b/2/P.Va0;
P.a_phi2=.5*P.rho*P.Va0*2*P.S_wing*P.C_p_delta_a;
P.a_beta1=-(P.rho*P.Va0*P.S_wing)/(2*P.mass)*P.C_Y_beta;
P.a_beta2=(P.rho*P.Va0*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;
P.a_theta1=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_q*P.c/2/P.Va0;
P.a_theta2=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_alpha;
P.a_theta3=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_delta_e;
P.a_V1=P.rho*P.Va0*P.S_wing/P.mass*(P.C_D_0+P.C_D_alpha*P.alpha_trim+P.C_D_delta_e*P.delta_e_trim)...
    + P.rho*P.S_prop/P.mass*P.C_prop*P.Va0;
P.a_V2=P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*P.delta_t_trim;
P.a_V3=P.gravity*cos(P.theta_trim-P.alpha_trim);

%roll
P.delta_a_max=45*pi/180;
P.e_phi_max=15*pi/180;

P.kp_phi=P.delta_a_max/P.e_phi_max*sign(P.a_phi2);
P.omega_n_phi=sqrt(abs(P.a_phi2)*P.delta_a_max/P.e_phi_max);
P.zeta_phi=.707*4;
P.kd_phi=(2*P.zeta_phi*P.omega_n_phi-P.a_phi1)/P.a_phi2;

%course
P.W_x=5; %bandwidth separation for roll and course
%P.W_x=15; %bandwidth separation for roll and course
P.omega_n_chi=P.omega_n_phi/P.W_x;
P.zeta_chi=.707*1.5;
P.kp_chi=2*P.zeta_chi*P.omega_n_chi*P.Va0/P.gravity;
P.ki_chi=P.omega_n_chi^2*P.Va0/P.gravity;

%pitch
P.delta_e_max=45*pi/180;
P.e_theta_max=10*pi/180;
P.kp_theta=-1*P.delta_e_max/P.e_theta_max*sign(P.a_theta3);
P.omega_n_theta=sqrt(P.a_theta2+P.delta_e_max/P.e_theta_max*abs(P.a_theta3));
P.zeta_theta=.707;
P.kd_theta=(2*P.zeta_theta*P.omega_n_theta-P.a_theta1)/P.a_theta3;
P.K_theta_DC=(P.kp_theta*P.a_theta3)/(P.a_theta2+P.kp_theta*P.a_theta3);

% altitude
P.W_h=16; %bandwidth separation pitch and altitude
P.omega_n_h=P.omega_n_theta/P.W_h;
P.zeta_h=.707*2;
P.ki_h=P.omega_n_h^2/(P.K_theta_DC*P.Va0);
P.kp_h=(2*P.zeta_h*P.omega_n_h)/(P.K_theta_DC*P.Va0);

% airspeed with Pitch
P.W_V2=8; % bandwidth separation
%P.W_V2=16; % bandwidth separation
P.omega_n_V2=P.omega_n_theta/P.W_V2;
P.zeta_V2=.707*4;
%P.zeta_V2=.707*8;

P.ki_V2=-P.omega_n_V2^2/(P.K_theta_DC*P.gravity);
P.kp_V2=(P.a_V1-2*P.zeta_V2*P.omega_n_V2)/(P.K_theta_DC*P.gravity);

% airspeed with throttle
P.omega_n_V=P.omega_n_V2;
P.zeta_V=.707;
P.ki_V=P.omega_n_V^2/P.a_V2;
P.kp_V=(2*P.zeta_V*P.omega_n_V-P.a_V1)/P.a_V2;


% sensor standard deviations
P.std_gyrox=.13*pi/180;
P.std_gyroy=.13*pi/180;
P.std_gyroz=.13*pi/180;
P.std_accelx=.0025*P.gravity;
P.std_accely=.0025*P.gravity;
P.std_accelz=.0025*P.gravity;
P.bias_abspres=.125;
P.std_abspres=.01;
P.bias_diffpres=.02;
P.std_diffpres=.002;

P.k_GPS=1/1100;
P.std_gpsn=.21;
P.std_gpse=.21;
P.std_gpsh=.4;

P.std_V=.01;

P.bias_gyro_x=0;
P.bias_gyro_y=0;
P.bias_gyro_z=0;

P.maxDefl=.3;
P.ksusp=20*P.mass*P.gravity/P.maxDefl;
P.Bsusp=30000;%20*100;
P.kfric=.7;


P.wheelpositions={[.5;0;.7],[-1;.5; .45],[-1;-.5; .45]};
%P.wheelpositions={[1;0;.7],[-2;1; .45],[-2;-1; .45]};











