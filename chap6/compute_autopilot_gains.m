addpath('../chap5')
% load transfer_function_coef
addpath('../parameters')
simulation_parameters
compute_tf_model

% AP stands for autopilot
AP.gravity = 9.81;
AP.sigma = 0;
AP.Va0 = 150;
AP.Ts = SIM.ts_simulation; %sample period in digital implementation of PID loops
AP.tau = 0.1;
AP.climb_pitch = 15/180*pi;

AP.altitude_hold_zone = 100;
AP.altitude_take_off_zone = 500;

% saturation limits
AP.delta_a_max = 45/180*pi;
AP.delta_r_max = 45/180*pi;
AP.delta_e_max = 45/180*pi;
AP.e_phi_max = 15/180*pi;
AP.e_beta_max = 15/180*pi;
AP.e_theta_max = 10/180*pi;

%----------roll loop-------------
zeta_phi = 0.95;
omega_phi = sqrt(abs(a_phi2)*AP.delta_a_max/AP.e_phi_max);
AP.roll_kp = sign(a_phi2)*AP.delta_a_max/AP.e_phi_max;
AP.roll_kd = (2*zeta_phi*omega_phi - a_phi1)/a_phi2;

%----------course loop-------------
W_course = 15;
zeta_course = 0.98;
omega_course = omega_phi/W_course;
AP.course_kp = 2*zeta_course*omega_course*AP.Va0/AP.gravity;
AP.course_ki = omega_course*omega_course*AP.Va0/AP.gravity;

%----------sideslip loop-------------
zeta_beta = 0.95;
AP.sideslip_kp = sign(a_beta2)*AP.delta_r_max/AP.e_beta_max;
AP.sideslip_ki = ((a_beta1 + a_beta2*AP.sideslip_kp)/2/zeta_beta)^2/a_beta2;

%----------yaw damper-------------

AP.yaw_damper_tau_r = 0;
AP.yaw_damper_kp = 0;

%----------pitch loop-------------
zeta_theta = 0.9;
omega_theta = sqrt(a_theta2 + abs(a_theta3*AP.delta_e_max/AP.e_theta_max));
AP.pitch_kp = sign(a_theta3)*AP.delta_e_max/AP.e_theta_max;
AP.pitch_kd = (2*zeta_theta*omega_theta - a_theta1)/a_theta3;
AP.K_theta_DC = AP.pitch_kp*a_theta3/(a_theta2 + AP.pitch_kp*a_theta3);

%----------altitude loop-------------
W_h = 15;
omega_h = omega_theta/W_h;
zeta_h = 0.93;
AP.altitude_kp = 2*zeta_h*omega_h/AP.K_theta_DC/AP.Va0;
AP.altitude_ki = omega_h*omega_h/AP.K_theta_DC/AP.Va0;

%---------airspeed hold using pitch---------------
W_v2 = 13;
omega_v2 = omega_theta/W_v2;
zeta_v2 = 0.93;
AP.airspeed_pitch_ki = -omega_v2*omega_v2/AP.K_theta_DC/AP.gravity;
AP.airspeed_pitch_kp = (a_V1 - 2*zeta_v2*omega_v2)/AP.K_theta_DC/AP.gravity;

%---------airspeed hold using throttle---------------
omega_v = 5;
zeta_v = 0.98;
AP.airspeed_throttle_kp = (2*zeta_v*omega_v - a_V1)/a_V2;
AP.airspeed_throttle_ki = omega_v*omega_v/a_V2;
