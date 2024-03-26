function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input
u_r=x_trim(4);
v_r=x_trim(5);
w_r=x_trim(6);
theta_trim=x_trim(8);
delta_e_trim=u_trim(1);
delta_t_trim=u_trim(4);
Va_trim=sqrt(u_r^2+v_r^2+w_r^2);
alpha_trim=atan(w_r/u_r);





gamma=P.Jx*P.Jz-P.Jxz^2;
% gam1=P.Jxz*(P.Jx-P.Jy+P.Jz)/gamma;
% gam2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/gamma;
gam3=P.Jz/gamma;
gam4=P.Jxz/gamma;
% gam5=(P.Jz-P.Jx)/P.Jy;
% gam6=P.Jxz/P.Jy;
% gam7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/gamma;
% gam8=P.Jx/gamma;

% C_p_0=gam3*P.C_ell_0+gam4*P.C_n_0;
% C_p_beta=gam3*P.C_ell_beta+gam4*P.C_n_beta;
C_p_p=gam3*P.C_ell_p+gam4*P.C_n_p;
% C_p_r=gam3*P.C_ell_r+gam4*P.C_n_r;
C_p_delta_a=gam3*P.C_ell_delta_a+gam4*P.C_n_delta_a;
% C_p_delta_r=gam3*P.C_ell_delta_r+gam4*P.C_n_delta_r;
% C_r_0=gam4*P.C_ell_0+gam8*P.C_n_0;
% C_r_beta=gam4*P.C_ell_beta+gam8*P.C_n_beta;
% C_r_p=gam4*P.C_ell_p+gam8*P.C_n_p;
% C_r_r=gam4*P.C_ell_r+gam8*P.C_n_r;
% C_r_delta_a=gam4*P.C_ell_delta_a+gam8*P.C_n_delta_a;
% C_r_delta_r=gam4*P.C_ell_delta_r+gam8*P.C_n_delta_r;

% add stuff here
a_phi1=-.5*P.rho*P.Va0^2*P.S_wing*P.b*C_p_p*P.b/2/P.Va0;
a_phi2=.5*P.rho*P.Va0*2*P.S_wing*C_p_delta_a;
a_beta1=-(P.rho*P.Va0*P.S_wing)/(2*P.mass)*P.C_Y_beta;
a_beta2=(P.rho*P.Va0*P.S_wing)/(2*P.mass)*P.C_Y_delta_r;
a_theta1=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_q*P.c/2/P.Va0;
a_theta2=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_alpha;
a_theta3=-(P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)*P.C_m_delta_e;
a_V1=P.rho*P.Va0*P.S_wing/P.mass*(P.C_D_0+P.C_D_alpha*alpha_trim+P.C_D_delta_e*delta_e_trim)...
    + P.rho*P.S_prop/P.mass*P.C_prop*Va_trim;
a_V2=P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*delta_t_trim;
a_V3=P.gravity*cos(theta_trim-alpha_trim);
    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

