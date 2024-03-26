% x_trim is the trimmed state,
% u_trim is the trimmed input
% 计算气动参数
compute_trim

C_p0 = MAV.Gamma3*MAV.C_ell_0 + MAV.Gamma4*MAV.C_n_0;
C_p_beta = MAV.Gamma3*MAV.C_ell_beta + MAV.Gamma4*MAV.C_n_beta;
C_p_p = MAV.Gamma3*MAV.C_ell_p + MAV.Gamma4*MAV.C_n_p;
C_p_r = MAV.Gamma3*MAV.C_ell_r + MAV.Gamma4*MAV.C_n_r;
C_p_delta_a = MAV.Gamma3*MAV.C_ell_delta_a + MAV.Gamma4*MAV.C_n_delta_a;
C_p_delta_r = MAV.Gamma3*MAV.C_ell_delta_r + MAV.Gamma4*MAV.C_n_delta_r;
C_r0 = MAV.Gamma4*MAV.C_ell_0 + MAV.Gamma8*MAV.C_n_0;
C_r_beta = MAV.Gamma4*MAV.C_ell_beta + MAV.Gamma8*MAV.C_n_beta;
C_r_p = MAV.Gamma4*MAV.C_ell_p + MAV.Gamma8*MAV.C_n_p;
C_r_r = MAV.Gamma4*MAV.C_ell_r + MAV.Gamma8*MAV.C_n_r;
C_r_delta_a = MAV.Gamma4*MAV.C_ell_delta_a + MAV.Gamma8*MAV.C_n_delta_a;
C_r_delta_r = MAV.Gamma4*MAV.C_ell_delta_r + MAV.Gamma8*MAV.C_n_delta_r;

% Va_trim = sqrt(MAV.u0^2+MAV.v0^2+MAV.w0^2);
% alpha_trim = atan(MAV.w0/MAV.u0);
% beta_trim = asin(MAV.v0/Va_trim);
% chi_trim = beta_trim + MAV.psi0;
%theta_trim = MAV.theta0;

Va_trim=sqrt(x_trim(4)^2+x_trim(5)^2+x_trim(6)^2);
alpha_trim=y_trim(2);
theta_trim=x_trim(8);
beta_trim =y_trim(3);
chi_trim = beta_trim + x_trim(9);
% delta_e=u_trim(1);
% delta_t=u_trim(4);

% define transfer functions
%副翼到滚转通道
a_phi1 = -1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.b*C_p_p*MAV.b/2/Va_trim;
a_phi2 = 1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.b*C_p_delta_a;
% d_phi2 = MAV.Gamma1*MAV.p0*MAV.q0 - MAV.Gamma2*MAV.q0*MAV.r0 + 1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.b*(C_p0+C_p_beta*beta_trim ...
%     -C_p_p*MAV.b/2/Va_trim*d_phi1+C_p_r*MAV.r0*MAV.b/2/Va_trim+C_r_delta_r*u_trim(3));
    
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([MAV.gravity/Va_trim],[1,0]);

%方向舵到侧滑角
a_beta1 = -1/2*MAV.rho*Va_trim*MAV.S_wing/MAV.mass*MAV.C_Y_beta;
a_beta2 = -1/2*MAV.rho*Va_trim*MAV.S_wing/MAV.mass*MAV.C_Y_delta_r;

T_v_delta_r     = tf([a_beta2],[1,a_beta1]);

%升降舵到俯仰
a_theta1 = -1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.c/MAV.Jy*MAV.C_m_q*MAV.c/2/Va_trim;
a_theta2 = -1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.c/MAV.Jy*MAV.C_m_alpha;
a_theta3 = 1/2*MAV.rho*Va_trim*Va_trim*MAV.S_wing*MAV.c/MAV.Jy*MAV.C_m_delta_e;


T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);

%引擎节流阀到空速
a_V1 = MAV.rho*Va_trim*MAV.S_wing/MAV.mass*(MAV.C_D_0+MAV.C_D_alpha*alpha_trim+MAV.C_D_delta_e*u_trim(1))...
    + MAV.rho*MAV.S_prop*MAV.C_prop*Va_trim/MAV.mass;
a_V2 = MAV.rho*Va_trim*MAV.S_wing/MAV.mass*MAV.C_prop*MAV.k_motor*MAV.k_motor*u_trim(4);
a_V3 = MAV.gravity*cos(MAV.theta0-chi_trim);

T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
