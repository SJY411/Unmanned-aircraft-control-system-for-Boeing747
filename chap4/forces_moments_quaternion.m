% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments_quaternion(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
%     e0      = x(7);
%     e1      = x(8);
%     e2      = x(9);
%     e3      = x(10);
    p       = x(11);
    q       = x(12);
    r       = x(13);
    delta_e = delta(1);%elevator升降舵
    delta_a = delta(2);%aileron副翼
    delta_r = delta(3);%rudder方向舵
    delta_t = delta(4);%throttle引擎节流阀
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis

    [phi, theta, psi] = Quaternion2Euler(x(7:10)');
    
    % compute wind data in NED
    w_n = cos(theta)*cos(psi)*w_ns + cos(theta)*sin(psi)*w_es -sin(theta)*w_ds + u_wg;
    w_e = (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*w_ns + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*w_es...
        + sin(phi)*cos(theta)*w_ds + v_wg;
    w_d = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*w_ns + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w_es...
        + cos(phi)*cos(theta)*w_ds + w_wg;
    %上面计算的是body frame下的风速分量u_w, v_w, w_w
    
    % compute air data
    Va = sqrt((u - w_n)^2 + (v - w_e)^2 + (w - w_d)^2);
    alpha = atan((w - w_d)/(u - w_n));
    beta = asin((v - w_e)/Va);
    
    % compute external forces and torques on aircraft
    C_D = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2/pi/P.e/P.AR;
    sigma_alpha = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/(1 + exp(-P.M*(alpha-P.alpha0)))/(1 + exp(P.M*(alpha+P.alpha0)));
    C_L = (1 - sigma_alpha)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma_alpha*(2*sign(alpha)*sin(alpha)*sin(alpha)*cos(alpha));
    C_X = -C_D*cos(alpha) + C_L*sin(alpha);
    C_X_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    C_X_delta_e = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    C_Z = -C_D*sin(alpha) - C_L*cos(alpha);
    C_Z_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    C_Z_delta_e = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);

    Force(1) =  -P.mass*P.gravity*sin(theta) + (1/2*P.rho*P.S_wing*Va^2)*(C_X + C_X_q*P.c*q/2/Va + C_X_delta_e*delta_e)...
        + (1/2*P.rho*P.S_prop*P.C_prop)*((P.k_motor*delta_t)^2 - Va*Va);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi) + (1/2*P.rho*P.S_wing*Va^2)*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/2/Va + P.C_Y_r*P.b*r/2/Va...
        + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r) + 0;
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi) + (1/2*P.rho*P.S_wing*Va^2)*(C_Z + C_Z_q*P.c*q/2/Va + C_Z_delta_e*delta_e)...
        + 0;
    
    Torque(1) = (1/2*P.rho*P.S_wing*Va^2)*(P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b*p/2/Va + P.C_ell_r*P.b*r/2/Va...
        + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r)) - P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = (1/2*P.rho*P.S_wing*Va^2)*(P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c*q/2/Va + P.C_m_delta_e*delta_e)) + 0;   
    Torque(3) = (1/2*P.rho*P.S_wing*Va^2)*(P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b*p/2/Va + P.C_n_r*P.b*r/2/Va)...
        + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r) + 0;
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end

