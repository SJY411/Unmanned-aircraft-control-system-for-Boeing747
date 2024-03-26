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

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    brake1  = delta(5);
    brake2  = delta(6);
    brake3  = delta(7);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    %inertial to body
    Rvb=[cos(theta)*cos(psi), ...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(theta)*sin(psi),...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    %body to inertial
    Rbv=Rvb';
    % compute wind data in NED
    gustv=Rbv*[u_wg;v_wg;w_wg];
    
    
    w_n = w_ns+gustv(1);
    w_e = w_es+gustv(2);
    w_d = w_ds+gustv(3);
    windb=Rvb*[w_n;w_e;w_d];
    urvrwr=[u;v;w]-windb;
    
    % compute air data
    Va = sqrt(sum(urvrwr.^2));
    alpha = atan(urvrwr(3)/urvrwr(1));
    beta = asin(urvrwr(2)/Va);
    
    
    % compute external forces and torques on aircraft
    sigma_alpha=(1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0)))/...
                ((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
    C_L=(1-sigma_alpha)*(P.C_L_0+P.C_L_alpha*alpha)+...
         sigma_alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    C_D=P.C_D_p+(P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*(P.b^2/P.S_wing));
    C_X=-C_D*cos(alpha)+C_L*sin(alpha);
    C_X_q=-P.C_D_q*cos(alpha)+P.C_L_q*sin(alpha);
    C_X_delta_e=-P.C_D_delta_e*cos(alpha)+P.C_L_delta_e*sin(alpha);
    C_Z=-C_D*sin(alpha)-C_L*cos(alpha);
    C_Z_q=-P.C_D_q*sin(alpha)-P.C_L_q*cos(alpha);
    C_Z_delta_e=-P.C_D_delta_e*sin(alpha)-P.C_L_delta_e*cos(alpha);
    
    gForce(1) =  -P.mass*P.gravity*sin(theta);
    gForce(2) =  P.mass*P.gravity*cos(theta)*sin(phi);
    gForce(3) =  P.mass*P.gravity*cos(theta)*cos(phi);
    
    aForce(1) = .5*P.rho*Va^2*P.S_wing*(C_X+C_X_q*P.c/(2*Va)*q+C_X_delta_e*delta_e);
    aForce(2) = .5*P.rho*Va^2*P.S_wing*(P.C_Y_0+P.C_Y_beta*beta+...
        P.C_Y_p*P.b/(2*Va)*p+P.C_Y_r*P.b/(2*Va)*r+P.C_Y_delta_a*delta_a+...
        P.C_Y_delta_r*delta_r);
    aForce(3) = .5*P.rho*Va^2*P.S_wing*(C_Z+C_Z_q*P.c/(2*Va)*q+C_Z_delta_e*delta_e);
    
    %new model
    %pForce(1) = P.rho*P.S_prop*P.C_prop*(Va+delta_t*(P.k_motor-Va))*(delta_t*(P.k_motor-Va));
    %old model
    pForce(1) = .5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    pForce(2) = 0;
    pForce(3) = 0;
    
    Force(1) = gForce(1) + aForce(1) + pForce(1);
    Force(2) = gForce(2) + aForce(2) + pForce(2);
    Force(3) = gForce(3) + aForce(3) + pForce(3);
    
    Torque(1) = P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*P.b/(2*Va)*p+...
        P.C_ell_r*P.b/(2*Va)*r+P.C_ell_delta_a*delta_a+P.C_ell_delta_r*delta_r);
    Torque(2) = P.c*(P.C_m_0+P.C_m_alpha*alpha+P.C_m_q*P.b/(2*Va)*q+...
        P.C_m_delta_e*delta_e);   
    Torque(3) = P.b*(P.C_n_0+P.C_n_beta*beta+P.C_n_p*P.b/(2*Va)*p+...
        P.C_n_r*P.b/(2*Va)*r+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r);
    Torque=Torque.*.5*P.rho*Va^2*P.S_wing;...
    Torque(1)=Torque(1)-P.k_T_P*(P.k_Omega*delta_t)^2;
    

    defl1=0;
    defl2=0;
    defl3=0;
    out = [Force'; Torque'; Va; alpha; beta;  w_n; w_e; w_d;defl1; defl2; defl3];
end



