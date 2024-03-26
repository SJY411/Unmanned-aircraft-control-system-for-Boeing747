% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   defl1         = uu(9);
   defl2         = uu(10);
   defl3         = uu(11);
   y_gps_n       = uu(12);
   y_gps_e       = uu(13);
   y_gps_h       = uu(14);
   y_gps_Vg      = uu(15);
   y_gps_course  = uu(16);
   t             = uu(17);
    persistent y_gps_nprev
    persistent y_gps_eprev
    persistent y_static_presprev
    persistent y_diff_presprev
    persistent y_accel_xprev
    persistent y_accel_yprev
    persistent y_accel_zprev
    persistent phatprev
    persistent qhatprev
    persistent rhatprev
    persistent xhat_att
    persistent xhat_gps
    persistent Patt
    persistent Pgps
    persistent gpsPers
   if t==0
       y_gps_nprev=0;
       y_gps_eprev=0;
       y_static_presprev=0;
       y_diff_presprev=0;
       y_accel_xprev=0;
       y_accel_yprev=0;
       y_accel_zprev=0;
       qhatprev=0;
       phatprev=0;
       rhatprev=0; 
       xhat_att=[P.phi0;P.theta0];
       xhat_gps=[P.pn0;P.pe0;sqrt(P.u0^2+P.v0^2+P.w0^2);
           atan2(P.u0*sin(P.psi0)+P.v0*cos(P.psi0),P.u0*cos(P.psi0)+P.v0*sin(P.psi0));
           P.wind_n;P.wind_e;P.psi0];
       %Patt=[(40*pi/180)^2,0;0,(40*pi/180)^2];
       Patt=[0,0;0,0];
       Pgps=zeros(7);
       gpsPers=[0;0;0;0;0];
   end
   Naccel=20;
   Toutaccel=P.Ts;
   Ngps=20;
   Toutgps=P.Ts;
  
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
  
    alpf=30;
    alphaLPF=exp(-alpf*P.Ts);
    alphaLPFgyro=exp(-alpf*.5*P.Ts);
    alphaLPFGPS=exp(-alpf*P.Ts_gps);
%     y_gps_nprev=alphaLPFGPS*y_gps_nprev+(1-alphaLPFGPS)*y_gps_n;
%     y_gps_eprev=alphaLPFGPS*y_gps_eprev+(1-alphaLPFGPS)*y_gps_e;
%     pnhat=y_gps_nprev;
%     pehat=y_gps_eprev;
    y_static_presprev=alphaLPF*y_static_presprev+(1-alphaLPF)*y_static_pres;
    hhat=y_static_presprev/P.rho/P.gravity;
    y_diff_presprev=alphaLPF*y_diff_presprev+(1-alphaLPF)*y_diff_pres;
    Vahat=sqrt(2/P.rho*y_diff_presprev);
    
    
   
    phatprev=alphaLPFgyro*phatprev+(1-alphaLPFgyro)*y_gyro_x;
    phat=phatprev;
    qhatprev=alphaLPFgyro*qhatprev+(1-alphaLPFgyro)*y_gyro_y;
    qhat=qhatprev;
    rhatprev=alphaLPFgyro*rhatprev+(1-alphaLPFgyro)*y_gyro_z;
    rhat=rhatprev;
    
    
%     y_accel_xprev=alphaLPF*y_accel_xprev+(1-alphaLPF)*y_accel_x;
%     y_accel_yprev=alphaLPF*y_accel_yprev+(1-alphaLPF)*y_accel_y;
%     y_accel_zprev=alphaLPF*y_accel_zprev+(1-alphaLPF)*y_accel_z;
%     phihat=atan(y_accel_yprev/y_accel_zprev);
%     thetahat=asin(y_accel_xprev)/P.gravity;
%%%%%%%%%%%%%% attititude Kalman %%%%%%%%%%%%%%%%%%%%%%%
    for ii=1:Naccel
        phihat=xhat_att(1);
        thetahat=xhat_att(2);
        G_att=[1,sin(phihat)*tan(thetahat),cos(phihat)*tan(thetahat);
                0,cos(phihat), -sin(phihat)];
        xhat_att=xhat_att+(Toutaccel/Naccel)*G_att*[y_gyro_x;y_gyro_y;y_gyro_z];
        A_att=[y_gyro_y*cos(phihat)*tan(thetahat)-y_gyro_z*sin(phihat)*tan(thetahat),...
            (y_gyro_y*sin(phihat)+y_gyro_z*cos(phihat))/cos(thetahat)^2;
            -y_gyro_y*sin(phihat)-y_gyro_z*cos(phihat),0];
        Qatt=.00001*G_att*diag([0.25*P.std_gyrox^2, 0.25*P.std_gyroy^2, 0.25*P.std_gyroz^2])*G_att';
        Patt=Patt+(Toutaccel/Naccel)*(A_att*Patt+Patt*A_att'+Qatt);
    end
    phihat=xhat_att(1);
    thetahat=xhat_att(2);
    %always have a new measurement
    C1=[0,y_gyro_y*Vahat*cos(thetahat)+P.gravity*cos(thetahat)];
    C2=[-P.gravity*cos(phihat)*cos(thetahat),-y_gyro_z*Vahat*sin(thetahat)-...
        y_gyro_x*Vahat*cos(thetahat)+P.gravity*sin(phihat)*sin(thetahat)];
    C3=[P.gravity*sin(phihat)*cos(thetahat),(y_gyro_y*Vahat+P.gravity*cos(phihat))*sin(thetahat)];

    L1=Patt*C1'*(P.std_accelx^2+C1*Patt*C1')^-1;
    L2=Patt*C2'*(P.std_accely^2+C2*Patt*C2')^-1;
    L3=Patt*C3'*(P.std_accelz^2+C3*Patt*C3')^-1;
    Patt=(eye(2)-L1*C1)*Patt;
    Patt=(eye(2)-L2*C2)*Patt;
    Patt=(eye(2)-L3*C3)*Patt;
%     xhat_att=xhat_att+L1*(y_accel_x-(y_gyro_y*Vahat*sin(thetahat)+...
%         P.gravity*sin(thetahat)));
    xhat_att=xhat_att+L1*(y_accel_x-(y_gyro_y*Vahat*sin(thetahat)+...
        P.gravity*sin(thetahat)));
    xhat_att=xhat_att+L2*(y_accel_y-(y_gyro_z*Vahat*cos(thetahat)-...
        y_gyro_x*Vahat*sin(thetahat)-P.gravity*cos(thetahat)*sin(phihat)));
    xhat_att=xhat_att+L3*(y_accel_z-(y_gyro_y*Vahat*cos(thetahat)-...
        P.gravity*cos(thetahat)*cos(phihat)));
    xhat_att=mod(xhat_att,2*pi);
    xhat_att(xhat_att>pi)=xhat_att(xhat_att>pi)-2*pi;
    phihat=xhat_att(1);
    thetahat=xhat_att(2);
    %%%%%%%%%%%%%%%%%%%% gps Kalman %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    for ii=1:Ngps
        pnhat=xhat_gps(1);
        pehat=xhat_gps(2);
        Vghat=xhat_gps(3);
        chihat=xhat_gps(4);
        wnhat=xhat_gps(5);
        wehat=xhat_gps(6);
        psihat=xhat_gps(7);
        %chihatdot=P.gravity/Vghat*tan(phihat)*cos(chihat-psihat);
        psihatdot=qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat);
        Vghatdot=(((Vahat*cos(psihat)+wnhat)*(-Vahat*psihatdot*sin(psihat)))+...
            ((Vahat*sin(psihat)+wehat)*(Vahat*psihatdot*cos(psihat))))/Vghat;
        
        
        xhat_gps=xhat_gps+(Toutgps/Ngps)*...
            [Vghat*cos(chihat);Vghat*sin(chihat);
            Vghatdot;
            P.gravity/Vghat*tan(phihat)*cos(chihat-psihat);
            0;
            0;
            psihatdot];
        A_gps=[0,0,cos(chihat),-Vghat*sin(chihat),0,0,0;
            0,0,sin(chihat),Vghat*cos(chihat),0,0,0;
            0,0,-Vghatdot/Vghat,0,-psihatdot*Vahat*sin(psihat)/...
            Vghat,psihatdot*Vahat*cos(psihat)/Vghat,-psihatdot*Vahat/Vghat*...
            (wnhat*cos(psihat)+wehat*sin(psihat));
            0,0,-P.gravity/Vghat^2*tan(phihat)*cos(chihat-psihat),-P.gravity/...
            Vghat*tan(phihat)*sin(chihat-psihat),0,0,P.gravity/Vghat*...
            tan(phihat)*sin(chihat-psihat);
            0,0,0,0,0,0,0;
            0,0,0,0,0,0,0;
            0,0,0,0,0,0,0];
        Qgps=diag([0.0000001,0.0000001,0.001,0.0000001,0.0000000001,0.0000001,0.00000001]);
        Pgps=Pgps+(Toutgps/Ngps)*(A_gps*Pgps+Pgps*A_gps'+Qgps);
    end
    if any(gpsPers~=[y_gps_n;y_gps_e;y_gps_h;y_gps_Vg;y_gps_course])
        gpsPers=[y_gps_n;y_gps_e;y_gps_h;y_gps_Vg;y_gps_course];
        pnhat=xhat_gps(1);
        pehat=xhat_gps(2);
        Vghat=xhat_gps(3);
        chihat=xhat_gps(4);
        wnhat=xhat_gps(5);
        wehat=xhat_gps(6);
        psihat=xhat_gps(7);
        C1=[1 0 0 0 0 0 0];
        C2=[0 1 0 0 0 0 0];
        C3=[0 0 1 0 0 0 0];
        C4=[0 0 0 1 0 0 0];
        C5=[0 0 -cos(chihat) Vghat*sin(chihat) 1 0 -Vahat*sin(psihat)];
        C6=[0 0 -sin(chihat) -Vghat*cos(chihat) 0 1 Vahat*cos(psihat)];
        y_gps_wn=Vahat*cos(psihat)+wnhat-y_gps_Vg*cos(y_gps_course);
        y_gps_we=Vahat*sin(psihat)+wehat-y_gps_Vg*sin(y_gps_course);
        L1=Pgps*C1'*(P.std_gpsn^2+C1*Pgps*C1')^-1;
        L2=Pgps*C2'*(P.std_gpse^2+C2*Pgps*C2')^-1;
        L3=Pgps*C3'*(P.std_V^2+C3*Pgps*C3')^-1;
        L4=Pgps*C4'*((P.std_V/Vghat)^2+C4*Pgps*C4')^-1;
        L5=Pgps*C5'*(P.std_V^2+C5*Pgps*C5')^-1;
        L6=Pgps*C6'*(P.std_V^2+C6*Pgps*C6')^-1;
        Pgps=(eye(7)-L1*C1)*Pgps;
        Pgps=(eye(7)-L2*C2)*Pgps;
        Pgps=(eye(7)-L3*C3)*Pgps;
        Pgps=(eye(7)-L4*C4)*Pgps;
        Pgps=(eye(7)-L5*C5)*Pgps;
        Pgps=(eye(7)-L6*C6)*Pgps;
        xhat_gps=xhat_gps+L1*(y_gps_n-pnhat);
        xhat_gps=xhat_gps+L2*(y_gps_e-pehat);
        xhat_gps=xhat_gps+L3*(y_gps_Vg-Vghat);
        xhat_gps=xhat_gps+L4*(y_gps_course-chihat);
        Vghat=xhat_gps(3);
%         chihat=xhat_gps(4);
%         psihat=xhat_gps(7);
        xhat_gps=xhat_gps+L5*(y_gps_wn-(Vahat*cos(psihat)+wnhat-Vghat*cos(chihat)));
        xhat_gps=xhat_gps+L6*(y_gps_we-(Vahat*sin(psihat)+wehat-Vghat*sin(chihat)));

    end
   
    
    xhat_gps(4)=mod(xhat_gps(4),2*pi);
    if xhat_gps(4)>pi
        xhat_gps(4)=xhat_gps(4)-2*pi;
    end
    xhat_gps(7)=mod(xhat_gps(7),2*pi);
    if xhat_gps(7)>pi
        xhat_gps(7)=xhat_gps(7)-2*pi;
    end
    pnhat=xhat_gps(1);
    pehat=xhat_gps(2);
    Vghat=xhat_gps(3);
    chihat=xhat_gps(4);
    wnhat=xhat_gps(5);
    wehat=xhat_gps(6);
    psihat=xhat_gps(7);
    
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        defl1;...
        defl2;...
        defl3;...
        ];
end
