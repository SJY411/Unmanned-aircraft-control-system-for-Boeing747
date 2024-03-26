function y = autopilot(uu, AP)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   11/14/2014 - RWB
%   2/16/2019 - RWB
%   

    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
    alpha    = uu(5+NN);  % angle of attack
    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    Vg       = uu(13+NN); % ground speed
    wn       = uu(14+NN); % wind North
    we       = uu(15+NN); % wind East
    psi      = uu(16+NN); % heading
    bx       = uu(17+NN); % x-gyro bias
    by       = uu(18+NN); % y-gyro bias
    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    
%     % If phi_c_ff is specified in Simulink model, then do the following
%     phi_c_ff = uu(4+NN);  % feedforward roll command (rad)
%     NN = NN+4;
    
    % If no phi_c_ff is included in inputs in Simulink model, then do this
    NN = NN+3;
    phi_c_ff = 0;
    
    t        = uu(1+NN);   % time
    
    %----------------------------------------------------------
    % lateral autopilot
%     chi_ref = wrap(chi_c, chi);
    chi_ref = chi_c;
    if t==0
        phi_c   = course_with_roll(chi_ref, chi, 0, AP);
        delta_r = 0; % no rudder
    else
        phi_c   = course_with_roll(chi_ref, chi, 1, AP);
        delta_r = 0; % no rudder
    end
    delta_a = roll_with_aileron(phi_c, phi, p, AP);
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
%     h_ref = sat(h_c, h + AP.altitude_zone, h - AP.altitude_zone);
    persistent flight_mode
    if h <= AP.altitude_take_off_zone     
        flight_mode_t = 1;
    elseif h <= h_c - AP.altitude_hold_zone
        flight_mode_t = 2;
    elseif h <= h_c + AP.altitude_hold_zone
        flight_mode_t = 3;
    else
        flight_mode_t = 4;
    end

    % 判断是否需要初始化
    if t==0
        init_flag = 0;
    elseif flight_mode ~= flight_mode_t
        init_flag = 0;
    else
        init_flag = 1;
    end

    % 根据飞行模式采用不同控制策略
    flight_mode = flight_mode_t;
    switch flight_mode
        case 1
            delta_t = 1;
            theta_c = AP.climb_pitch;
        case 2
            delta_t = 1;
            theta_c = airspeed_with_pitch(Va_c, Va, init_flag, AP);
        case 3
            delta_t = airspeed_with_throttle(Va_c, Va, init_flag, AP);
            theta_c = altitude_with_pitch(h_c, h, init_flag, AP);
        case 4
            delta_t = 0;
            theta_c = airspeed_with_pitch(Va_c, Va, init_flag, AP);
    end

    delta_e = pitch_with_elevator(theta_c, theta, q, AP);
    
    % limit range of throttle setting to [0,1]
    delta_t = sat(delta_t, 1, 0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...              % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_with_roll
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c_sat = course_with_roll(chi_c, chi, flag, AP)
  persistent integrator_course
  persistent error_course

  if flag == 0
      integrator_course = 0;
      error_course = 0;
  end
  error = chi_c - chi;
  integrator_course = integrator_course + AP.Ts/2*(error + error_course);
  error_course = error;
  phi_c_sat = sat(AP.course_kp*error + AP.course_ki*integrator_course, AP.e_phi_max, -AP.e_phi_max);
  if AP.course_ki ~= 0
      phi_c_unsat = AP.course_kp*error + AP.course_ki*integrator_course;
      integrator_course = integrator_course + AP.Ts/AP.course_ki*(phi_c_sat - phi_c_unsat);
  end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_with_aileron
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_with_aileron(phi_c, phi, p, AP)
  error = phi_c - phi;
  delta_a = sat(AP.roll_kp*error - AP.roll_kd*p, AP.delta_a_max, -AP.delta_a_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_with_elevator
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_with_elevator(theta_c, theta, q, AP)
  error = theta_c - theta;
  delta_e = sat(AP.pitch_kp*error - AP.pitch_kd*q, AP.delta_e_max, -AP.delta_e_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP)
  persistent integrator_v_t
  persistent error_v_t

  if flag == 0
      integrator_v_t = 0;
      error_v_t = 0;
  end
  error = Va_c - Va;
  integrator_v_t = integrator_v_t + AP.Ts/2*(error + error_v_t);
  delta_t_sat = sat(AP.airspeed_throttle_kp*error + AP.airspeed_throttle_ki*integrator_v_t, 1, 0);
  error_v_t = error;
  if AP.airspeed_throttle_ki ~= 0
      delta_t_unsat = AP.airspeed_throttle_kp*error + AP.airspeed_throttle_ki*integrator_v_t;
      integrator_v_t = integrator_v_t + AP.Ts/AP.airspeed_throttle_ki*(delta_t_sat - delta_t_unsat);
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)
  persistent integrator_h
  persistent error_h

  if flag == 0
      integrator_h = 0;
      error_h = 0;
  end
  error = h_c - h;
  integrator_h = integrator_h + AP.Ts/2*(error + error_h);
  theta_c_sat = sat(AP.altitude_kp*error + AP.altitude_ki*integrator_h, AP.e_theta_max, -AP.e_theta_max);
  if AP.airspeed_pitch_ki ~= 0
      theta_c_unsat = AP.altitude_kp*error + AP.altitude_ki*integrator_h;
      integrator_h = integrator_h + AP.Ts/AP.altitude_ki*(theta_c_sat - theta_c_unsat);
  end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = airspeed_with_pitch(Va_c, Va, flag, AP)
  persistent integrator_v_pitch
  persistent error_v_pitch

  if flag == 0
      integrator_v_pitch = 0;
      error_v_pitch = 0;
  end
  error = Va_c - Va;
  integrator_v_pitch = integrator_v_pitch + (AP.Ts/2)*(error + error_v_pitch);
  theta_c_sat = sat(AP.airspeed_pitch_kp*error + AP.airspeed_pitch_ki*integrator_v_pitch, AP.e_theta_max, -AP.e_theta_max);
  error_v_pitch = error;
  if AP.airspeed_pitch_ki ~= 0
      theta_c_unsat = AP.airspeed_pitch_kp*error + AP.airspeed_pitch_ki*integrator_v_pitch;
      integrator_v_pitch = integrator_v_pitch + AP.Ts/AP.airspeed_pitch_ki*(theta_c_sat - theta_c_unsat);
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function theta_r = sideslip_with_rudder(v, flag, AP)
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% yaw_damper
%   - yaw rate with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function delta_r = yaw_damper(r, flag, AP)
% end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
    if in > up_limit, out = up_limit;
    elseif in < low_limit, out = low_limit;
    else, out = in;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function chi_c = wrap(chi_c, chi)
if chi_c - chi > pi, chi_c = chi_c - 2*pi;
elseif chi_c - chi < -pi, chi_c = chi_c + 2*pi;
end
end

  
 