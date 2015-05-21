% select gains for roll loop
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 45*pi/180;
    % Roll command when delta_a_max is achieved
    phi_max = 15*pi/180;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_roll = 0.707;
    wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
    
    % set control gains based on zeta and wn
    P.roll_kp = delta_a_max/(30*pi/180);
    P.roll_kd = (2*zeta_roll*wn_roll-a_phi1)/a_phi2;
    P.roll_ki = P.roll_kp/20;
    
% select gains for course loop
   wn_course = wn_roll/30;
   zeta_course = 1;
   
   P.course_kp = 2*zeta_course*wn_course*P.Va/P.gravity;
   P.course_ki = wn_course^2*P.Va/P.gravity;
   P.course_kd = 0;
   
% select gains for sideslip hold
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_v_delta_r,'v');
    a_beta2 = num(2);
    a_beta1 = -den(2);
    % maximum possible rudder command
    delta_r_max = 45*pi/180;
    % sideslip command when delta_a_max is achieved
    beta_c = 15*pi/180;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_side_slip = 0.707;
    wn_side_slip = ((delta_r_max/beta_c)*a_beta2 + a_beta1)/(2*zeta_side_slip);

    P.beta_kp = delta_r_max/beta_c*sign(a_beta2);
    %P.beta_kp = (2*zeta_side_slip*wn_side_slip - a_beta1)/a_beta2;
    P.beta_ki = (1/a_beta2)*((a_beta1+a_beta2*P.beta_kp)/(2*zeta_side_slip))^2;
    %P.beta_ki = wn_sild_slip^2/a_beta2;
    P.beta_kd = 0;

% select gains for the pitch loop
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta2 = den(3);
   a_theta1 = den(2);
   a_theta3 = num(3);
   % maximum possible elivator command
   delta_e_max = 45*pi/180;
   % pitch command when delta_a_max is achieved
   theta_max = 15*pi/180;
   % pick natural frequency to achieve delta_a_max for step of phi_max
   zeta_pitch = 0.707;
   wn_pitch = sqrt(a_theta2+abs(a_theta3)*delta_e_max*sqrt(1-zeta_roll^2)/theta_max);
   
   P.pitch_kp = delta_e_max/(15*pi/180)*sign(a_theta3);
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;
   P.pitch_ki = 0.0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   wn_h = wn_pitch/10;
   
   P.altitude_kp = (2*.9*wn_h)/(P.K_theta_DC*P.Va);
   P.altitude_ki = wn_h^2/(P.K_theta_DC*P.Va);
   P.altitude_kd = 0;
 
% airspeed hold using pitch
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   a_V2 = num(2);
   wn_airspeed_pitch = wn_pitch/10;
   
   P.airspeed_pitch_kp = (a_V1 - 2*zeta_pitch*wn_airspeed_pitch)/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_kd = 0;
 
% airspeed hold using throttle
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throt = .707;
   wn_airspeed_throt = wn_side_slip;
   
   P.airspeed_throttle_kp = (2*zeta_airspeed_throt*wn_airspeed_throt - a_Vt1)/a_Vt2;
   P.airspeed_throttle_ki = wn_airspeed_throt^2/a_Vt2*50;
   P.airspeed_throttle_kd = 0;
 


