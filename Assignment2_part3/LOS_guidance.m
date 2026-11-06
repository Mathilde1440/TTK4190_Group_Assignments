%LOS Guidance


function chi_d = LOS_guidance(pi_h, y_ep)
L_pp = 161;
look_ahead = 1.5*L_pp;  %tuning parameter, oftent values from 1.5 -> 2.5 of shiplength

Kp = 1 / look_ahead;


chi_d = pi_h - atan2( y_ep,Kp);

end