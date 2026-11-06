%LOS Guidance


function chi_d = LOS_guidance(pi_h, y_ep)
L_pp = 161;
look_ahead = 9*L_pp;  %tuning parameter, oftent values from 1.5 -> 2.5 of shiplength

Kp = 1 / look_ahead;


chi_d = pi_h - atan( y_ep* Kp);
chi_d = atan2(sin(chi_d), cos(chi_d));

end