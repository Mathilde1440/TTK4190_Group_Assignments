%LOS Guidance
function chi_d = LOS_guidance(pi_h, y_ep)
L_pp = 161;
look_ahead = 6*L_pp;  %tuning parameter

Kp = 1 / look_ahead;


chi_d = pi_h - atan( y_ep* Kp);

end