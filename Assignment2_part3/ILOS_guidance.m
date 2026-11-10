%ILOS Guidance
function chi_d = ILOS_guidance(pi_h, y_ep, h)

    persistent y_int
    if isempty(y_int)
        y_int = 0;
    end

    L_pp = 161;
    look_ahead = 6*L_pp;  %tuning parameter
    kappa = 0.5;          %tuning parameter
    
    Kp = 1 / look_ahead;
    Ki = kappa*Kp;
    
    chi_d = pi_h - atan( y_ep* Kp + Ki*y_int);

    y_int_dot = (look_ahead*y_ep) / (look_ahead^2 + (y_ep + kappa*y_int)^2);
    y_int = y_int + y_int_dot*h;

     %y_int

end