
function delta_c = PD_heading_KF(e_psi, e_r)

    % Kp = 244.4;
    % Kd = 3938;
    
    Kp = 12;
    Kd = 50;
    delta_max = deg2rad(40);

    delta_c = -Kp*e_psi - Kd*e_r;

    delta_c = max(-delta_max, min(delta_max, delta_c));

end
