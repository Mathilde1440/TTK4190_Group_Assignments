function delta_c = PID_heading(e_psi, e_r, e_int)

    Kp = 244.4;
    Kd = 3938;
    Ki = 4.887;

    delta_c = Kp*e_psi + Kd*e_r + Ki*e_int;

end