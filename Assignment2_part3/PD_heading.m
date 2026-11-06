function delta_c = PD_heading(e_psi, e_r)

    Kp = 244.4;
    Kd = 3938;


    delta_c = Kp*e_psi + Kd*e_r ;

end