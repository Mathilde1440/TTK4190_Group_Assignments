function xd_dot = ref_model(xd, psi_ref)

    omega_ref = 0.03;
    zeta = 1;

    psi_d = xd(1);
    r_d = xd(2);
    a_d = xd(3);

    psi_d_dot = r_d;
    r_d_dot = a_d;

    a_d_dot = omega_ref^3*(psi_ref - psi_d) ...
            - (1 + 2*zeta)*omega_ref^2*r_d ...
            - (2*zeta + 1)*omega_ref*a_d;

    xd_dot = [psi_d_dot; r_d_dot; a_d_dot];

end