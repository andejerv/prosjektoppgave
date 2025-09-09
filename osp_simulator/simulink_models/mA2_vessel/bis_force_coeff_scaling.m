function k = bis_force_coeff_scaling(vel_degree, rot_degree, m , L, g)
    k_m = m;
    k_l = L;
    k_t = sqrt(L/g);
    x = 1-vel_degree;
    y = 2 - vel_degree - rot_degree;
    k = (k_m*k_l^x) / (k_t^y);
end