clear; close all; clc;

load mA;
m = 1667;
L = 5.0;
g = 9.81;

bis.m_11 = mA.m_11/m;
bis.m_22 = mA.m_22/m;
bis.m_33 = mA.m_33/(m*L^2);
bis.X_u = mA.X_u/bis_force_coeff_scaling(1,0,m,L,g);
bis.X_auu = mA.X_auu/bis_force_coeff_scaling(2,0,m,L,g);
bis.X_uuu = mA.X_uuu/bis_force_coeff_scaling(3,0,m,L,g);
bis.Y_v = mA.Y_v/bis_force_coeff_scaling(1,0,m,L,g);
bis.Y_avv = mA.Y_avv/bis_force_coeff_scaling(2,0,m,L,g);
bis.Y_vvv = mA.Y_vvv/bis_force_coeff_scaling(3,0,m,L,g);
bis.Y_arv = mA.Y_arv/bis_force_coeff_scaling(1,1,m,L,g);
bis.Y_r = mA.Y_r/bis_force_coeff_scaling(0,1,m,L,g);
bis.Y_avr = mA.Y_avr/bis_force_coeff_scaling(1,1,m,L,g);
bis.Y_arr = mA.Y_arr/bis_force_coeff_scaling(0,2,m,L,g);
bis.Y_ur = mA.Y_ur/bis_force_coeff_scaling(1,1,m,L,g);
bis.N_v = mA.N_v/bis_moment_coeff_scaling(1,0,m,L,g);
bis.N_avv = mA.N_avv/bis_moment_coeff_scaling(2,0,m,L,g);
bis.N_arv = mA.N_arv/bis_moment_coeff_scaling(1,1,m,L,g);
bis.N_r = mA.N_r/bis_moment_coeff_scaling(0,1,m,L,g);
bis.N_arr = mA.N_arr/bis_moment_coeff_scaling(0,2,m,L,g);
bis.N_avr = mA.N_avr/bis_moment_coeff_scaling(1,1,m,L,g);
bis.N_uv = mA.N_uv/bis_moment_coeff_scaling(2,0,m,L,g);
bis.N_ur = mA.N_ur/bis_moment_coeff_scaling(1,1,m,L,g);

save('bis.mat', 'bis');