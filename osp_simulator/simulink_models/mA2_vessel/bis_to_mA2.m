clear; close all; clc;

load bis;
m = 4807;
L = 8.3;
g = 9.81;

mA2.m_11 = bis.m_11*m;
mA2.m_22 = bis.m_22*m;
mA2.m_33 = bis.m_33*m*L^2;
mA2.X_u = bis.X_u*bis_force_coeff_scaling(1,0,m,L,g);
mA2.X_auu = bis.X_auu*bis_force_coeff_scaling(2,0,m,L,g);
mA2.X_uuu = bis.X_uuu*bis_force_coeff_scaling(3,0,m,L,g);
mA2.Y_v = bis.Y_v*bis_force_coeff_scaling(1,0,m,L,g);
mA2.Y_avv = bis.Y_avv*bis_force_coeff_scaling(2,0,m,L,g);
mA2.Y_vvv = bis.Y_vvv*bis_force_coeff_scaling(3,0,m,L,g);
mA2.Y_arv = bis.Y_arv*bis_force_coeff_scaling(1,1,m,L,g);
mA2.Y_r = bis.Y_r*bis_force_coeff_scaling(0,1,m,L,g);
mA2.Y_avr = bis.Y_avr*bis_force_coeff_scaling(1,1,m,L,g);
mA2.Y_arr = bis.Y_arr*bis_force_coeff_scaling(0,2,m,L,g);
mA2.Y_ur = bis.Y_ur*bis_force_coeff_scaling(1,1,m,L,g);
mA2.N_v = bis.N_v*bis_moment_coeff_scaling(1,0,m,L,g);
mA2.N_avv = bis.N_avv*bis_moment_coeff_scaling(2,0,m,L,g);
mA2.N_arv = bis.N_arv*bis_moment_coeff_scaling(1,1,m,L,g);
mA2.N_r = bis.N_r*bis_moment_coeff_scaling(0,1,m,L,g);
mA2.N_arr = bis.N_arr*bis_moment_coeff_scaling(0,2,m,L,g);
mA2.N_avr = bis.N_avr*bis_moment_coeff_scaling(1,1,m,L,g);
mA2.N_uv = bis.N_uv*bis_moment_coeff_scaling(2,0,m,L,g);
mA2.N_ur = bis.N_ur*bis_moment_coeff_scaling(1,1,m,L,g);

save('mA2.mat', 'mA2');