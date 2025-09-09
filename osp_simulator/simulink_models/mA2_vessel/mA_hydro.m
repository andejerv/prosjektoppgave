clear; close all; clc;

mA.m_11 = 2390;
mA.m_22 = 2448;
mA.m_33 = 4862;
mA.X_u = -106.6;
mA.X_auu = -21.39;
mA.X_uuu = -37.43;
mA.Y_v = -29.44;
mA.Y_avv = -172.9;
mA.Y_vvv = -1.338;
mA.Y_arv = -1517;
mA.Y_r = 62.58;
mA.Y_avr = 488.7;
mA.Y_arr = -198.2;
mA.Y_ur = 77.58;
mA.N_v = 7.34;
mA.N_avv = -4.352;
mA.N_arv = 437.8;
mA.N_r = -142.7;
mA.N_arr = -831.7;
mA.N_avr = -122;
mA.N_uv = -90.97;
mA.N_ur = 178.5;

save('mA.mat', 'mA');