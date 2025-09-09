clear all; close all; clc;

path = "C:\Users\tobiasvt\OneDrive - NTNU\PhD\Projects\milliAmpere digital twin\milliampere_osp_sim\osp_system\logs\milliAmpere_0_20200216_174500.csv";
ts = create_timeseries(path);

%plot(ts("eta[2]"),ts("eta[1]"), 'r', 'LineWidth', 2)
axis equal
grid on
xlabel('East [m]')
ylabel('North [m]')
rectangle('Position',[0 0 10 10], 'LineWidth',2)
title('4-corner DP Test')
axis([-2 12 -2 12])
%print('figures/four_corner', '-depsc')

path = "C:\Users\tobiasvt\OneDrive - NTNU\PhD\Projects\milliAmpere digital twin\milliampere_osp_sim\osp_system\logs\milliAmpere_0_20200216_180716.csv";
ts = create_timeseries(path);
hold on;
plot(ts("eta[2]"),ts("eta[1]"), 'LineWidth', 2)

path = "C:\Users\tobiasvt\OneDrive - NTNU\PhD\Projects\milliAmpere digital twin\milliampere_osp_sim\osp_system\logs\milliAmpere_0_20200216_181139.csv";
ts = create_timeseries(path);
hold on;
plot(ts("eta[2]"),ts("eta[1]"), 'LineWidth', 2)

