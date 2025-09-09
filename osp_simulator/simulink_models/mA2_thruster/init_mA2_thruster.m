%Placement
Lx = -2.85;
Ly = -0.7;

%Servo dynamics
max_turn_rate = 30;%deg/s
servo_time_constant = 0.5;%s
neutral_angle = 45;%[deg]
max_angle_displacement = 45;%[deg]

%Thruster dynamics
max_rpm = 1200;
max_rpm_rate = 1200;
rpm_time_constant = 1.0;

%Open-water propeller characteristics
propeller_diameter = 0.36;

%Four quadrant model (Wageningen B4-70 propeller)
AT = [0.0254,0.1782,0.0147,0.0281,-0.0163,-0.0530,0.0006,0.0368,-0.0025, ...
      -0.0177,0.0027,0.0214,-0.0025,0.0012,0.0051,0.0078,-0.0038,0.0035,0.0053,0.0022,-0.0028];
BT = [0,-0.7478,-0.0138,0.1008,-0.0113,0.0472,0.0107,-0.0090,-0.0078, ...
      0.0239,0.0081,-0.0001,-0.0032,0.0093,0.0016,-0.0066,-0.0006,0.0051,-0.0006,-0.0082,-0.0006];
AQ = [0.0025,0.0267,0.0016,0.0066,-0.0022,-0.0078,0.0002,0.0061,-0.0016, ...
      -0.0033,0.0012,0.0031,-0.0013,0.0014,0.0009,0.0001,-0.0008,0.0013,0.0012,-0.0001,-0.0007];
BQ = [0,-0.1108,0.0002,0.0165,-0.0021,0.0085,0.0009,-0.0031,-0.0010,0.0043, ...
      0.0012,0.0001,-0.0008,0.0015,0.0002,-0.0017,0.0005,0.0012,-0.0005,-0.0015,0.0002];


% rpm = -1200:1:1200;
% for i = 1:length(rpm)
%     [T(i),Q(i)] = four_quadrant_propeller(AT,BT,AQ,BQ,2*pi*rpm(i)/60,-2.0,propeller_diameter,1025);
% end
% 
% figure;
% plot(rpm,T); hold on; plot(rpm,Q)
% 
% beta = -pi:0.01:pi;
% CT = zeros(length(beta),1); CQ = CT;
% for i = 1:length(beta)
%     for k = 0:length(AT)-1
%         CT(i) = CT(i) + AT(k+1)*cos(beta(i)*k) + BT(k+1)*sin(beta(i)*k);
%         CQ(i) = CQ(i) + AQ(k+1)*cos(beta(i)*k) + BQ(k+1)*sin(beta(i)*k);
%     end
% end
% figure;
% plot(beta, CT); hold on; plot(beta, 10*CQ);