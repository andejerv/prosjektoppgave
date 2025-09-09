function [detection_distance, valid_detection] = ultrasonic_sensor(eta, range, quays, T_s_b)
% ULTRASONIC_SENSOR Generate ultrasonic distance measurement to quay(s)
%   Inputs the current 3dof pose of the vessel, 
%   the min and max range of the sensor, a number of quay definitions in
%   NED, and finally transformations between sensor and body, as well as
%   body and ned frames. Outputs the detection distance, as well as whether
%   this detection is within the configured range.
%
%   Inputs:
%   - eta   - (3, 1) double - 3 DOF vessel position in NED [m, m, rad]
%   - range - (2, 1) double - min and max range in meters. [m, m]
%   - quays - (n, 1) struct - {line: (2,2) double, depth: double}
%       The two quay line points should be given in NED with decreasing
%       angles with respect to the quay interior - i.e. in counter-clockwise 
%       direction when looking down at the quay.
%   - T_s_b - (3, 3) double - 
%       Homogenous transform from the sensor to the body frame, used to
%       build the matrix T_s_n for projecting the sensor detection line
%       into the NED frame.
%
%   Outputs:
%    - detection_distance - double [m]
%       Distance to closest detection within configured range
%       defaults to max range if no detection, min range if 'too close' detection
%    - valid_detection - boolean - Flag to denote whether returned detection is within nominal range 

% build sensor to NED frame transform:
T_b_n = [
    cos(eta(3)), -sin(eta(3)), eta(1);
    sin(eta(3)),  cos(eta(3)), eta(2);
              0,            0,      1
];

T_s_n = T_b_n*T_s_b;

% detection line in NED frame
D_s = [[0, range(2)]; zeros(1,2); ones(1,2)];
det_line_ned = T_s_n*D_s;

% default 'no detection' return values
detection_distance = range(2);
valid_detection = false;

% foreach quay in quays
num_quays = numel([quays.depth]);

for i=1:num_quays
    % Build quay poly
    quay = quays(i);

    % angle between x-axis of NED frame and quay front edge (i.e. x-axis of quay frame)
    quay_dir = atan2(quay.line(2,2) - quay.line(2,1), ... % p1y - p0y 
                     quay.line(1,2) - quay.line(1,1));    % p1x - p0x 

    R_q_n = [   cos(quay_dir), -sin(quay_dir)
                sin(quay_dir),  cos(quay_dir)];

    quay_poly = [quay.line(:,2) - R_q_n*[0; quay.depth], ...
                 quay.line(:,2), ...
                 quay.line(:,1), ...
                 quay.line(:,1) - R_q_n*[0; quay.depth]];
    
    % check for intersection within max range of sensor 
    [intersection, t] = line_poly_intersection( ...
        det_line_ned(1:2,1), det_line_ned(1:2,2), quay_poly ...
    );

    if numel(intersection) == 0
        continue;
    end

    % check whether this is closer than previous closest
    % some superfluous indexing here to maintain unambiguous dimensions on output
    if range(2)*t(1,1) <= detection_distance
        detection_distance = range(2)*t(1,1);
        valid_detection    = true;
    end
end

% check whether the closest detection is closer than min range
if detection_distance < range(1)
    valid_detection    = false;
    detection_distance = range(1);
end

end