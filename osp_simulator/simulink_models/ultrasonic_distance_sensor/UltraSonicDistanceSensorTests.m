% tests for the ultrasonic distance sensor,

% This function accepts the following parameters:
%   eta   - (3, 1) double - north, east, heading [m, m, rad] which is the vessel coordinates in the NED frame
%   range - (2, 1) double - min and max sensor range [m, m]
%   quays - (n, 1) struct - vector of structs {line: (2, 2) double, depth: double}, one for each quay, in the NED frame
%   T_s_b - (3, 3) double - (x,y,1) homogenous transform matrix from sensor to body frame

% Expected behaviour of the function: 
%   each quay in the quays vector implicitly defines a square polygon based on its forward face being
%   from p0 = quay.line(:, 1) to p1 = quat.line(:,2), and extending for quay.depth in a counterclockwise fashion, i.e.
%  As seen from above in a NED frame
%  N
%  ^  p3 ------ p2  \ 
%  |   |         |   }depth
%  |   |         |  /
%  |  p0 ------ p1 /
%  +-----> E
% the ultrasonic distance sensor is assumed to be facing in the x direction of the sensor frame,
% and is able to provide an exact measurement of the distance to the closest quay edge within [range(1), range(2)]
% this is represented by two return values:
%   distance_measurement - double - distance to closest quay edge if anything within range, returns range(2) if no detection, returns range(1) if detection closer than range(1) 
%   valid_signal         - bool   - true if measurement is within the nominal range, i.e. [range(1), range(2)] 

% These tests are structured in a per-parameter sense, and aims to exhaustively test each parameter separately,
% then finally multiple parameters in conjuntion
% Furthermore, the quays parameter is implicitly tested via the other parameters.
%
% TODO - What's missing from this is some testing of non straight quays (i.e. not facing in a cardinal direction), 
% as well as quays with a depth different than 1. This would also be good for testing near paralell edges to the sensor line. 

%% Prepare tests
clearvars
nearZero = @(x) abs(x) < 1e-9; % an arbitrary 4x of the precision of the provided param

%% TESTS

% ---------------------------- tests of range parameter ------------------------------
range = [0.15, 1];
D_too_close = 0.1;
D_in_range  = 0.5;
D_too_far   = 1.1;
eta = zeros(3,1); % vessel facing north in origin of ned frame
T_s_b = eye(3);   % sensor mounted in origin of body frame
% => sensor facing north in origin of ned frame 

    % Single quay within nominal range, p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [D_in_range, D_in_range; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - D_in_range), valid]));

    % Single quay inside of min range, p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [D_too_close, D_too_close; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - range(1)), ~valid]));

    % Single quay outside of max range, p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [D_too_far, D_too_far; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

    % Sensor inside of quay, p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [-0.5, -0.5; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - range(1)), ~valid]));

    % Quay behind sensor, p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [-D_too_far, -D_too_far; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

    % Multiple quays partially obscuring eachother, both quays with p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [0.5, 0.5; -1, 1], "depth", 1); ...
            struct("line", [0.4, 0.4; -0.5, 0.5], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - 0.4), valid]));

    % Multiple quays partially obscuring eachother, flipped, both quays with p0 -> p1 in negative y direction of ned frame
    quays = [struct("line", [0.4, 0.4; -0.5, 0.5], "depth", 1); ...
            struct("line", [0.5, 0.5; -1, 1], "depth", 1)];
    [dist, valid] = ultrasonic_sensor(eta, range, quays, T_s_b);
    assert(all([nearZero(dist - 0.4), valid]));

% 

% ---------------------------- tests of T_s_b parameter ------------------------------
range = [0.15, 1];
eta = zeros(3,1); % vessel facing north in origin of ned frame

% a set of 1x1 m quays, one in each cardinal direction from the origin with a unique distance from the origin
% each quay here will have its forward edge (p0 -> p1) facing the origin
D_north = 0.2;
D_east  = 0.4;
D_south = 0.6;
D_west  = 0.8;  

unique_direction_quays = [...
    struct("line", [D_north, D_north;    -0.5,   0.5], "depth", 1); ... % north, distance of 0.2
    struct("line", [    0.5,    -0.5; D_east, D_east], "depth", 1); ... % east, distance of 0.4
    struct("line", [-D_south, -D_south;   0.5,  -0.5], "depth", 1); ... % south, distance of 0.6
    struct("line", [   -0.5,    0.5; -D_west, -D_west], "depth", 1) ... % west, distance of 0.8
];

ang_north = 0;
ang_northeast = pi/4;
ang_east = pi/2;
ang_southeast = 3*pi/4;
ang_south = pi;
ang_southwest = -3*pi/4;
ang_west = -pi/2;
ang_northwest = -pi/4;

T_2dof = @(x, y, ang) [cos(ang), -sin(ang), x; sin(ang), cos(ang), y; 0, 0, 1];

% Starting with simple rotations

    % Sensor facing north
    T_s_b = T_2dof(0, 0, ang_north);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_north), valid]));

    % Sensor facing northeast
    T_s_b = T_2dof(0, 0, ang_northeast);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_north), valid]));

    % Sensor facing east
    T_s_b = T_2dof(0, 0, ang_east);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_east), valid]));

    % Sensor facing southeast
    T_s_b = T_2dof(0, 0, ang_southeast);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_east), valid]));

    % Sensor facing south
    T_s_b = T_2dof(0, 0, ang_south);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_south), valid]));

    % Sensor facing southwest
    T_s_b = T_2dof(0, 0, ang_southwest);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

    % Sensor facing west
    T_s_b = T_2dof(0, 0, ang_west);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_west), valid]));

    % Sensor facing northwest
    T_s_b = T_2dof(0, 0, ang_northwest);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_north), valid]));

% Compound translation and rotation

    % Moved past north quay and facing south, should pick up northern end of north quay
    T_s_b = T_2dof(2, 0, ang_south);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - (2 - D_north - 1)), valid]));

    % Moved into north quay and facing west, should pick up invalid too
    % close as inside of quay
    T_s_b = T_2dof(1, 0, ang_west);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(1)), ~valid]));

    % Moved southeast and facing west, should pick up nothing as eastern end of west quay is too far away
    T_s_b = T_2dof(-0.3, 0.3, ang_west);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

% ---------------------------- tests of eta parameter ------------------------------
range = [0.15, 1];
T_s_b = eye(3);

% Reusing unique_direction_quays from previous tests

% Starting with simple rotations

    % Sensor facing north
    eta = [0; 0; ang_north];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_north), valid]));

    % Sensor facing northeast
    eta = [0; 0; ang_northeast];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_north), valid]));

    % Sensor facing east
    eta = [0; 0; ang_east];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_east), valid]));

    % Sensor facing southeast
    eta = [0; 0; ang_southeast];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_east), valid]));

    % Sensor facing south
    eta = [0; 0; ang_south];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_south), valid]));

    % Sensor facing southwest
    eta = [0; 0; ang_southwest];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

    % Sensor facing west
    eta = [0; 0; ang_west];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_west), valid]));

    % Sensor facing northwest
    eta = [0; 0; ang_northwest];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - sqrt(2)*D_north), valid]));

% Compound translation and rotation

    % Moved past north quay and facing south, should pick up northern end of north quay
    eta = [2; 0; ang_south];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - (2 - D_north - 1)), valid]));

    % Moved into north quay and facing west, should pick up western end of north quay 
    eta = [1; 0; ang_west];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(1)), ~valid]));

    % Moved southeast and facing west, should pick up nothing as eastern end of west quay is too far away
    eta = [-0.3; 0.3; ang_west];
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

% ---------------------------- compound tests of eta and T_s_b ------------------------------
range = [0.15, 1];

% Reusing unique_direction_quays from previous tests

% Simple rotations: 

    % vessel northeast and sensor northeast, compound east
    eta = [0; 0; ang_northeast];
    T_s_b = T_2dof(0, 0, ang_northeast);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_east), valid]));

    % vessel east and sensor south, compound west
    eta = [0; 0; ang_east];
    T_s_b = T_2dof(0, 0, ang_south);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_west), valid]));

    % vessel south and sensor south, compound north
    eta = [0; 0; ang_south];
    T_s_b = T_2dof(0, 0, ang_south);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - D_north), valid]));

% Compound translation and rotation

    % vessel moved and facing northeast, sensor moved and facing northeast, compound [1, 1+sqrt(2)*0.2, east]
    % Should pick up nothing as we have moved past both north and east quays while facing east
    eta = [1; 1; ang_northeast];
    T_s_b = T_2dof(0.2, 0.2, ang_northeast);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - range(2)), ~valid]));

    % vessel moved and facing northeast, sensor moved northeast and facing southwest, compound [1, 1+sqrt(2)*0.2, west]
    % Should pick eastern side of north quay as we have moved past both north and east quays while facing west
    eta = [1; 1; ang_northeast];
    T_s_b = T_2dof(0.2, 0.2, ang_southwest);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - (1 + sqrt(2)*0.2 - 0.5)), valid]));

    % vessel moved northeast facing south, moved northeast facing south, compound [-0.5, -0.5, north]
    eta = [1; 1; ang_south];
    T_s_b = T_2dof(1.5, 1.5, ang_south);
    [dist, valid] = ultrasonic_sensor(eta, range, unique_direction_quays, T_s_b);
    assert(all([nearZero(dist - (D_north + 0.5)), valid]));