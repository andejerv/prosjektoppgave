function quay_tau = quay_tau(eta, nu, tau, quays, par)
%quay_tau Calculate quay interaction forces
%   Inputs the current position, velocity (absolute), total force acting on
%   the vessel, and quay parameters.
%
%   Inputs:
%   - eta: 6 DOF vessel position in NED (only North, East and yaw is used)
%   - nu: 6 DOF vessel velocity in body (only surge, sway and yaw is used)
%   - tau: 6 DOF generalized force acting on the vessel in body. This
%       should include all forces, including hydrodynamic forces.
%   - quays: 
%       The quays input should be a struct array with the fields line and
%       depth. The two quay line points should be given in NED with decreasing
%       angles with respect to the quay interior - i.e. in counter-clockwise 
%       direction when looking down at the quay.
%   - par:
%       The par input should contain:
%       - mu: The friction coefficient
%       - k: The spring coefficient
%       - d: The dampning coefficient
%       - v_static: Velocity threshold to choose between Coloumb friction
%           and stiction.
%       - stiction_damping: If true, a linear interpolation of the Coloumb 
%           friction inside of the v_static domain is added to the stiction
%           force. This can be useful for making the sliding velocity
%           converge to zero.

% Store vriables for quay forces and contact points in body-frame
num_quays = numel([quays.depth]);
quay_normal_forces = nan(2, num_quays);
quay_contact_points = nan(2, num_quays);

for i=1:num_quays
    % Check interaction with the i-th quay
    quay = quays(i);
    [quay_normal_force, quay_contact_point] = quay_interaction(eta, nu, par.vessel_polygon, quay.line(:,1), quay.line(:,2), quay.depth, par.k, par.d);

    % Add quay normal force and contact point to storage variables
    quay_normal_forces(:,i) = quay_normal_force;
    quay_contact_points(:,i) = quay_contact_point;
end

% Calculate total normal force and the induced torque
quay_normal_force = sum(quay_normal_forces, 2);
quay_normal_force_torque = sum(...
    cross(...
        [quay_contact_points ; zeros(1, num_quays)], ...
        [quay_normal_forces; zeros(1, num_quays)] ...
    ), ...
2);
if norm(quay_normal_force) < 1e-6
    % No interaction
    quay_tau = zeros(6,1);
    return
end

% Compute friction force - contact point is weighted mean of normal force
% contact points
quay_contact_point = sum(quay_contact_points.*vecnorm(quay_normal_forces),2) ...
    /sum(vecnorm(quay_normal_forces));
[friction_force, friction_yaw_torque] = calculate_friction(nu, tau, quay_normal_force, quay_contact_point, par.mu, par.v_static, par.stiction_damping);

% Compute total force and resulting torque in 6 DOF
quay_force = [quay_normal_force + friction_force; 0];
quay_torque = quay_normal_force_torque + [0; 0; friction_yaw_torque];

quay_tau = [quay_force ; quay_torque];
end

function [quay_normal_force, quay_contact_point] = quay_interaction(eta, nu, vessel_polygon_body, quay_p0, quay_p1, quay_depth, quay_k, quay_d)
%quay_interaction Calculate interaction (normal force) for a single quay
%   Inputs the current position, velocity (absolute), vessel polygon, and 
%   quay parameters.
%
%   Inputs:
%   - eta: 6 DOF vessel position in NED (only North, East and yaw is used)
%   - nu: 6 DOF vessel velocity in body (only surge, sway and yaw is used)
%   - vessel_polygon_body: Vessel polygon given in counter-clockwise
%       direction. Should be of size (2,N)
%   - quay_p0: Quay line start point in NED
%   - quay_p1: Quay line end point in NED
%   - quay_depth: Quay depth
%   - quay_k: Quay spring constant
%   - quay_k: Quay dampning constant
%
%   Returns
%   - normal_force: Normal force in the x-y plane of the BODY frame.
%       If no interaction is found, [0, 0]' is returned
%   - contact_point: Contact point in the x-y plane of the BODY frame.
%       If no interaction is found, [0, 0]' is returned

quay_normal_force = [0, 0]';
quay_contact_point = [0, 0]';

% Prepare transformimg coordinates to Quay frame with origo at first
% quay point and x-axis pointing along quay line
diff = quay_p1 - quay_p0;
quay_ang = atan2(diff(2), diff(1));

quay_origo_ned = quay_p0;
R_q_n = Rzyx(0, 0, quay_ang)'; % Rotation matrix from ned to quay
R_q_n = R_q_n(1:2,1:2);
R_n_b = Rzyx(0, 0, eta(6)); % Rotation matrix from body to ned
R_n_b = R_n_b(1:2,1:2);
R_q_b = R_q_n * R_n_b; % Rotation matrix from quay to body

% Quay polygon
quay_len = norm(diff);
quay_polygon = [
    [quay_len, -quay_depth]
    [quay_len, 0]
    [0, 0]
    [0, -quay_depth]
]';

% Vessel position, speed and polygon in quay frame
p_vessel = R_q_n * (eta(1:2) - quay_origo_ned);
v_vessel = R_q_b * nu(1:2);
vessel_polygon = p_vessel + R_q_b * vessel_polygon_body;

% Loop over vessel edges and calculate force for each edge
dock_interaction = false;
f_n = 0;  % Normal force for current quay
p_n = zeros(2,1);
v_n = zeros(2,1);
for k = 0:(size(vessel_polygon,2) - 1)
    % Calculate intersection between
    k0 = mod(k-1, size(vessel_polygon,2))+1;
    k1 = mod((k+1)-1, size(vessel_polygon,2))+1;
    p_edge0 = vessel_polygon(:, k0);
    p_edge1 = vessel_polygon(:, k1);
    [intersection, ~] = line_poly_intersection(p_edge0, p_edge1, quay_polygon);
    if numel(intersection) == 0
        % No intersection for current edge
        continue
    end
    dock_interaction = true;

    % Calculate speed at contact point
    p0 = intersection(:,1);
    p1 = intersection(:,2);
    v0 = calc_vel(p0, p_vessel, nu, R_q_b);
    v1 = calc_vel(p1, p_vessel, nu, R_q_b);

    % Calculate normal force and contact point for line segment
    [f_segment, p_segment, v_segment] = normal_force(p0, v0, p1, v1, quay_k, quay_d);
    f_n = f_n + f_segment;
    p_n = p_n + f_segment * p_segment;
    v_n = v_n + f_segment * v_segment;
end

if dock_interaction == false
    % No interaction with quay
    return
end

% Calculate mean contact point
if abs(f_n) < 1e-6
    % Avoid division by zero - we are likely just touching the quay
    % getting a single interaction point.
    return
end
p_n = p_n / f_n;
v_n = v_n / f_n;

% Add quay normal force and contact point to storage variables
quay_normal_force = R_q_b'*[0;f_n];
quay_contact_point = R_q_b'*(-p_vessel + p_n);
end

function [friction_force, friction_yaw_torque] = calculate_friction(nu, tau, quay_normal_force, quay_contact_point, quay_mu, v_static, stiction_damping)
%calculate_friction Compute friction force given quay normal force and 
% contact point
% 
%   Inputs
%   - nu: 6 DOF vessel velocity in body (only surge, sway and yaw is used)
%   - tau: 6 DOF generalized force acting on the vessel in body. This
%       should include all forces, including hydrodynamic forces.
%   - quay_normal_force: Normal force in the x-y plane of the BODY frame.
%   - quay_contact_point: Contact point in the x-y plane of the BODY frame.
%   - quays_mu: The quay friction coefficient
%   - v_static: Velocity threshold to choose between Coloumb friction and 
%       stiction.
%   - stiction_damping: If true, a linear interpolation of the Coloumb 
%       friction inside of the v_static domain is added to the stiction
%       force. This can be useful for making the sliding velocity
%       converge to zero.
%
%   Returns
%   - friction_force: Friction force in the x-y plane of the BODY frame
%   - friction_torque: Yaw friction moment

ang = atan2(quay_normal_force(2), quay_normal_force(1)) + pi/2; % Friction force angle in body frame
e_f = [cos(ang) ; sin(ang)]; % Friction force unit vector
f_f_max = quay_mu * norm(quay_normal_force);

slide_velocity = dot(nu(1:2), e_f);
if abs(slide_velocity) < v_static
    % Vessel not sliding - counteract applied force
    f_f = - dot(tau([1,2]), e_f);
    if stiction_damping
        % Add linear damping
        d = -f_f_max/v_static;
        f_f = f_f + d*slide_velocity;
    end
    % Saturate friction force
    f_f = max(min(f_f, f_f_max), -f_f_max);
else
    % Vessel sliding, apply friction force
    f_f = - sign(slide_velocity) * f_f_max;
end
friction_force = f_f * e_f;
friction_torque = cross([quay_contact_point ; 0], [friction_force; 0]);
friction_yaw_torque = friction_torque(3);
end

function vel = calc_vel(p_contact, p_vessel, nu, R_q_b)
r = p_contact - p_vessel;  % Vector from vessel center to contact point
r_ang = atan2(r(2), r(1));  % Vector angle
r_normal = [cos(r_ang + pi / 2), sin(r_ang + pi / 2)]';
v_rot = nu(6) * norm(r) * r_normal;
vel = R_q_b * nu(1:2) + v_rot;
end

function [f, mean_point, mean_vel]= normal_force(p0, v0, p1, v1, k_dock, d_dock)
% Calculates normal force and point of contact for a linear contact edge
% :param p0: Start vertex of contact edge
% :param v0: Start vertex speed
% :param p1: End vertex of contact edge
% :param v1: End vertex speed
% :return: (f, p_contact, v_contact)

% Riemann integral
mean_point = p0 + (p1 - p0) / 2;
mean_vel = (v0 + v1) / 2;
df = - k_dock * mean_point(2) - d_dock * mean_vel(2);
dx = abs(p1(1) - p0(1));
f = df*dx;
end
