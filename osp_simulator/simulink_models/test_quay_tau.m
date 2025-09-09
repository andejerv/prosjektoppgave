%% Prepare tests, single quay
clearvars
nearZero = @(x,tol) abs(x) < tol;

% Quay residing in the first and fourth quadrant (positive North
% coordinates)
quays(1).line = [
    [0, -10]
    [0, 10]
]';
quays(1).depth = 50;

% Test parameters - very low spring and dampning constants, do not use for
% quay actual modeling!
par.mu = 1;
par.k = 10;
par.d = 5;
par.v_static = 0.1;
par.stiction_damping = true;
L = 10;
B = 5;
par.vessel_polygon = [
    [L/2, B/2]
    [L/2, -B/2]
    [-L/2, -B/2]
    [-L/2, B/2]
]';

%% Slightly outside of dock
tau = [1,2,3,4,5,6]';
eta = [-L/2 - 0.01,0,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% Touching dock
tau = [1,2,3,4,5,6]';
eta = [-L/2,0,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% To the side of the dock
tau = [1,2,3,4,5,6]';
eta = [0, 10 + B/2,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))


%% Behind dock
tau = [1,2,3,4,5,6]';
eta = [50 + L/2 + 0.0001,0,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% Interaction - one meter inside dock, zero surge speed
tau = [1,2,3,4,5,6]';
eta = [-L/2+1,0,0,0,0,0]';

% Zero speed, counteract sway force
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B;
Y = -2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force
nu = [0,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B;
Y = - abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

tau = [1,1000,3,4,5,6]';
% Zero speed, counteract sway force - high sway force
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B;
Y = - abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force - high sway force
nu = [0,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B;
Y = - abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - one meter inside dock, positive surge speed
tau = [1,2,3,4,5,6]';
eta = [-L/2+1,0,0,0,0,0]';

% Zero sway speed, counteract sway force
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
Y = -2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force
nu = [1,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
Y = - abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - one meter inside dock, negative surge speed
tau = [1,2,3,4,5,6]';
eta = [-L/2+1,0,0,0,0,0]';

% Zero sway speed, counteract sway force
nu = [-1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B + 1*par.d*B;
Y = -2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force
nu = [-1,-1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B + 1*par.d*B;
Y = abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - front port section of boat inside dock, zero surge speed
tau = [1,2,3,4,5,6]';
eta = [0,10,0,0,0,0]';

% Zero speed, counteract sway force
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*L/2*B/2;
Y = -2;
N = Y*L/2 - abs(X)*B/4; % Resulting yaw moment from sway and surge force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force
nu = [0,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*L/2*B/2;
Y = - abs(X) * par.mu;
N = Y*L/2 - abs(X)*B/4; % Resulting yaw moment from sway and surge force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - one meter inside dock, sway speed under v_static threshold
eta = [-L/2+1,0,0,0,0,0]';

sway_speed = par.v_static/2;

% Zero sway speed, counteract sway force
tau = [1,2,3,4,5,6]';
nu = [1,sway_speed,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
f_f_max = par.mu*abs(X);
Y = -2 - f_f_max/2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Zero sway speed, counteract sway force - high sway force
tau = [1,2000,3,4,5,6]';
nu = [1,sway_speed,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
f_f_max = par.mu*abs(X);
Y = -f_f_max;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Test with stiction_damping false
par_ = par; % Create separate parameter struct here
par_.stiction_damping = false;

tau = [1,2,3,4,5,6]';
nu = [1,sway_speed,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par_);
X = -10*B - 1*par_.d*B;
Y = -2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Test non-zero heading
tau = [1,2,3,4,5,6]';
fore_stb_corner = [1;0];
R = Rzyx(0,0,pi/4);
R = R(1:2,1:2);
r_b = -[L/2; -B/2];
p_n = fore_stb_corner + R*r_b;
eta = [p_n(1), p_n(2), 0,0,0,pi/4]';

% Zero speed along quay line
nu = [1,-1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
f_n = (0.5*10 + norm(nu)*5)*2; % Mean contact point is 0.5 inside dock, 2 meter of dock covered
f_f = dot(tau(1:2), [1,1]/sqrt(2));
X = -f_n/sqrt(2) - f_f/sqrt(2);
Y = f_n/sqrt(2) - f_f/sqrt(2);
d_from_side = 0.5/sqrt(2); % Contact point is 0.5m from the corner of the vessel
N = Y*(L/2-d_from_side) - X*(-B/2 + d_from_side); % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero speed along quay line
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
f_n = (0.5*10 + 1/sqrt(2)*5)*2; % Mean contact point is 0.5 inside dock, 2 meter of dock covered
f_f = f_n*par.mu;
X = -f_n/sqrt(2) - f_f/sqrt(2);
Y = f_n/sqrt(2) - f_f/sqrt(2);
d_from_side = 0.5/sqrt(2); % Contact point is 0.5m from the corner of the vessel
N = Y*(L/2-d_from_side) - X*(-B/2 + d_from_side); % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Prepare tests, two quays
clearvars
nearZero = @(x,tol) abs(x) < tol;

% Quay 1 residing in the fourth quadrant (positive North, negative East
% coordinates)
quays(1).line = [
    [0, -10]
    [0, 0]
]';
quays(1).depth = 50;
% Quay 2 residing in the second quadrant (negative North, positive East
% coordinates)
quays(2).line = [
    [0, 0]
    [-10, 0]
]';
quays(2).depth = 50;

% Test parameters - very low spring and dampning constants, do not use for
% quay actual modeling!
par.mu = 1;
par.k = 10;
par.d = 5;
par.v_static = 0.1;
L = 10;
B = 5;
par.vessel_polygon = [
    [L/2, B/2]
    [L/2, -B/2]
    [-L/2, -B/2]
    [-L/2, B/2]
]';
par.stiction_damping = true;

%% Slightly outside of dock
tau = [1,2,3,4,5,6]';
eta = [-L/2 - 0.01, - B/2 - 0.01,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% Touching dock
tau = [1,2,3,4,5,6]';
eta = [-L/2,-B/2,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% To the side of the dock 1
tau = [1,2,3,4,5,6]';
eta = [0, -10 - B/2,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% To the side of the dock 2
tau = [1,2,3,4,5,6]';
eta = [-10-L/2,0,0,0,0,0]';
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);

assert(all(q==0))

%% Interaction - one meter inside dock 1, positive surge speed
tau = [1,2,3,4,5,6]';
eta = [-L/2+1,-B/2,0,0,0,0]';

% Zero sway speed, counteract sway force
nu = [1,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
Y = -2;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero sway speed, apply friction force
nu = [1,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -10*B - 1*par.d*B;
Y = - abs(X) * par.mu;
N = Y*L/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - one meter inside dock 2, positive sway speed
tau = [1,2,3,4,5,6]';
eta = [-L/2,-B/2+1,0,0,0,0]';

% Zero surge speed, counteract surge force
nu = [0,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
X = -1;
Y = -10*L - 1*par.d*L;
N = -X*B/2; % Resulting yaw moment from surge force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero surgespeed, apply friction force
nu = [1,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
Y = -10*L - 1*par.d*L;
X = -abs(Y) * par.mu;
N = -X*B/2; % Resulting yaw moment from sway force
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

%% Interaction - one meter inside both docks
eta = [-L/2+1,-B/2+1,0,0,0,0]';

% Zero speed, zero force
tau = [0,0,0,0,0,0]';
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
fn = [-10*(B-1) ; -10*(L-1)];
X = fn(1);
Y = fn(2);
N = -abs(fn(1)) * 0.5 + abs(fn(2)) * 0.5; % No friction force, but torque from normal forces
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Zero speed, non-zero force
tau = [10,10,0,0,0,0]';
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
fn = [-10*(B-1) ; -10*(L-1)];
contact_point = [
    [L/2, -0.5]
    [-0.5, B/2]
];
N_n = abs(fn(1)) * contact_point(2,1) - abs(fn(2)) * contact_point(1,2);
ang = atan2(fn(2), fn(1))+pi/2;
e_f = [cos(ang); sin(ang)]; % Friction force unit vector
f_f = -dot(tau(1:2), e_f)*e_f; % Stiction, counteract applied force
friction_contact_point = (contact_point(:,1)*abs(fn(1)) + contact_point(:,2)*abs(fn(2)))/sum(abs(fn));
N_f = f_f(2) * friction_contact_point(1) - f_f(1) * friction_contact_point(2);
X = fn(1) + f_f(1);
Y = fn(2) + f_f(2);
N = N_n + N_f;
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Zero speed, high force - we have stiction, but max at maximum friction
% force
tau = [1000,1000,0,0,0,0]';
nu = [0,0,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
fn = [-10*(B-1) ; -10*(L-1)];
contact_point = [
    [L/2, -0.5]
    [-0.5, B/2]
];
N_n = abs(fn(1)) * contact_point(2,1) - abs(fn(2)) * contact_point(1,2);
ang = atan2(fn(2), fn(1))+pi/2;
e_f = [cos(ang); sin(ang)]; % Friction force unit vector
f_f = -par.mu*norm(fn)*e_f; % Stiction, counteract applied force - maxing at friction force
friction_contact_point = (contact_point(:,1)*abs(fn(1)) + contact_point(:,2)*abs(fn(2)))/sum(abs(fn));
N_f = f_f(2) * friction_contact_point(1) - f_f(1) * friction_contact_point(2);
X = fn(1) + f_f(1);
Y = fn(2) + f_f(2);
N = N_n + N_f;
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero speed, non-zero force - we are sliding, getting max friction
% force
tau = [0,0,0,0,0,0]';
nu = [1,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
fn = [-10*(B-1)-1*par.d*(B-1); -10*(L-1)-1*par.d*(L-1)];
contact_point = [
    [L/2, -0.5]
    [-0.5, B/2]
];
N_n = abs(fn(1)) * contact_point(2,1) - abs(fn(2)) * contact_point(1,2);
ang = atan2(fn(2), fn(1))+pi/2;
e_f = [cos(ang); sin(ang)]; % Friction force unit vector
f_f = -par.mu*norm(fn)*e_f; % Friction force
friction_contact_point = (contact_point(:,1)*abs(fn(1)) + contact_point(:,2)*abs(fn(2)))/sum(abs(fn));
N_f = f_f(2) * friction_contact_point(1) - f_f(1) * friction_contact_point(2);
X = fn(1) + f_f(1);
Y = fn(2) + f_f(2);
N = N_n + N_f;
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))

% Non-zero speed, non-zero force - we are sliding, getting max friction
% force
tau = [10,20,0,0,0,30]';
nu = [1,1,0,0,0,0]';
q = quay_tau(eta,nu,tau,quays,par);
fn = [-10*(B-1)-1*par.d*(B-1); -10*(L-1)-1*par.d*(L-1)];
contact_point = [
    [L/2, -0.5]
    [-0.5, B/2]
];
N_n = abs(fn(1)) * contact_point(2,1) - abs(fn(2)) * contact_point(1,2);
ang = atan2(fn(2), fn(1))+pi/2;
e_f = [cos(ang); sin(ang)]; % Friction force unit vector
f_f = -par.mu*norm(fn)*e_f; % Friction force
friction_contact_point = (contact_point(:,1)*abs(fn(1)) + contact_point(:,2)*abs(fn(2)))/sum(abs(fn));
N_f = f_f(2) * friction_contact_point(1) - f_f(1) * friction_contact_point(2);
X = fn(1) + f_f(1);
Y = fn(2) + f_f(2);
N = N_n + N_f;
assert(all(nearZero(q - [X, Y, 0, 0, 0, N]', 1e-9)))
