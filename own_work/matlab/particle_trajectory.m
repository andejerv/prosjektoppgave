%% System

A = diag([1 1 1 1 1 1 0 0 0]);
B = [zeros(6, 3); eye(3)];


%% Weights

Q_h = eye(9); % Weight matrix for quadratic state error
R_h = eye(3); % Weight matrix for quadratic control inputs

Q_f = eye(9); % Weight matrix for linear state error
R_f = eye(3); % Weight matrix for linear control inputs


%% Optimization setup
H = []; % Quadratic cost
f = []; %Linear cost

A = []; % Linear inequality A*x <= b
b = [];

Aeq = []; % Linear equality Aeq*x = b
beq = [];

lb = []; % x(i) >= lb(i), size(z)
ub = []; % x(i) <= ub(i), size(z)

z0 = zeros(1); % Size of opt_var dim

options = optimoptions("quadprog", ...
    "Algorithm","interior-point-convex", ...
    "MaxIterations",500, ...
    "OptimalityTolerance",1e-6, ...
    "StepTolerance",1e-6);


%% Optimization

[z,fval,exitflag,output] = quadprog(H,f,A,b,Aeq,beq,lb,ub,z0,options);


%% Parse output