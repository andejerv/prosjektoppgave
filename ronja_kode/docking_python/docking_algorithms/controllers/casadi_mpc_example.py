import casadi as ca
import numpy as np

# Create an Opti instance
opti = ca.Opti()

# Define the time step and horizon
dt = 0.1   # Discretization time step
N = 20     # Prediction horizon

# State variables: x, y, psi, u, v, r
X = opti.variable(6, N+1)

# Control inputs: tau_u, tau_v, tau_r
U = opti.variable(3, N)

# Extract individual state variables for clarity
x, y, psi, u, v, r = X[0, :], X[1, :], X[2, :], X[3, :], X[4, :], X[5, :]

# Extract individual control inputs
tau_u, tau_v, tau_r = U[0, :], U[1, :], U[2, :]

# Rotation matrix (transform body-frame velocities to global frame)
def rotation_matrix(psi):
    return ca.vertcat(
        ca.horzcat(ca.cos(psi), -ca.sin(psi), 0),
        ca.horzcat(ca.sin(psi),  ca.cos(psi), 0),
        ca.horzcat(0, 0, 1)
    )

# Define the vehicle parameters (example values)
m = 1000  # Mass
Iz = 500  # Moment of inertia
Xu, Yv, Nr = -50, -100, -30  # Hydrodynamic damping
M_rb = ca.diag([m, m, Iz])
D = ca.diag([-Xu, -Yv, -Nr])

# Continuous-time dynamics function
def dynamics(x, u):
    eta_dot = ca.mtimes(rotation_matrix(x[2]), x[3:6])  # Position update
    nu_dot = ca.mtimes(ca.inv(M_rb), (-ca.mtimes(D, x[3:6]) + u))  # Velocity update
    return ca.vertcat(eta_dot, nu_dot)

# Discretized dynamics using Euler integration
for k in range(N):
    x_next = X[:, k] + dt * dynamics(X[:, k], U[:, k])
    opti.subject_to(X[:, k+1] == x_next)

# Define cost function (example: minimize control effort)
cost = ca.sumsqr(U)  # Quadratic cost on inputs
opti.minimize(cost)

# Constraints (example: control limits)
u_max = 500
opti.subject_to(opti.bounded(-u_max, U, u_max))

# Initial state constraint
x_init = [0, 0, 0, 0, 0, 0]  # Example initial condition
opti.subject_to(X[:, 0] == x_init)

# Solver setup
opti.solver('ipopt')

# Solve the optimization problem
sol = opti.solve()

# Extract solution
X_sol = sol.value(X)
U_sol = sol.value(U)
