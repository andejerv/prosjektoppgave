import numpy as np
from casadi import MX, DM, Function, vertcat, vcat, nlpsol
from casadi import pi, inf

# Setup

MAX_VEL = 1
MAX_ANG_VEL = 1

MAX_ACC = 1
MAX_ANG_ACCEL = 1

MAX_JERK = 1
MAX_ANG_JERK = 1



T = 20. # Time horizon
N = 200 # number of control intervals

# Declare model variables
x1 = MX.sym('X')
x2 = MX.sym('Y')
x3 = MX.sym('psi')
x4 = MX.sym('V_x')
x5 = MX.sym('V_y')
x6 = MX.sym('omega')
x7 = MX.sym('A_x')
x8 = MX.sym('A_y')
x9 = MX.sym('omegadot')

x = vertcat(x1, x2, x3, x4, x5, x6, x7, x8, x9)

u1 = MX.sym('J_x')
u2 = MX.sym('J_y')
u3 = MX.sym('J_angular')

u = vertcat(u1, u2, u3)

# Model equations
xdot = vertcat(x4, x5, x6, x7, x8, x9, u1, u2, u3)

# Objective term
L = 2*x1**2 + 2*x2**2 + 2*x3**2 + u1**2 + u2**2 + u3**2

# Formulate discrete time dynamics
# Fixed step Runge-Kutta 4 integrator
M = 1 # Time steps per control action
DT = T/N/M
f = Function('f', [x, u], [xdot, L])
X0 = MX.sym('X0', 9)
U = MX.sym('U', 3)
X = X0
Q = 0
for j in range(M):
    k1, k1_q = f(X, U)
    k2, k2_q = f(X + DT/2 * k1, U)
    k3, k3_q = f(X + DT/2 * k2, U)
    k4, k4_q = f(X + DT * k3, U)
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

# Start with an empty NLP
w=[]
w0 = []
lbw = []
ubw = []
J = 0
g=[]
lbg = []
ubg = []

# Formulate the NLP
X_init = [1, 2, pi, 0, 0, 0, 0, 0, 0] # POS VEL ACC

Xk = MX(X_init)
for k in range(N):
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k), 3)
    w += [Uk]
    lbw += [-MAX_JERK, -MAX_JERK, -MAX_ANG_JERK] # LB jerk
    ubw += [MAX_JERK, MAX_JERK, MAX_ANG_JERK] # UB jerk
    w0 += [0,0,0]

    # Integrate till the end of the interval
    Fk = F(x0=Xk, p=Uk)
    Xk = Fk['xf']
    J=J+Fk['qf']

    # Add inequality constraint
    g += [Xk]
    lbg += [0, 0, -inf, -MAX_VEL, -MAX_VEL, -MAX_ANG_VEL, -MAX_ACC, -MAX_ACC, -MAX_ANG_ACCEL] # POS VEL ACC
    ubg += [inf, inf, inf, MAX_VEL, MAX_VEL, MAX_ANG_VEL, MAX_ACC, MAX_ACC, MAX_ANG_ACCEL]


# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob)

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
u_opt = sol['x']


# Parse solution

x_opt = [X_init]
for k in range(N):
    Fk = F(x0=x_opt[-1], p=u_opt[k*3:(k+1)*3])
    x_opt += [Fk['xf'].full()]
x1_opt = vcat([r[0] for r in x_opt])
x2_opt = vcat([r[1] for r in x_opt])
x3_opt = vcat([r[2] for r in x_opt])
x4_opt = vcat([r[3] for r in x_opt])
x5_opt = vcat([r[4] for r in x_opt])
x6_opt = vcat([r[5] for r in x_opt])
x7_opt = vcat([r[6] for r in x_opt])
x8_opt = vcat([r[7] for r in x_opt])
x9_opt = vcat([r[8] for r in x_opt])

u1_opt = u_opt[0:N*3:3]
u2_opt = u_opt[1:N*3:3]
u3_opt = u_opt[2:N*3:3]


# Plot solution

tgrid = [T/N*k for k in range(N+1)]
import matplotlib.pyplot as plt

arrow_len = 0.05
x = np.array(x1_opt.full()).ravel()
y = np.array(x2_opt.full()).ravel()
theta = np.array(x3_opt.full()).ravel()

U = (arrow_len * np.cos(theta)).ravel()
V = (arrow_len * np.sin(theta)).ravel()

plt.figure(1); plt.clf()
plt.quiver(x, y, U, V, angles='xy', scale_units='xy', scale=1, width=0.005, headwidth=3)
plt.title('Map')
plt.xlabel('x'); plt.ylabel('y')          # <- these are spatial axes, not time
plt.legend(['Pose arrows'])
plt.axis('equal'); plt.grid(True)
plt.show()

plt.figure(2)
plt.clf()
plt.plot(tgrid, x1_opt)
plt.plot(tgrid, x2_opt)
plt.plot(tgrid, x3_opt)
plt.title('Map')
plt.xlabel('t')
plt.legend(['x', 'y', r'$\theta$'])
plt.grid()
plt.show()

plt.figure(3)
plt.clf()
plt.plot(tgrid, x4_opt)
plt.plot(tgrid, x5_opt)
plt.plot(tgrid, x6_opt)
plt.title('Velocity')
plt.xlabel('t')
plt.legend([r'$\dot{x}$',r'$\dot{y}$',r'$\omega$'])
plt.grid()
plt.show()

plt.figure(4)
plt.clf()
plt.plot(tgrid, x7_opt)
plt.plot(tgrid, x8_opt)
plt.plot(tgrid, x9_opt)
plt.title('Acceleration')
plt.xlabel('t')
plt.legend([r'$\ddot{x}$',r'$\ddot{y}$',r'$\dot{\omega}$'])
plt.grid()
plt.show()

plt.figure(5)
plt.clf()
plt.plot(tgrid[1:], u1_opt)
plt.plot(tgrid[1:], u2_opt)
plt.plot(tgrid[1:], u3_opt)
plt.title('Jerk')
plt.xlabel('t')
plt.legend([r'$J_x$',r'$J_y$',r'$J_{\omega}$'])
plt.grid()
plt.show()
