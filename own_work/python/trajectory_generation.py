from numpy import ndarray
from casadi import MX, Function, vertcat

T = 10. # Time horizon
N = 20 # number of control intervals

# Declare model variables
# x1 = MX.sym('x1')
# x2 = MX.sym('x2')
# x = vertcat(x1, x2)
# u = MX.sym('u')

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
# xdot = vertcat((1-x2**2)*x1 - x2 + u, x1)
xdot = vertcat(x4, x5, x6, x7, x8, x9, u1, u2, u3)

# Objective term
L = x1**2 + x2**2 + u**2

# Formulate discrete time dynamics
if False:
   # CVODES from the SUNDIALS suite
   dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
   F = integrator('F', 'cvodes', dae, 0, T/N)
else:
   # Fixed step Runge-Kutta 4 integrator
   M = 4 # RK4 steps per interval
   DT = T/N/M
   f = Function('f', [x, u], [xdot, L])
   X0 = MX.sym('X0', 2)
   U = MX.sym('U')
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

# Evaluate at a test point
Fk = F(x0=[0.2,0.3],p=0.4)
print(Fk['xf'])
print(Fk['qf'])

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
Xk = MX([0, 1])
for k in range(N):
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k))
    w += [Uk]
    lbw += [-1]
    ubw += [1]
    w0 += [0]

    # Integrate till the end of the interval
    Fk = F(x0=Xk, p=Uk)
    Xk = Fk['xf']
    J=J+Fk['qf']

    # Add inequality constraint
    g += [Xk[0]]
    lbg += [-.25]
    ubg += [inf]

# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob)

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x']

# Plot the solution
u_opt = w_opt
x_opt = [[0, 1]]
for k in range(N):
    Fk = F(x0=x_opt[-1], p=u_opt[k])
    x_opt += [Fk['xf'].full()]
x1_opt = vcat([r[0] for r in x_opt])
x2_opt = vcat([r[1] for r in x_opt])

tgrid = [T/N*k for k in range(N+1)]
import matplotlib.pyplot as plt
plt.figure(1)
plt.clf()
plt.plot(tgrid, x1_opt, '--')
plt.plot(tgrid, x2_opt, '-')
plt.step(tgrid, vertcat(DM.nan(1), u_opt), '-.')
plt.xlabel('t')
plt.legend(['x1','x2','u'])
plt.grid()
plt.show()
