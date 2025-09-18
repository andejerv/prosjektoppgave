import numpy as np
from numpy import ndarray, hstack
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from casadi import (
    MX, DM, Function, nlpsol, vertcat, inf, pi
)

from constraint_functions import c_avoid_rectangle, c_SAT

def get_config() -> dict:
    cfg = dict(
        # Initial state
        X_init = [5, 10, -pi/2, 0, 0, 0, 0, 0, 0],
        # Limits
        MAX_VEL = 2,
        MAX_ANG_VEL = 1,
        MAX_ACC = 1,
        MAX_ANG_ACCEL = 1,
        MAX_JERK = 10,
        MAX_ANG_JERK = 10,
        T_LB = 1e-3,
        T_UB = 100,
        T_w0 = 20,
        TERMINAL_EPS = 1e-2,
        # Discretization
        N = 400,
        M = 1, # Time steps per control interval
        # Obstacles
        boat = [0.9,0.9],
        rect = [[2,3,0.9,3], [0, -2, 1, 0.9]]
    )
    return cfg


def build_model(cfg: dict) -> Function:
    # Decision-time symbol
    t = MX.sym('T')

    # State symbols
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

    # Control symbols (jerks)
    u1 = MX.sym('J_x')
    u2 = MX.sym('J_y')
    u3 = MX.sym('J_angular')
    u = vertcat(u1, u2, u3)

    # Continuous-time dynamics
    xdot = vertcat(x4, x5, x6, x7, x8, x9, u1, u2, u3)

    # Cost function
    L = x4**2 + x5**2 + x6**2 + u1**2 + u2**2 + u3**2

    # Fixed-step RK4 over one control interval of duration DT = T/N/M
    N = int(cfg['N'])
    M = int(cfg['M'])
    DT = t / N / M

    f = Function('f', [x, u, t], [xdot, L])

    X0 = MX.sym('X0', 9)
    U = MX.sym('U', 3)
    X = X0
    Q = 0
    for _ in range(M):
        k1, k1_q = f(X, U, t)
        k2, k2_q = f(X + DT/2 * k1, U, t)
        k3, k3_q = f(X + DT/2 * k2, U, t)
        k4, k4_q = f(X + DT * k3, U, t)
        X = X + DT/6 * (k1 + 2*k2 + 2*k3 + k4)
        Q = Q + DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q)

    F = Function('F', [X0, U, t], [X, Q], ['x0','u','t'], ['xf','qf'])
    
    return F


def build_nlp_single_shooting(F: Function, cfg: dict) -> tuple[dict, list, list, list, list, list]:
    N = int(cfg['N'])
    eps = cfg['TERMINAL_EPS']
    rects = cfg['rect']

    # Start with an empty NLP
    w=[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []

    # Add time decision variable
    t = MX.sym('t')
    w += [t]
    lbw += [cfg['T_LB']]
    ubw += [cfg['T_UB']]
    w0 += [cfg['T_w0']]

    # Initial condition
    Xk = MX(cfg['X_init'])
    
    # Build over horizon
    for k in range(N-1):

        # Add decision variable and constraint
        Uk = MX.sym('U_'+ str(k), 3)
        w += [Uk]
        lbw += [-cfg['MAX_JERK'], -cfg['MAX_JERK'], -cfg['MAX_ANG_JERK']]
        ubw += [ cfg['MAX_JERK'], cfg['MAX_JERK'], cfg['MAX_ANG_JERK']]
        w0 += [0, 0, 0]

        Fk = F(x0=Xk, u=Uk, t=t)
        Xk = Fk['xf']
        J = J+Fk['qf']

        # Add inequality constraint
        g += [Xk]
        lbg += [
            -cfg['TERMINAL_EPS'], # x >= -eps
            -cfg['TERMINAL_EPS'], # y >= -eps
            -inf, # psi free below
            -cfg['MAX_VEL'],
            -cfg['MAX_VEL'],
            -cfg['MAX_ANG_VEL'],
            -cfg['MAX_ACC'],
            -cfg['MAX_ACC'],
            -cfg['MAX_ANG_ACCEL'],
        ]
        ubg += [
            inf, # x <= +inf
            inf, # y <= +inf
            inf, # psi <= +inf
            cfg['MAX_VEL'],
            cfg['MAX_VEL'],
            cfg['MAX_ANG_VEL'],
            cfg['MAX_ACC'],
            cfg['MAX_ACC'],
            cfg['MAX_ANG_ACCEL'],
        ]

        # Add obstacle constraints
        for r in rects:
            constraint = c_avoid_rectangle(Xk, r[0], r[1], r[2], r[3])
            g += constraint[0]
            lbg += constraint[1]
            ubg += constraint[2]
    
    # Add terminal
    Uk = MX.sym('U_'+ str(k), 3)
    w += [Uk]
    lbw += [-eps, -eps, -eps]
    ubw += [ eps, eps, eps]
    w0 += [0, 0, 0]

    Fk = F(x0=Xk, u=Uk, t=t)
    Xk = Fk['xf']
    J = J+Fk['qf']

    g += [Xk]
    lbg += [-eps]*9
    ubg += [ eps]*9

    # Add total-time cost
    J = J + t

    # Create a problem dict for casadi solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}

    return prob, w0, lbw, ubw, lbg, ubg


def build_nlp_mult_shooting(F: Function, cfg: dict) -> tuple[dict, list, list, list, list, list]:
    x_init = cfg['X_init']
    N = int(cfg['N'])
    eps = float(cfg['TERMINAL_EPS'])
    boat = cfg['boat']
    rects = cfg['rect']

    x_interpolate = np.concatenate((
        np.linspace(x_init[0], 0, N-(N//2)),
        np.zeros(N//2)
    ))
    y_interpolate = np.concatenate((
        x_init[1]*np.ones(N-N//2),
        np.linspace(x_init[1], 0, N//2)
    ))
    psi_interpolate = np.concatenate((
        np.linspace(x_init[2], 0, N//2),
        np.zeros(N//2)
    ))

    # Start with an empty NLP
    w=[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []

    # Add time decision variable
    t = MX.sym('t')
    w += [t]
    lbw += [cfg['T_LB']]
    ubw += [cfg['T_UB']]
    w0 += [cfg['T_w0']]

    # Initial condition
    Xk = MX.sym('X', 9)
    w += [Xk]
    lbw += cfg['X_init']
    ubw += cfg['X_init']
    w0 += cfg['X_init']

    # Build over horizon
    for k in range(N-1):

        # Add decision variable for control
        Uk = MX.sym('U_'+ str(k), 3)
        w += [Uk]
        lbw += [-cfg['MAX_JERK'], -cfg['MAX_JERK'], -cfg['MAX_ANG_JERK']]
        ubw += [ cfg['MAX_JERK'], cfg['MAX_JERK'], cfg['MAX_ANG_JERK']]
        w0 += [0, 0, 0]

        Fk = F(x0=Xk, u=Uk, t=t)
        Xk_end = Fk['xf']
        J = J+Fk['qf']

        # New NLP variable for start of next interval
        Xk = MX.sym('X_' + str(k+1), 9)

        w += [Xk]
        lbw += [
            -cfg['TERMINAL_EPS'], # x >= -eps
            -cfg['TERMINAL_EPS'], # y >= -eps
            -inf, # psi free below
            -cfg['MAX_VEL'],
            -cfg['MAX_VEL'],
            -cfg['MAX_ANG_VEL'],
            -cfg['MAX_ACC'],
            -cfg['MAX_ACC'],
            -cfg['MAX_ANG_ACCEL'],
        ]
        ubw += [
            inf, # x <= +inf
            inf, # y <= +inf
            inf, # psi <= +inf
            cfg['MAX_VEL'],
            cfg['MAX_VEL'],
            cfg['MAX_ANG_VEL'],
            cfg['MAX_ACC'],
            cfg['MAX_ACC'],
            cfg['MAX_ANG_ACCEL'],
        ]
        w0 += [x_interpolate[k], y_interpolate[k], psi_interpolate[k],0,0,0,0,0,0]

        # Add dynamics constraints
        g += [Xk_end-Xk]
        lbg += [0, 0, 0, 0, 0, 0, 0, 0 ,0]
        ubg += [0, 0, 0, 0, 0, 0, 0, 0 ,0]

        # Add obstacle constraints
        for r in rects:
            constraint = c_SAT(Xk, boat[0], boat[1], r[0], r[1], r[2], r[3])
            g += constraint[0]
            lbg += constraint[1]
            ubg += constraint[2]
    
    # Add terminal
    Uk = MX.sym('U_'+ str(k), 3)
    w += [Uk]
    lbw += [-cfg['MAX_JERK'], -cfg['MAX_JERK'], -cfg['MAX_ANG_JERK']]
    ubw += [ cfg['MAX_JERK'], cfg['MAX_JERK'], cfg['MAX_ANG_JERK']]
    w0 += [0, 0, 0]

    Fk = F(x0=Xk, u=Uk, t=t)
    Xk_end = Fk['xf']
    J = J+Fk['qf']

    g += [Xk_end]
    lbg += [-eps]*9
    ubg += [ eps]*9

    # Add total-time cost
    J = J + t

    # Create a problem dict for casadi solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}

    return prob, w0, lbw, ubw, lbg, ubg


def solve_nlp(prob, w0, lbw, ubw, lbg, ubg) -> dict:
    opts = {
        'ipopt': {
            'max_iter': 20000,
            #'print_level': 0,
            #'sb': 'yes'
        }
    }
    solver = nlpsol('solver', 'ipopt', prob, opts)
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    return sol


def parse_single_shooting_nlp(sol: dict, F: Function, cfg: dict) -> tuple[ndarray, ndarray, float]:
    z_opt: DM = sol['x']
    t_opt: float = float(z_opt[0])
    u_opt_dm = z_opt[1:]

    # Parse solution
    x_list = [np.array(cfg['X_init']).reshape(-1,1)]

    N = int(cfg['N'])
    for k in range(N):
        Fk = F(x0=x_list[-1], u=u_opt_dm[k*3:(k+1)*3], t=t_opt)
        x_list.append(Fk['xf'].full())

    x_opt: ndarray = hstack(x_list)
    u_opt: ndarray = u_opt_dm.full().reshape(3,-1,order='F')

    return x_opt, u_opt, t_opt


def parse_multiple_shooting_nlp(sol: dict, F: Function, cfg: dict) -> tuple[ndarray, ndarray, float]:
    z_opt: DM = sol['x']
    N = int(cfg['N'])

    # Extract time
    idx = 0
    t_opt: float = float(z_opt[idx])
    idx += 1

    # Helper to turn a DM slice into a (n,1) numpy column
    def col(dm_slice) -> np.ndarray:
        return np.array(dm_slice.full()).reshape(-1, 1)

    # X0
    Xk = z_opt[idx:idx+9]
    idx += 9
    x_cols = [col(Xk)]   # will hold X0..X_{N-1}, then we'll append X_N

    u_cols = []          # will hold U0..U_{N-1} as (3,1) columns

    # Loop over intervals 0..N-2, grabbing Uk then X_{k+1}
    for _ in range(N-1):
        Uk = z_opt[idx:idx+3]
        idx += 3
        u_cols.append(col(Uk))

        Xk = z_opt[idx:idx+9]
        idx += 9
        x_cols.append(col(Xk))

    # Final control U_{N-1}
    U_last = z_opt[idx:idx+3]
    u_cols.append(col(U_last))
    # idx += 3  # (not needed further)

    # Compute terminal state X_N by integrating one last interval
    Fk = F(x0=x_cols[-1], u=U_last, t=t_opt)
    X_terminal = np.array(Fk['xf'].full()).reshape(-1, 1)
    x_cols.append(X_terminal)

    # Stack into arrays
    x_opt: ndarray = np.hstack(x_cols)                 # shape (9, N+1)
    u_opt: ndarray = np.hstack(u_cols)                 # shape (3, N)

    return x_opt, u_opt, t_opt


def plot_solution(x_opt: ndarray, u_opt: ndarray, t, cfg):
    N = int(cfg['N'])
    tgrid = [t/N*k for k in range(N+1)]

    plt.figure(0)
    plt.clf()
    plt.plot(x_opt[0,:], x_opt[1,:])
    plt.title('Map')
    plt.xlabel('t')
    plt.legend(['x', 'y', r'$\psi$'])
    plt.grid()
    ax = plt.gca()  # get current axes
    # Plot boat
    L = 0.5    # boat length
    W = 0.5    # boat width
    plot_every = 20   # e.g., 3 or 10

    for k in range(0, len(tgrid), plot_every):
        x = x_opt[0, k]
        y = x_opt[1, k]
        theta = x_opt[2, k] - np.pi/2                  # radians assumed
        ct, st = np.cos(theta), np.sin(theta)

        # rectangle corners in boat-local frame (centered at 0,0)
        corners_local = np.array([
            [ L/2,  W/2],
            [ L/2, -W/2],
            [-L/2, -W/2],
            [-L/2,  W/2],
        ])

        # rotate + translate to world frame
        corners_world = np.column_stack((
            x + corners_local[:,0]*ct - corners_local[:,1]*st,
            y + corners_local[:,0]*st + corners_local[:,1]*ct
        ))

        poly = patches.Polygon(corners_world, closed=True,
                            facecolor='C1', edgecolor='k', alpha=0.5, zorder=3)
        ax.add_patch(poly)

        # heading/bow marker
        bow_x = x + (L/2)*ct
        bow_y = y + (L/2)*st
        ax.plot([x, bow_x], [y, bow_y], linewidth=1.2, zorder=4)
        

    # Plot obstacles
    for r in cfg['rect']:
        rect = patches.Rectangle(
            (r[0]-r[2], r[1]-r[3]),      # bottom-left corner (x, y)
            r[2]*2, r[3]*2,    # width, height
            linewidth=2,
            edgecolor='r',
            facecolor='red'
        )
        ax.add_patch(rect)
    plt.axis("equal")
    plt.show()

    plt.figure(1)
    plt.clf()
    plt.plot(tgrid, x_opt[0,:])
    plt.plot(tgrid, x_opt[1,:])
    plt.plot(tgrid, x_opt[2,:])
    plt.title('Position')
    plt.xlabel('t')
    plt.legend(['x', 'y', r'$\psi$'])
    plt.grid()
    plt.show()

    plt.figure(2)
    plt.clf()
    plt.plot(tgrid, x_opt[3,:])
    plt.plot(tgrid, x_opt[4,:])
    plt.plot(tgrid, x_opt[5,:])
    plt.title('Velocity')
    plt.xlabel('t')
    plt.legend([r'$\dot{x}$',r'$\dot{y}$',r'$\omega$'])
    plt.grid()
    plt.show()

    plt.figure(3)
    plt.clf()
    plt.plot(tgrid, x_opt[6,:])
    plt.plot(tgrid, x_opt[7,:])
    plt.plot(tgrid, x_opt[8,:])
    plt.title('Acceleration')
    plt.xlabel('t')
    plt.legend([r'$\ddot{x}$',r'$\ddot{y}$',r'$\dot{\omega}$'])
    plt.grid()
    plt.show()

    plt.figure(4)
    plt.clf()
    plt.plot(tgrid[1:], u_opt[0,:])
    plt.plot(tgrid[1:], u_opt[1,:])
    plt.plot(tgrid[1:], u_opt[2,:])
    plt.title('Jerk')
    plt.xlabel('t')
    plt.legend([r'$J_x$',r'$J_y$',r'$J_{\omega}$'])
    plt.grid()
    plt.show()


if __name__ == '__main__':
    conf = get_config()
    model_func = build_model(conf)
    prob, w0, lbw, ubw, lbg, ubg = build_nlp_mult_shooting(model_func, conf)
    sol = solve_nlp(prob,w0,lbw,ubw,lbg,ubg)
    x_opt, u_opt, t_opt = parse_multiple_shooting_nlp(sol, model_func, conf)
    plot_solution(x_opt, u_opt, t_opt, conf)