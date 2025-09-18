from casadi import MX, sqrt, sin, cos, exp, log, dot, vertcat, inf

def smooth_abs(z: MX, eps=1e-6) -> MX:
    return sqrt(z*z + eps*eps)

def quadratic_sqrt(z1: MX, z2: MX, eps=1e-6):
    return (1/2)*(z1+z2) + (1/2)*sqrt((z1-z2)**2+eps**2)


def c_avoid_rectangle(Xk: MX, rect_cx, rect_cy, half_length_x, half_length_y):
    qx = Xk[0] - rect_cx
    qy = Xk[1] - rect_cy
    a_qx = smooth_abs(qx)/half_length_x
    a_qy = smooth_abs(qy)/half_length_y
    c = [quadratic_sqrt(a_qx,a_qy)-1]
    ub = [inf]
    lb = [0]
    return c, lb, ub


def c_SAT(Xk: MX, l1, l2, rect_cx, rect_cy, half_length_x, half_length_y):
    # Unit axes
    # Assuming outrigger and quay is axis aligned with grid
    center_boat = vertcat(Xk[0],Xk[1])
    center_obs = vertcat(rect_cx,rect_cy)
    e_x = vertcat(1,0)
    e_y = vertcat(0,1)
    # u_x = MX([[cos(Xk[2])], [sin(Xk[2])]])
    # u_y = MX([[-sin(Xk[2])], [cos(Xk[2])]])
    # Along edge directions
    v1 = vertcat(sin(Xk[2]), -cos(Xk[2])) # Heading direction
    v2 = vertcat(cos(Xk[2]), sin(Xk[2])) # Lateral direction

    normals: list[MX] = (e_x, e_y, v1, v2)
    g = []

    # Half-projection i.e. length from center to extremety along u
    for u in normals:
        h_u_boat = l1 * smooth_abs(dot(u,v1)) + l2*smooth_abs(dot(u,v2))

        # I_u_min_boat = dot(u,center_boat) - h_u_boat
        # I_u_max_boat = dot(u,center_boat) + h_u_boat

        h_u_obs = half_length_x * smooth_abs(dot(u,e_x)) + half_length_y*smooth_abs(dot(u,e_y))

        # I_u_min_obs = dot(u,center_obs) - h_u_obs
        # I_u_max_obs = dot(u,center_obs) + h_u_obs

        d_u = dot(u,(center_boat-center_obs))

        g_u = smooth_abs(d_u)-(h_u_boat + h_u_obs)

        g.append(g_u)
    
    c = [smax_lse(g)]
    lb = [0]
    ub = [inf]
    return c, lb, ub


def smax_lse(z_list: list, tau: float = 0.05, calibrate: bool = True, eps: float = 1e-6):
    if not z_list:
        raise ValueError("smax_lse: z_list must be non-empty")

    # Smooth, differentiable upper-bound shift (avoid using a hard max)
    z0 = z_list[0]
    for zi in z_list[1:]:
        z0 = quadratic_sqrt(z0, zi, eps)  # ~max(z0, zi) but C^1

    # Log-sum-exp with stable shift
    acc = MX(0)
    for zi in z_list:
        acc += exp((zi - z0) / tau)

    lse = z0 + tau * log(acc)

    # Optional calibration so that smax_lse - tau*log(m) <= true max
    if calibrate:
        m = len(z_list)
        lse = lse - tau * log(m)

    return lse
