from casadi import MX, sqrt, inf

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