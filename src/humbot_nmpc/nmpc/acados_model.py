import casadi as ca
from acados_template import AcadosModel

def build_acados_model():

    model = AcadosModel()
    model.name = "unicycle"

    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    x = ca.vertcat(px,py,theta)

    v = ca.SX.sym("v")
    omega = ca.SX.sym("omega")
    u = ca.vertcat(v, omega)

    x_dot_expr = ca.vertcat(
        v*ca.cos(theta),
        v*ca.sin(theta),
        omega
    )

    model.x = x
    model.u = u
    model.xdot = ca.SX.sym("xdot", 3)
    model.f_expl_expr = x_dot_expr

    return model