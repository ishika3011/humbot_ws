import numpy as np

def set_acados_cost(ocp, Q, R, QN):

    nx = 3
    nu = 2
    ny = nx + nu

    ocp.cost.cost_type = "LINEAR_LS"
    #since using LINEAR_LS, it minimises at each stage k:
    #   (Vx·x + Vu·u - y_ref)ᵀ  W  (Vx·x + Vu·u - y_ref)
    #since y/y_ref is of shape (nx+nu , 1) this we need Vx and Vu of (5x3) and (5x2)
    ocp.cost.Vx = np.vstack([np.eye(nx),
                             np.zeros((nu,nx))
    ])
    
    ocp.cost.Vu = np.vstack([np.zeros((nx,nu)),
                             np.eye(nu)
    ])

    # W weight matrix = block diag (Q, R) --> dim = 5x5
    ocp.cost.W = np.block([
        [Q, np.zeros((nx,nu))],
        [np.zeros((nu,nx)), R]
    ])

    ocp.cost.yref = np.zeros(ny)  #this is updated at runtime

    #---------------Terminal cost-------------------

    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.W_e = QN
    ocp.cost.yref_e = np.zeros(nx)

    return ocp




