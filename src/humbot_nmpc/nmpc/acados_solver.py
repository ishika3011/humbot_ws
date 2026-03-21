import numpy as np
import time
from acados_template import AcadosOcp, AcadosOcpSolver
 
from acados_model import build_acados_model
from acados_cost  import set_acados_cost

N = 30
Tf = 3.0 #complete horizon time (in secs) --> dt = 0.1s/step

Q = np.diag([10.0, 10.0, 1.0])
R = np.diag([0.1, 0.05])
QN = np.diag([100.0, 100.0, 10.0])

V_MIN = -0.05 # m/s
V_MAX = 0.22
OMEGA_MIN = -2.84 # rad/sec
OMEGA_MAX = 2.84

def build_acados_solver():

    ocp = AcadosOcp()

    ocp.model = build_acados_model()

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    ocp = set_acados_cost(ocp, Q, R, QN)

    ocp.constraints.lbu = np.array([V_MIN, OMEGA_MIN])
    ocp.constraints.ubu = np.array([V_MAX, OMEGA_MAX])
    ocp.constraints.idxbu = np.array([0, 1]) # applying to both controls

    ocp.constraints.x0 = np.zeros(3)

    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  #one SQP iteration per timestamp
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # O(N)
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 0 # silent

    solver = AcadosOcpSolver(ocp, json_file='unicycle_ocp.json')

    return solver

def solve_step(solver, x0_val, x_ref):
    """
    Parameters
    ----------
    solver  : AcadosOcpSolver   (built once by build_acados_solver)
    x0_val  : np.array [3]      current robot state [px, py, theta] from /odom
    x_ref   : np.array [3]      subgoal [px, py, theta]  (from Nav2 or RL later)
 
    Returns
    -------
    u0      : np.array [2]      [v, omega] — publish on /cmd_vel
    status  : int               0=success, 2=max_iter (both acceptable for RTI)
    """

    solver.set(0, "lbx", x0_val)
    solver.set(0, "ubx", x0_val)
    #x0_val for creating EQUALITY CONSTRAINT

    y_ref_stage = np.append(x_ref, [0.0, 0.0])
    for k in range(N):
        solver.set(k, "yref", y_ref_stage)
    
    solver.set(N, "yref", x_ref)

    status = solver.solve()

    if status not in [0, 2]:
        print(f"[NMPC] solver warning:status={status}")

    u0 = solver.get(0, "u")
    return u0, status

#--------Standalone tests -----------------------

if __name__ == "__main__":
 
    print("Building solver (compiling C code — ~10–30s the first time)...")
    solver = build_acados_solver()
    print("Solver ready.\n")
 
    # ── Single solve test ──────────────────────────────────────────
    x0_val = np.array([0.0, 0.0, 0.0])          # start: origin, facing right
    x_ref  = np.array([1.0, 0.5, 0.785])         # goal: 1m right, 0.5m up, 45°
 
    t0     = time.perf_counter()
    u0, status = solve_step(solver, x0_val, x_ref)
    t_ms   = (time.perf_counter() - t0) * 1000
 
    print(f"Status : {status}  (0=success)")
    print(f"u0     : v={u0[0]:.4f} m/s   omega={u0[1]:.4f} rad/s")
    print(f"Solve  : {t_ms:.2f} ms  (target: <5ms)\n")
 
    # ── Bounds check ───────────────────────────────────────────────
    assert V_MIN     <= u0[0] <= V_MAX,     f"v={u0[0]} out of bounds!"
    assert OMEGA_MIN <= u0[1] <= OMEGA_MAX, f"omega={u0[1]} out of bounds!"
    print("Bounds check PASSED\n")

    # ── Predicted trajectory (all N+1 states) ─────────────────────
    print("Predicted states over horizon:")
    print(f"  {'k':>3}   {'px':>8}   {'py':>8}   {'theta':>8}")
    print(f"  {'-'*3}   {'-'*8}   {'-'*8}   {'-'*8}")
    for k in range(N + 1):
        xk = solver.get(k, "x")
        print(f"  {k:>3}   {xk[0]:>8.4f}   {xk[1]:>8.4f}   {xk[2]:>8.4f}")


