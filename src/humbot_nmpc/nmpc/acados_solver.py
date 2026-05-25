"""
acados NMPC solver: Human-Aware NMPC
 
Adds elliptical human safety constraints as SOFT nonlinear constraints
with slack variables.
 
CHANGES FROM WEEK 3:
1. Added human poses as parameters (3 humans * 3 values = 9 params)
2. Added nonlinear constraints (3 ellipse constraints per stage)
3. Added slack variables with L1+L2 penalties
4. Default human positions set "far away" so unused slots don't constrain

"""

import numpy as np
import time
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
 
from acados_model import build_acados_model
from acados_cost  import set_acados_cost
from acados_constraints import (
    create_multi_human_constraints,
    A_ELLIPSE,
    B_ELLIPSE,
    K_HUMANS,
)


N = 30
Tf = 3.0 #complete horizon time (in secs) --> dt = 0.1s/step

Q = np.diag([10.0, 10.0, 1.0])
R = np.diag([0.5, 0.2])
QN = np.diag([100.0, 100.0, 10.0])

V_MIN = -0.05 # m/s
V_MAX = 0.22
OMEGA_MIN = -2.84 # rad/sec
OMEGA_MAX = 2.84

# ── Slack Penalty Weights ─────────────────────────────────────────────────────
# Higher weights → constraint behaves more like hard constraint
# Lower weights → solver more willing to violate (use for diagnostic tuning)
SLACK_L1_PENALTY = 100.0   # zl, zu: linear penalty on slack
SLACK_L2_PENALTY = 1000.0  # Zl, Zu: quadratic penalty on slack
 
# ── Default Human Positions (when no humans detected) ─────────────────────────
FAR_AWAY_X = 100.0  # 1km in +x direction
FAR_AWAY_Y = 100.0

# Build Model with Parameters

def build_acados_model_with_humans():
    model = build_acados_model()

    p_humans = ca.SX.sym("p_humans", 3 * K_HUMANS)
    model.p = p_humans
    return model


def build_acados_solver():

    ocp = AcadosOcp()

    ocp.model = build_acados_model_with_humans()

    #----horizon--------------------------------------
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    #--cost--------------------------------------
    ocp = set_acados_cost(ocp, Q, R, QN)

    #--control bunds---------------------------------------
    ocp.constraints.lbu = np.array([V_MIN, OMEGA_MIN])
    ocp.constraints.ubu = np.array([V_MAX, OMEGA_MAX])
    ocp.constraints.idxbu = np.array([0, 1]) # applying to both controls

    ocp.constraints.x0 = np.zeros(3) # initial state

    #----------Humans constraints--------------------------------------
    #h(x,p) >= 0 
    
    h_expr = create_multi_human_constraints(
        ocp.model.x,
        ocp.model.p,
        n_humans=K_HUMANS
    )
    ocp.model.con_h_expr = h_expr  # constraint expression at intermediate stages
    
    # Bounds: h ≥ 0 means lh = 0, uh = +inf
    ocp.constraints.lh = np.zeros(K_HUMANS)
    ocp.constraints.uh = np.full(K_HUMANS, 1e6)  # effectively +infinity
    
    # -- Slack variables for soft constraint ------------------------------------
    # Make all K nonlinear constraints soft via slack variables
    ocp.constraints.idxsh = np.arange(K_HUMANS)  # all h constraints get slacks
    
    # Slack penalty weights (lower bound and upper bound separately)
    # Since we only have lh (lower), only zl and Zl are active
    ocp.cost.zl = SLACK_L1_PENALTY * np.ones(K_HUMANS)
    ocp.cost.zu = SLACK_L1_PENALTY * np.ones(K_HUMANS)
    ocp.cost.Zl = SLACK_L2_PENALTY * np.ones(K_HUMANS)
    ocp.cost.Zu = SLACK_L2_PENALTY * np.ones(K_HUMANS)
    
    # -- Default parameter values (humans far away) -------------------------------
    default_p = np.array([
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,  # human 1
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,  # human 2
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,  # human 3
    ])
    ocp.parameter_values = default_p

    #---solver options -----------------------------------------------------------
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  #one SQP iteration per timestamp
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # O(N)
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 0 # silent

    solver = AcadosOcpSolver(ocp, json_file='unicycle_ocp.json')

    return solver

def solve_step(solver, x0_val, x_ref, human_poses=None):
    """
    Parameters
    ----------
    solver  : AcadosOcpSolver   (built once by build_acados_solver)
    x0_val  : np.array [3]      current robot state [px, py, theta] from /odom
    x_ref   : np.array [3]      subgoal [px, py, theta]  (from Nav2)
    human_poses: np.array [9] - stacked [x1,y1,θ1, x2,y2,θ2, x3,y3,θ3]
                     or None to use default (humans far away)
 
    Returns
    -------
    u0      : np.array [2]      [v, omega] — publish on /cmd_vel
    status  : int               0=success, 2=max_iter (both acceptable for RTI)
    slack_active: bool          True if any slack was activated (constraint violated)
    """

    solver.set(0, "lbx", x0_val)
    solver.set(0, "ubx", x0_val)
    #x0_val for creating EQUALITY CONSTRAINT

    # ── Set human parameters ──────────────────────────────────────────────────
    if human_poses is None:
        # No humans detected — use defaults (far away)
        human_poses = np.array([
            FAR_AWAY_X, FAR_AWAY_Y, 0.0,
            FAR_AWAY_X, FAR_AWAY_Y, 0.0,
            FAR_AWAY_X, FAR_AWAY_Y, 0.0,
        ])
    
    # Set parameters at every shooting node
    for k in range(N + 1):
        solver.set(k, "p", human_poses)

    #-------set reference------------------------------
    y_ref_stage = np.append(x_ref, [0.0, 0.0])
    for k in range(N):
        solver.set(k, "yref", y_ref_stage)
    
    solver.set(N, "yref", x_ref)

    status = solver.solve()

    if status not in [0, 2]:
        print(f"[NMPC] solver warning:status={status}")

    u0 = solver.get(0, "u")

    # ── Check if any slack was activated (diagnostic) ─────────────────────────
    slack_active = False
    try:
        # Try to read slack values from any intermediate stage
        for k in range(1, N):
            sl = solver.get(k, "sl")  # lower slack
            if np.any(sl > 1e-4):
                slack_active = True
                break
    except Exception:
        pass  # not all versions support reading slacks this way

    return u0, status, slack_active

#--------Standalone tests -----------------------

if __name__ == "__main__":
 
    print("Building solver (compiling C code — ~10-30s the first time)...")
    solver = build_acados_solver()
    print("Solver ready.\n")
 
    # # ── Single solve test ──────────────────────────────────────────
    # x0_val = np.array([0.0, 0.0, 0.0])          # start: origin, facing right
    # x_ref  = np.array([1.0, 0.5, 0.785])         # goal: 1m right, 0.5m up, 45°
 
    # t0     = time.perf_counter()
    # u0, status = solve_step(solver, x0_val, x_ref)
    # t_ms   = (time.perf_counter() - t0) * 1000
 
    # print(f"Status : {status}  (0=success)")
    # print(f"u0     : v={u0[0]:.4f} m/s   omega={u0[1]:.4f} rad/s")
    # print(f"Solve  : {t_ms:.2f} ms  (target: <5ms)\n")
 
    # # ── Bounds check ───────────────────────────────────────────────
    # assert V_MIN     <= u0[0] <= V_MAX,     f"v={u0[0]} out of bounds!"
    # assert OMEGA_MIN <= u0[1] <= OMEGA_MAX, f"omega={u0[1]} out of bounds!"
    # print("Bounds check PASSED\n")

    # # ── Predicted trajectory (all N+1 states) ─────────────────────
    # print("Predicted states over horizon:")
    # print(f"  {'k':>3}   {'px':>8}   {'py':>8}   {'theta':>8}")
    # print(f"  {'-'*3}   {'-'*8}   {'-'*8}   {'-'*8}")
    # for k in range(N + 1):
    #     xk = solver.get(k, "x")
    #     print(f"  {k:>3}   {xk[0]:>8.4f}   {xk[1]:>8.4f}   {xk[2]:>8.4f}")



# ==================NMPC with Human-Aware Soft Constraints=================================

# ── Test 1: No humans — should match Week 3 behavior ─────────────────────
    print("Test 1: No humans present (default: far away)")
    print("-" * 70)
    
    x0_val = np.array([0.0, 0.0, 0.0])
    x_ref = np.array([1.0, 0.5, 0.785])
    
    t0 = time.perf_counter()
    u0, status, slack_active = solve_step(solver, x0_val, x_ref)
    t_ms = (time.perf_counter() - t0) * 1000
    
    print(f"  Status     : {status}")
    print(f"  v          : {u0[0]:.4f} m/s")
    print(f"  omega      : {u0[1]:.4f} rad/s")
    print(f"  Slack      : {'ACTIVE' if slack_active else 'inactive'}")
    print(f"  Solve time : {t_ms:.2f} ms")
    print()
    
    # ── Test 2: Human directly in robot's path ────────────────────────────────
    print("Test 2: Human at (0.5, 0) blocking direct path to (1.0, 0)")
    print("-" * 70)
    
    x0_val = np.array([0.0, 0.0, 0.0])
    x_ref = np.array([1.0, 0.0, 0.0])  # straight ahead
    
    # Human in the middle, facing +x (perpendicular to robot path)
    human_poses = np.array([
        0.5, 0.0, np.pi/2,             # human 1: blocking path
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,   # human 2: far away
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,   # human 3: far away
    ])
    
    t0 = time.perf_counter()
    u0, status, slack_active = solve_step(solver, x0_val, x_ref, human_poses)
    t_ms = (time.perf_counter() - t0) * 1000
    
    print(f"  Status     : {status}")
    print(f"  v          : {u0[0]:.4f} m/s")
    print(f"  omega      : {u0[1]:.4f} rad/s (should be non-zero to avoid)")
    print(f"  Slack      : {'ACTIVE (violation!)' if slack_active else 'inactive (safe path found)'}")
    print(f"  Solve time : {t_ms:.2f} ms")
    
    # Inspect predicted trajectory
    print(f"\n  Predicted trajectory (first 5 steps):")
    print(f"  {'k':>3}   {'px':>7}   {'py':>7}   {'theta':>7}")
    for k in range(min(6, N + 1)):
        xk = solver.get(k, "x")
        print(f"  {k:>3}   {xk[0]:>7.3f}   {xk[1]:>7.3f}   {xk[2]:>7.3f}")
    print()
    
    # ── Test 3: Three humans — multiple constraints ──────────────────────────
    print("Test 3: Three humans in different positions")
    print("-" * 70)
    
    x0_val = np.array([0.0, 0.0, 0.0])
    x_ref = np.array([2.0, 0.0, 0.0])
    
    human_poses = np.array([
        0.7, 0.4, np.pi,        # human 1: ahead and to left, facing -x
        1.3, -0.3, np.pi/2,     # human 2: ahead and to right, facing +y
        FAR_AWAY_X, FAR_AWAY_Y, 0.0,  # human 3: far away
    ])
    
    u0, status, slack_active = solve_step(solver, x0_val, x_ref, human_poses)
    
    print(f"  Status : {status}")
    print(f"  v      : {u0[0]:.4f} m/s")
    print(f"  omega  : {u0[1]:.4f} rad/s")
    print(f"  Slack  : {'ACTIVE' if slack_active else 'inactive'}")
    print()
    
    # ── Test 4: Timing benchmark with humans ──────────────────────────────────
    print("Test 4: Timing benchmark (100 solves with humans)")
    print("-" * 70)
    
    times = []
    np.random.seed(42)
    for _ in range(100):
        x0 = np.random.uniform(-0.5, 0.5, 3)
        x0[2] = np.random.uniform(-np.pi, np.pi)
        x_ref = np.random.uniform(-2, 2, 3)
        x_ref[2] = np.random.uniform(-np.pi, np.pi)
        
        # Random human positions
        humans = np.zeros(9)
        for i in range(K_HUMANS):
            humans[3*i] = np.random.uniform(-1, 2)
            humans[3*i+1] = np.random.uniform(-1, 1)
            humans[3*i+2] = np.random.uniform(-np.pi, np.pi)
        
        t0 = time.perf_counter()
        solve_step(solver, x0, x_ref, humans)
        times.append((time.perf_counter() - t0) * 1000)
    
    print(f"  Mean : {np.mean(times):.2f} ms")
    print(f"  Std  : {np.std(times):.2f} ms")
    print(f"  P95  : {np.percentile(times, 95):.2f} ms")
    print(f"  P99  : {np.percentile(times, 99):.2f} ms")
    print(f"  Max  : {np.max(times):.2f} ms")
    
    p95 = np.percentile(times, 95)
    print(f"\n  Target <5ms P95: {'PASS ✓' if p95 < 5.0 else 'FAIL ✗'}")
    
    print()
    print("=" * 70)
