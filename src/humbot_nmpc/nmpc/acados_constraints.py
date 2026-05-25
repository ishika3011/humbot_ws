"""
Implements anisotropic elliptical safety zones around humans.
The ellipse is larger in front of the human (where they're heading)
and smaller at the sides, matching natural personal space patterns.
"""

import casadi as ca
import numpy as np


# ══════════════════════════════════════════════════════════════════════════════
# Ellipse Parameters
# ══════════════════════════════════════════════════════════════════════════════

# Semi-major axis (along human heading direction — front/back)
A_ELLIPSE = 0.25  # metres

# Semi-minor axis (perpendicular to heading — sides)
B_ELLIPSE = 0.15  # metres

# Maximum number of humans to track (k-nearest)
K_HUMANS = 3


# ══════════════════════════════════════════════════════════════════════════════
# Single Human Constraint
# ══════════════════════════════════════════════════════════════════════════════

def create_human_ellipse_constraint(
    x_robot: ca.SX,
    p_human: ca.SX,
    a: float = A_ELLIPSE, #major axis aligned to their heading dir
    b: float = B_ELLIPSE
) -> ca.SX:

    # ── Extract positions ─────────────────────────────────────────────────────
    # Robot position (only x, y needed, not theta)
    x_r = x_robot[0] # Robot state [px, py, theta]
    y_r = x_robot[1]
    
    # Human position and heading
    x_h = p_human[0] #Human pose [x_h, y_h, theta_h]
    y_h = p_human[1]
    theta_h = p_human[2]
    
    # ── Step 1: Translate to human-centered frame ─────────────────────────────
    # Vector from human to robot in world frame
    dx = x_r - x_h
    dy = y_r - y_h
    
    # ── Step 2: Rotate to human's body frame ──────────────────────────────────
    # Rotation matrix R(-theta_h) transforms world → human frame
    # x' = cos(θ)·dx + sin(θ)·dy
    # y' = -sin(θ)·dx + cos(θ)·dy
    cos_th = ca.cos(theta_h)
    sin_th = ca.sin(theta_h)
    
    x_prime = cos_th * dx + sin_th * dy
    y_prime = -sin_th * dx + cos_th * dy
    
    # ── Step 3: Ellipse constraint ────────────────────────────────────────────
    # h >= 0 means outside (safe)
    # h < 0 means inside (violation)
    
    h = (x_prime / a)**2 + (y_prime / b)**2 - 1
    
    return h


# ══════════════════════════════════════════════════════════════════════════════
# Multiple Humans Constraint
# ══════════════════════════════════════════════════════════════════════════════

def create_multi_human_constraints(
    x_robot: ca.SX,
    p_humans: ca.SX,
    n_humans: int = K_HUMANS,
    a: float = A_ELLIPSE,
    b: float = B_ELLIPSE
) -> ca.SX:
    """
    Create stacked constraints for multiple humans.
    
    Args:
        x_robot: Robot state [px, py, theta] as CasADi SX (3,)
        p_humans: Stacked human poses [x1,y1,θ1, x2,y2,θ2, ...] as CasADi SX (3*n_humans,)
        n_humans: Number of humans
        a, b: Ellipse parameters
    
    Returns:
        h_stack: Column vector of constraints [h1; h2; ...; hn] as CasADi SX (n_humans,)
                 All must be >= 0 for safety
    """
    constraints = []
    
    for i in range(n_humans):
        # Extract i-th human pose from stacked parameter vector
        idx = i * 3
        p_human_i = p_humans[idx:idx+3]
        
        # Create constraint for this human
        h_i = create_human_ellipse_constraint(x_robot, p_human_i, a, b)
        constraints.append(h_i)
    
    # Stack into column vector
    h_stack = ca.vertcat(*constraints)
    
    return h_stack


# ══════════════════════════════════════════════════════════════════════════════
# Helper: Create Parameter Vector
# ══════════════════════════════════════════════════════════════════════════════

def create_human_parameter(n_humans: int = K_HUMANS) -> ca.SX:
    """
    Create CasADi symbolic parameter for human poses.
    
    Args:
        n_humans: Number of humans to track
    
    Returns:
        p: CasADi SX parameter vector of shape (3*n_humans,)
           Layout: [x1, y1, θ1, x2, y2, θ2, ..., xn, yn, θn]
    """
    return ca.SX.sym("p_humans", 3 * n_humans)


# ══════════════════════════════════════════════════════════════════════════════
# Standalone Test
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("Testing Elliptical Human Constraint")
    print("=" * 60)
    print()
    
    # ── Test 1: Single human, robot at various positions ─────────────────────
    print("Test 1: Single human at origin, heading along +x")
    print("-" * 60)
    
    # Create symbolic variables
    x_robot = ca.SX.sym("x_robot", 3)
    p_human = ca.SX.sym("p_human", 3)
    
    # Create constraint expression
    h_expr = create_human_ellipse_constraint(x_robot, p_human)
    
    # Create evaluator function
    h_func = ca.Function("h", [x_robot, p_human], [h_expr])
    
    # Human at origin, facing +x direction
    human_pose = np.array([0.0, 0.0, 0.0])
    
    # Test robot at various positions
    test_positions = [
        ([0.6, 0.0, 0.0], "In front (0.6m)", "Should be >= 0 (outside)"),
        ([0.4, 0.0, 0.0], "In front (0.4m)", "Should be < 0 (inside)"),
        ([0.0, 0.4, 0.0], "To the side (0.4m)", "Should be >= 0 (outside)"),
        ([0.0, 0.2, 0.0], "To the side (0.2m)", "Should be < 0 (inside)"),
        ([0.5, 0.0, 0.0], "Exactly on front boundary", "Should be ≈ 0"),
        ([0.0, 0.3, 0.0], "Exactly on side boundary", "Should be ≈ 0"),
        ([-0.5, 0.0, 0.0], "Behind (0.5m)", "Should be ≈ 0 (boundary)"),
        ([1.0, 0.0, 0.0], "Far in front (1.0m)", "Should be >> 0"),
    ]
    
    print(f"Human pose: x={human_pose[0]}, y={human_pose[1]}, θ={human_pose[2]}")
    print(f"Ellipse: a={A_ELLIPSE}m (front/back), b={B_ELLIPSE}m (sides)")
    print()
    print(f"{'Robot Position':<25} {'h value':>10} {'Status':<15} {'Expected'}")
    print("-" * 80)
    
    for pos, label, expected in test_positions:
        robot_pose = np.array(pos)
        h_val = float(h_func(robot_pose, human_pose))
        status = "SAFE ✓" if h_val >= 0 else "VIOLATION ✗"
        print(f"{label:<25} {h_val:>10.3f} {status:<15} {expected}")
    
    print()
    
    # ── Test 2: Rotated human ─────────────────────────────────────────────────
    print("Test 2: Human at origin, heading along +y (θ = π/2)")
    print("-" * 60)
    
    # Human facing +y direction
    human_pose_rotated = np.array([0.0, 0.0, np.pi/2])
    
    test_positions_rotated = [
        ([0.0, 0.6, 0.0], "In front (+y, 0.6m)", "Should be >= 0"),
        ([0.0, 0.4, 0.0], "In front (+y, 0.4m)", "Should be < 0"),
        ([0.4, 0.0, 0.0], "To the side (+x, 0.4m)", "Should be >= 0"),
        ([0.2, 0.0, 0.0], "To the side (+x, 0.2m)", "Should be < 0"),
    ]
    
    print(f"Human pose: x={human_pose_rotated[0]}, y={human_pose_rotated[1]}, θ={human_pose_rotated[2]:.3f} (π/2)")
    print()
    print(f"{'Robot Position':<25} {'h value':>10} {'Status':<15} {'Expected'}")
    print("-" * 80)
    
    for pos, label, expected in test_positions_rotated:
        robot_pose = np.array(pos)
        h_val = float(h_func(robot_pose, human_pose_rotated))
        status = "SAFE ✓" if h_val >= 0 else "VIOLATION ✗"
        print(f"{label:<25} {h_val:>10.3f} {status:<15} {expected}")
    
    print()
    
    # ── Test 3: Multiple humans ───────────────────────────────────────────────
    print("Test 3: Multiple humans (K=3)")
    print("-" * 60)
    
    # Create multi-human constraint
    p_humans = create_human_parameter(K_HUMANS)
    h_multi = create_multi_human_constraints(x_robot, p_humans)
    h_multi_func = ca.Function("h_multi", [x_robot, p_humans], [h_multi])
    
    # Three humans at different positions
    humans_stacked = np.array([
        1.0, 0.0, 0.0,      # Human 1: at (1,0), facing +x
        0.0, 1.0, np.pi/2,  # Human 2: at (0,1), facing +y
        -1.0, 0.0, np.pi,   # Human 3: at (-1,0), facing -x
    ])
    
    # Robot at origin
    robot_at_origin = np.array([0.0, 0.0, 0.0])
    
    h_values = np.array(h_multi_func(robot_at_origin, humans_stacked)).flatten()
    
    print(f"Robot at origin (0, 0)")
    print(f"Human 1 at (1, 0) facing +x:  h = {h_values[0]:.3f} {'✓' if h_values[0] >= 0 else '✗'}")
    print(f"Human 2 at (0, 1) facing +y:  h = {h_values[1]:.3f} {'✓' if h_values[1] >= 0 else '✗'}")
    print(f"Human 3 at (-1, 0) facing -x: h = {h_values[2]:.3f} {'✓' if h_values[2] >= 0 else '✗'}")
    
    all_safe = all(h >= 0 for h in h_values)
    print(f"\nAll constraints satisfied: {'YES ✓' if all_safe else 'NO ✗'}")
    
    print()
    print("=" * 60)
    print("Constraint module ready for integration with acados solver")
    print("=" * 60)