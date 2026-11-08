#!/usr/bin/env python3
"""Kinematics study: ABB IRB 4600-40/255 on a linear rail (numpy only, no ROS needed).

What it shows
  1. DH table and the homogeneous matrix A_i of every joint
  2. Forward kinematics (FK): base -> paint_tcp, and world -> paint_tcp with the rail
  3. Check: DH forward kinematics == forward kinematics chained from the URDF joint origins
  4. Geometric Jacobian (6x6), checked against finite differences
  5. Analytic inverse kinematics (IK) by wrist decoupling, checked by FK -> IK -> FK
  6. Reach analysis: why the far half of the roof needs the turntable

Run:                    python3 kinematics_study.py
Compare with RViz/TF:   python3 kinematics_study.py --q 0 0.5 -0.3 0 1.3 -1.57 --rail -1.2
                        (then, in another terminal:  ros2 run tf2_ros tf2_echo world paint_tcp)
"""
import argparse
import numpy as np

pi = np.pi
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# ---------------------------------------------------------------- robot data
# Numbers come from irb4600_40_255_macro.xacro (joint origins) and cell.xacro (rail, mount, gun).
A1, D1 = 0.175, 0.495        # joint 2 offset forward / joint 1 height
L1 = 1.095                   # upper arm (joint 2 -> joint 3)
A3, D4 = 0.175, 1.270        # forearm: 0.175 up, 1.270 along
D6 = 0.135                   # wrist centre -> flange
GUN = 0.31                   # flange (tool0) -> paint_tcp along the tool z axis
L2 = np.hypot(A3, D4)        # effective forearm length = 1.2819
DELTA = np.arctan2(A3, D4)   # forearm offset angle = 7.85 deg
LIMITS = np.radians([(-180, 180), (-90, 150), (-180, 75), (-400, 400), (-125, 120), (-400, 400)])

RAIL_Y, BASE_Z, BASE_YAW = -2.0, 0.75, pi / 2   # rail line y, base height (0.10 + 0.65), base yaw

# classic Denavit-Hartenberg table: theta_i = q_i + offset_i
#            offset   d_i    a_i    alpha_i
DH = [(0.0,     D1,   A1,   -pi / 2),
      (-pi / 2, 0.0,  L1,    0.0),
      (0.0,     0.0,  A3,   -pi / 2),
      (0.0,     D4,   0.0,   pi / 2),
      (0.0,     0.0,  0.0,  -pi / 2),
      (0.0,     D6,   0.0,   0.0)]

# fixed transform from DH frame 6 to paint_tcp: rotate 180 deg about z, then GUN along z
T6_TCP = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, GUN], [0, 0, 0, 1.0]])


# ---------------------------------------------------------------- helpers
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T(R=None, p=(0, 0, 0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = p
    return M


def wrap(a):
    return (a + pi) % (2 * pi) - pi


def dh_matrix(theta, d, a, alpha):
    """Homogeneous matrix A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1.0]])


# ---------------------------------------------------------------- forward kinematics
def frames(q):
    """Return [T_0^0, T_0^1, ..., T_0^6] (base -> DH frame i)."""
    out = [np.eye(4)]
    for (off, d, a, al), qi in zip(DH, q):
        out.append(out[-1] @ dh_matrix(off + qi, d, a, al))
    return out


def fk_base(q):
    """Homogeneous matrix base_link -> paint_tcp."""
    return frames(q)[-1] @ T6_TCP


def fk_urdf(q):
    """Same thing, chained from the URDF joint origins and axes (independent check)."""
    q1, q2, q3, q4, q5, q6 = q
    M = T(p=(0, 0, D1)) @ T(Rz(q1))
    M = M @ T(p=(A1, 0, 0)) @ T(Ry(q2))
    M = M @ T(p=(0, 0, L1)) @ T(Ry(q3))
    M = M @ T(p=(0, 0, A3)) @ T(Rx(q4))
    M = M @ T(p=(D4, 0, 0)) @ T(Ry(q5))
    M = M @ T(p=(D6, 0, 0)) @ T(Rx(q6))
    M = M @ T(Ry(pi / 2))            # flange -> tool0  (rpy 0 90deg 0)
    M = M @ T(p=(0, 0, GUN))         # gun -> paint_tcp
    return M


def base_in_world(rail):
    """world -> base_link: rail slides along x, base sits at height 0.75 and is yawed 90 deg."""
    return T(Rz(BASE_YAW), (rail, RAIL_Y, BASE_Z))


def fk_world(q, rail):
    return base_in_world(rail) @ fk_base(q)


# ---------------------------------------------------------------- Jacobian
def jacobian(q):
    """Geometric Jacobian (6x6) in the base frame: [v; w] = J qdot.
    Column i (revolute): [ z_{i-1} x (p_tcp - o_{i-1}) ; z_{i-1} ]."""
    F = frames(q)
    p_tcp = (F[-1] @ T6_TCP)[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = F[i][:3, 2]
        o = F[i][:3, 3]
        J[:3, i] = np.cross(z, p_tcp - o)
        J[3:, i] = z
    return J


def jacobian_numeric(q, eps=1e-6):
    q = np.asarray(q, float)
    T0 = fk_base(q)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        T1 = fk_base(dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
        dR = T1[:3, :3] @ T0[:3, :3].T                     # small rotation in the base frame
        J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * eps)
    return J


# ---------------------------------------------------------------- inverse kinematics
def ik(T_base_tcp, apply_limits=True):
    """Analytic IK by wrist decoupling. Returns a list of 6-vectors (up to 8 solutions).
    Axes 4, 5, 6 meet at the wrist centre (= joint 5 origin), so position and orientation split:
      step 1  wrist centre  p_w = p_tcp - (D6 + GUN) * z_tcp
      step 2  q1 = atan2(y, x)   (or +pi: arm reaching backwards)
      step 3  q3 from the law of cosines in the arm plane (elbow up / down)
      step 4  q2 from the triangle angles
      step 5  q4, q5, q6 from R_3^6 = Rx(q4) Ry(q5) Rx(q6)   (wrist flip: 2 solutions)"""
    R = T_base_tcp[:3, :3]
    p = T_base_tcp[:3, 3]
    pw = p - (D6 + GUN) * R[:, 2]
    R_link6 = R @ Ry(-pi / 2)
    sols = []
    for back in (False, True):
        q1 = np.arctan2(pw[1], pw[0]) + (pi if back else 0.0)
        rho = np.hypot(pw[0], pw[1]) * (-1.0 if back else 1.0)
        r, h = rho - A1, pw[2] - D1
        D2 = r * r + h * h
        c = (L1 ** 2 + L2 ** 2 - D2) / (2 * L1 * L2)          # = sin(q3 - delta)
        if abs(c) > 1.0:
            continue                                            # wrist centre out of reach
        for elbow in (0, 1):
            s = np.arcsin(c)
            q3 = DELTA + (s if elbow == 0 else pi - s)
            kappa = pi / 2 + q3 - DELTA
            eps = np.arctan2(L2 * np.sin(kappa), L1 + L2 * np.cos(kappa))
            q2 = np.arctan2(r, h) - eps
            W = Ry(q2 + q3).T @ Rz(q1).T @ R_link6              # = Rx(q4) Ry(q5) Rx(q6)
            sb = np.hypot(W[1, 0], W[2, 0])
            for flip in (0, 1):
                if sb > 1e-9:
                    q5 = np.arctan2(sb, W[0, 0])
                    q4 = np.arctan2(W[1, 0], -W[2, 0])
                    q6 = np.arctan2(W[0, 1], W[0, 2])
                    if flip:
                        q5, q4, q6 = -q5, q4 + pi, q6 + pi
                else:                                           # wrist singularity: q4 and q6 are coupled
                    if flip:
                        continue
                    q5, q4, q6 = 0.0, 0.0, np.arctan2(W[2, 1], W[1, 1])
                q = np.array([wrap(x) for x in (q1, q2, q3, q4, q5, q6)])
                inside = np.all(q >= LIMITS[:, 0] - 1e-9) and np.all(q <= LIMITS[:, 1] + 1e-9)
                if apply_limits and not inside:
                    continue
                sols.append(q)
    return sols


def pose_error(T_a, T_b):
    return np.abs(T_a - T_b).max()


def tool_down(x, y, z):
    """paint_tcp pose with the spray (tool +z) pointing straight down: quaternion (1,0,0,0) = Rx(180 deg)."""
    return T(Rx(pi), (x, y, z))


# ---------------------------------------------------------------- sections
def section_dh():
    print("\n=== 1. DH table (classic):  A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i),  theta_i = q_i + offset_i")
    print(" i   offset[deg]   d[m]    a[m]    alpha[deg]")
    for i, (off, d, a, al) in enumerate(DH, 1):
        print(" %d   %8.1f   %6.3f  %6.3f  %8.1f" % (i, np.degrees(off), d, a, np.degrees(al)))
    print("tool: T_6_tcp =\n", T6_TCP)


def section_fk():
    print("\n=== 2/3. Forward kinematics and DH-vs-URDF check")
    rng = np.random.default_rng(1)
    err = 0.0
    for _ in range(500):
        q = rng.uniform(LIMITS[:, 0], LIMITS[:, 1])
        err = max(err, pose_error(fk_base(q), fk_urdf(q)))
    print("max |DH - URDF| over 500 random poses: %.2e" % err)
    q = np.array([0.0, 0.5219, -0.3177, 0.0, 1.3666, -1.5708])
    print("example q =", q)
    print("T_base_tcp =\n", fk_base(q))
    print("T_world_tcp (rail = -1.2) =\n", fk_world(q, -1.2))


def section_jacobian():
    print("\n=== 4. Jacobian")
    rng = np.random.default_rng(2)
    err = 0.0
    for _ in range(100):
        q = rng.uniform(LIMITS[:, 0], LIMITS[:, 1])
        err = max(err, np.abs(jacobian(q) - jacobian_numeric(q)).max())
    print("max |analytic - numeric| over 100 poses: %.2e" % err)
    q = np.array([0.0, 0.5219, -0.3177, 0.0, 1.3666, -1.5708])
    J = jacobian(q)
    print("J(q) =\n", J)
    print("rank = %d,  det = %.4f,  singular values = %s" % (np.linalg.matrix_rank(J), np.linalg.det(J),
                                                           np.linalg.svd(J, compute_uv=False)))
    sing = np.array([0.0, 0.5, -1.43, 0.0, 0.0, 0.0])          # wrist straight (q5 = 0) -> singular
    print("det J at q5 = 0 (wrist singularity): %.2e" % np.linalg.det(jacobian(sing)))


def section_ik():
    print("\n=== 5. Analytic IK, round trip FK -> IK -> FK")
    rng = np.random.default_rng(3)
    worst, counts, missed = 0.0, [], 0
    for _ in range(300):
        q = rng.uniform(LIMITS[:, 0], LIMITS[:, 1])
        T0 = fk_base(q)
        sols = ik(T0)
        counts.append(len(sols))
        if not any(np.allclose(wrap(s - q), 0, atol=1e-6) for s in sols):
            missed += 1
        for s in sols:
            worst = max(worst, pose_error(fk_base(s), T0))
    print("solutions per pose: min %d, max %d, mean %.1f" % (min(counts), max(counts), np.mean(counts)))
    print("worst pose error of any returned solution: %.2e   (true q not among the solutions: %d of 300)" % (worst, missed))


def section_reach():
    print("\n=== 6. Reach analysis (gun pointing straight down)")
    sh_y, sh_z = RAIL_Y + A1, BASE_Z + D1                       # shoulder = joint 2 (arm pointing along +y)
    r_max = L1 + L2
    print("shoulder (joint 2) at world y = %.3f, z = %.3f;  wrist-centre reach limit = L1 + L2 = %.4f m" %
          (sh_y, sh_z, r_max))
    print("wrist centre sits %.3f m above the tool tip (D6 + GUN)" % (D6 + GUN))
    print("largest reachable y for a given roof-top gun height z (gun down, rail at the stripe x):")
    for z in (1.55, 1.60, 1.65, 1.70, 1.75):
        dz = z + D6 + GUN - sh_z
        print("   z = %.2f  ->  y_max = %+.3f m" % (z, sh_y + np.sqrt(r_max ** 2 - dz ** 2)))
    print("\nfar_00 stripe, car NOT turned (world y positive):")
    for y, z in [(0.0, 1.663), (0.2, 1.6562), (0.4, 1.6356), (0.41, 1.63437), (0.5, 1.6233)]:
        n = len(ik(np.linalg.inv(base_in_world(-1.2)) @ tool_down(-1.2, y, z)))
        d = np.hypot(y - sh_y, (z + D6 + GUN) - sh_z)
        print("   y = %.2f z = %.4f   wrist distance %.4f m   IK solutions: %d" % (y, z, d, n))
    print("same stripe after the 180 deg turntable flip: car point (-1.2, y) lands at world (+1.2, -y), rail moves to x = +1.2")
    for y, z in [(0.0, 1.663), (0.4, 1.6356), (0.5, 1.6233)]:
        n = len(ik(np.linalg.inv(base_in_world(1.2)) @ tool_down(1.2, -y, z)))
        print("   car y = %.2f -> world y = %+.2f   IK solutions: %d" % (y, -y, n))
    print("\nelbow fully stretched when q3 = delta - 90 deg = %.2f deg  (waypoint 5 of far_00 had q3 = -80.4 deg)" %
          np.degrees(DELTA - pi / 2))


def section_pose(q, rail):
    print("\n=== FK of the given joint values (compare with: ros2 run tf2_ros tf2_echo world paint_tcp)")
    Tw = fk_world(np.array(q, float), rail)
    print("T_world_tcp =\n", Tw)
    print("position xyz:", Tw[:3, 3], "  z axis (spray direction):", Tw[:3, 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q", nargs=6, type=float, help="joint_1..joint_6 in radians")
    ap.add_argument("--rail", type=float, default=0.0, help="rail_joint in metres")
    args = ap.parse_args()
    if args.q:
        section_pose(args.q, args.rail)
        return
    section_dh()
    section_fk()
    section_jacobian()
    section_ik()
    section_reach()


if __name__ == "__main__":
    main()