"""
MANTA — corrected architecture: arm-braced wingsuit-extension wing.

This is the SOLE source of truth for MANTA CAD. It supersedes the prior
broken aircraft-strapped-on-back integration. The pilot is the fuselage;
the structure is an arm-aligned LE spar + body-mounted TE spar + telescoping
tip extensions, all packaged into a wingsuit-derivative harness.

Frame
-----
Origin at pilot's center of mass.
    +x: forward (direction of flight, head pointing this way)
    +y: starboard (right side)
    +z: up (out of pilot's back, away from belly)

Deploy state
------------
Single parameter `deploy_state ∈ [0, 1]`:
    0.00 - 0.30   stowed — arms tucked along torso, legs together
    0.30 - 0.50   Phase A — arms + legs spread to deployed sweep
    0.50 - 0.70   Phase B — wrist + ankle telescoping tip extensions snap out
    0.70 - 1.00   Phase D — wing skin tensions across the deployed structure

Phase C (rib snap-through) happens passively as spars pass each rib coil; we
don't model individual ribs in this CAD — they're geometry-implicit in the
skin tensioning.

Outputs
-------
    cad/out/parts/{part}.stl   per-part STLs for the viewer
    cad/out/full_deployed.stl  whole assembly at deploy_state = 1
    cad/out/full_stowed.stl    whole assembly at deploy_state = 0
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402


# ---------------------------------------------------------------------------
# Anthropometry — typical adult skydiving pilot
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Anthro:
    head_diam: float = 0.20
    neck_len: float = 0.08
    torso_len: float = 0.55
    torso_w: float = 0.36
    torso_t: float = 0.22
    shoulder_y: float = 0.18           # half-distance between shoulders
    upper_arm_len: float = 0.30
    upper_arm_diam: float = 0.10
    forearm_len: float = 0.27
    forearm_diam: float = 0.08
    hand_len: float = 0.20
    hip_y: float = 0.12                # half-distance between hips
    upper_leg_len: float = 0.45
    upper_leg_diam: float = 0.15
    lower_leg_len: float = 0.42
    lower_leg_diam: float = 0.10
    foot_len: float = 0.27


# ---------------------------------------------------------------------------
# Architecture parameters
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Architecture:
    # Spine yoke (CFRP backbone along the pilot's spine, integrated into harness)
    spine_yoke_len: float = 0.65
    spine_yoke_diam: float = 0.045

    # Shoulder pivot — where the LE spar hinges off the spine yoke
    shoulder_pivot_x: float = 0.30          # forward of CG (chest level)
    shoulder_pivot_z: float = 0.10          # above pilot's body axis (above belly)

    # LE spar (along arm)
    le_spar_diam: float = 0.062             # 62 mm OD root (close to bending-sized 73mm but more anatomically reasonable for arm fit)
    le_spar_wall: float = 0.0025

    # Wrist tip extension (telescoping, 3-stage)
    wrist_ext_n_stages: int = 3
    wrist_ext_total_len: float = 3.30       # full deployed length from wrist
    wrist_ext_od_root: float = 0.046         # 46 mm at root of tip extension (smaller than arm spar OD)
    wrist_ext_od_tip: float = 0.020          # 20 mm at far tip

    # TE spar — body-mounted spanwise boom (NOT along leg)
    te_hub_x: float = -0.55                  # aft of CG (lower back / hip area)
    te_hub_z: float = 0.05
    te_spar_diam_root: float = 0.040
    te_spar_total_len: float = 3.70          # full deployed half-span (covers full TE)
    te_spar_n_stages: int = 3
    te_spar_od_tip: float = 0.018

    # Hip pivot — for the leg motion (purely cosmetic in flight; legs don't bear load)
    hip_pivot_x: float = -0.20
    hip_pivot_z: float = -0.02

    # Stowed arm angle (close to body, slightly forward, like in rest)
    arm_stow_sweep_deg: float = 0.0          # straight aft along body
    arm_stow_dive_deg: float = 70.0          # arms hanging down close to torso

    # Deployed arm angle — arms aligned with the wing's LE
    arm_deploy_sweep_deg: float = 25.0       # matches wing LE sweep (aft of perpendicular)
    arm_deploy_dive_deg: float = 0.0         # arms in horizontal plane (level with pilot body)

    # Stowed leg angle
    leg_stow_sweep_deg: float = 0.0          # legs straight aft
    leg_stow_dive_deg: float = 0.0
    leg_spread_deg: float = 0.0              # legs together

    # Deployed leg angle
    leg_deploy_spread_deg: float = 25.0      # legs spread laterally

    # Wing root chord & TE distances driven by analysis/aero/planform/geometry.py


# ---------------------------------------------------------------------------
# Deploy-state mappings
# ---------------------------------------------------------------------------

def _ease(t: float) -> float:
    """Smooth cubic ease (3t² − 2t³)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def phase_A(s: float) -> float:
    """Phase A: arm+leg spread (s ∈ [0.30, 0.50])."""
    return _ease((s - 0.30) / 0.20) if 0.30 <= s <= 0.50 else (0.0 if s < 0.30 else 1.0)


def phase_B(s: float) -> float:
    """Phase B: wrist+ankle telescoping tip extension (s ∈ [0.50, 0.70])."""
    return _ease((s - 0.50) / 0.20) if 0.50 <= s <= 0.70 else (0.0 if s < 0.50 else 1.0)


def phase_D(s: float) -> float:
    """Phase D: skin tension (s ∈ [0.70, 1.00])."""
    return _ease((s - 0.70) / 0.30) if 0.70 <= s <= 1.00 else (0.0 if s < 0.70 else 1.0)


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

def _capsule(length: float, diam: float, axis: tuple[float, float, float]) -> cq.Workplane:
    """Cylindrical capsule along the given axis vector. Returned at origin
    with one end at the origin, extending in +axis direction."""
    ax = (axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) ** 0.5
    if ax < 1e-9:
        return cq.Workplane("XY").box(0.001, 0.001, 0.001)
    a = (axis[0] / ax, axis[1] / ax, axis[2] / ax)
    plane = cq.Plane(origin=cq.Vector(0, 0, 0),
                     xDir=cq.Vector(0, 0, 1) if abs(a[2]) < 0.9 else cq.Vector(1, 0, 0),
                     normal=cq.Vector(*a))
    return cq.Workplane(plane).circle(diam / 2).extrude(length)


def _ellipsoid(rx: float, ry: float, rz: float, n: int = 12) -> cq.Workplane:
    """Approximate ellipsoid. Use a sphere scaled — CadQuery sphere then scale."""
    sphere = cq.Workplane("XY").sphere(1.0)
    # Scale by transforming each face
    matrix = cq.Matrix([
        [rx, 0, 0, 0],
        [0, ry, 0, 0],
        [0, 0, rz, 0],
    ])
    sphere = sphere.val().transformGeometry(matrix)  # type: ignore[attr-defined]
    return cq.Workplane(obj=sphere)


def _hollow_cyl(length: float, od: float, id_: float) -> cq.Workplane:
    """Hollow tube along +z."""
    if id_ <= 0:
        return cq.Workplane("XY").circle(od / 2).extrude(length)
    return (
        cq.Workplane("XY")
        .circle(od / 2)
        .circle(id_ / 2)
        .extrude(length)
    )


def _segment_tube(P_in: tuple[float, float, float],
                   P_out: tuple[float, float, float],
                   od: float, id_: float = 0.0) -> cq.Workplane:
    """Tube from P_in to P_out with OD/ID."""
    dx = P_out[0] - P_in[0]
    dy = P_out[1] - P_in[1]
    dz = P_out[2] - P_in[2]
    L = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
    if L < 1e-9:
        return cq.Workplane("XY").box(0.001, 0.001, 0.001)
    n = (dx / L, dy / L, dz / L)
    plane = cq.Plane(
        origin=cq.Vector(*P_in),
        xDir=cq.Vector(0, 0, 1) if abs(n[2]) < 0.9 else cq.Vector(1, 0, 0),
        normal=cq.Vector(*n),
    )
    outer = cq.Workplane(plane).circle(od / 2).extrude(L)
    if id_ > 0:
        inner = cq.Workplane(plane).circle(id_ / 2).extrude(L)
        return outer.cut(inner)
    return outer


# ---------------------------------------------------------------------------
# Pilot — humanoid in flight pose
# ---------------------------------------------------------------------------

def build_pilot(deploy: float, anthro: Anthro = Anthro(),
                  arch: Architecture = Architecture()) -> dict:
    """Return a dict of part_name -> Workplane for the pilot in current pose."""
    parts = {}

    # Torso — box centered between chest and hip
    torso_x_center = (arch.shoulder_pivot_x + arch.hip_pivot_x) / 2
    torso = (
        cq.Workplane("XY")
        .box(anthro.torso_len, anthro.torso_w, anthro.torso_t)
        .translate((torso_x_center, 0, 0))
    )
    parts["torso"] = torso

    # Head — sphere forward of torso
    head_x = arch.shoulder_pivot_x + 0.12
    head = (
        cq.Workplane("XY")
        .sphere(anthro.head_diam / 2)
        .translate((head_x, 0, 0.05))
    )
    parts["head"] = head

    # Arms — left and right, pose interpolated by phase A
    a = phase_A(deploy)
    arm_sweep = (1 - a) * arch.arm_stow_sweep_deg + a * arch.arm_deploy_sweep_deg
    arm_dive = (1 - a) * arch.arm_stow_dive_deg + a * arch.arm_deploy_dive_deg

    arm_yaw_rad = math.radians(arm_sweep)        # rotation about +z (sweep aft)
    arm_pitch_rad = math.radians(arm_dive)       # rotation about +x... no, about +y
    # Direction of arm in body frame:
    #   start out along +y (perpendicular to body), then rotate
    #   - pitch (dive): rotate about +x, dropping the arm down (-z direction)
    #   - yaw (sweep): rotate about +z, sweeping aft (-x direction)
    cos_p, sin_p = math.cos(arm_pitch_rad), math.sin(arm_pitch_rad)
    cos_y, sin_y = math.cos(arm_yaw_rad), math.sin(arm_yaw_rad)
    arm_dir_right = (
        -sin_y * cos_p,                          # x component (aft sweep)
        +cos_y * cos_p,                          # y component (lateral)
        -sin_p,                                  # z component (down)
    )
    arm_dir_left = (
        -sin_y * cos_p,
        -cos_y * cos_p,
        -sin_p,
    )

    arm_total_len = anthro.upper_arm_len + anthro.forearm_len + anthro.hand_len * 0.5

    for side, arm_dir in [("right", arm_dir_right), ("left", arm_dir_left)]:
        sign = +1 if side == "right" else -1
        sh_x = arch.shoulder_pivot_x
        sh_y = sign * anthro.shoulder_y
        sh_z = arch.shoulder_pivot_z
        wr_x = sh_x + arm_total_len * arm_dir[0]
        wr_y = sh_y + arm_total_len * arm_dir[1]
        wr_z = sh_z + arm_total_len * arm_dir[2]

        # Upper arm
        ua = _segment_tube(
            (sh_x, sh_y, sh_z),
            (sh_x + anthro.upper_arm_len * arm_dir[0],
             sh_y + anthro.upper_arm_len * arm_dir[1],
             sh_z + anthro.upper_arm_len * arm_dir[2]),
            anthro.upper_arm_diam,
        )
        # Forearm
        elbow = (
            sh_x + anthro.upper_arm_len * arm_dir[0],
            sh_y + anthro.upper_arm_len * arm_dir[1],
            sh_z + anthro.upper_arm_len * arm_dir[2],
        )
        fa = _segment_tube(
            elbow,
            (elbow[0] + anthro.forearm_len * arm_dir[0],
             elbow[1] + anthro.forearm_len * arm_dir[1],
             elbow[2] + anthro.forearm_len * arm_dir[2]),
            anthro.forearm_diam,
        )
        # Hand
        wrist = (
            elbow[0] + anthro.forearm_len * arm_dir[0],
            elbow[1] + anthro.forearm_len * arm_dir[1],
            elbow[2] + anthro.forearm_len * arm_dir[2],
        )
        hand = _segment_tube(
            wrist,
            (wrist[0] + anthro.hand_len * arm_dir[0],
             wrist[1] + anthro.hand_len * arm_dir[1],
             wrist[2] + anthro.hand_len * arm_dir[2]),
            0.06,
        )

        parts[f"upper_arm_{side}"] = ua
        parts[f"forearm_{side}"] = fa
        parts[f"hand_{side}"] = hand

        # Save wrist position for tip-extension placement
        parts[f"_wrist_{side}"] = wrist

    # Legs
    leg_spread = a * arch.leg_deploy_spread_deg
    leg_total_len = anthro.upper_leg_len + anthro.lower_leg_len + anthro.foot_len * 0.5

    for side in ("right", "left"):
        sign = +1 if side == "right" else -1
        hip_x = arch.hip_pivot_x
        hip_y = sign * anthro.hip_y
        hip_z = arch.hip_pivot_z

        # Leg direction: in stowed straight aft (-x); in deployed, splayed laterally
        # Phase A interpolates the spread.
        spread_rad = math.radians(leg_spread)
        # Stowed: leg points aft (-x). Deployed: leg points outward (±y) and slightly aft (-x).
        # Interpolate as: dir = (cos(spread)*-1, sign*sin(spread), 0) which at spread=0 is (-1, 0, 0)
        # and at spread=25° is (-cos(25°)*1, sign*sin(25°), 0) — ~hip-spread orientation.
        leg_dir = (
            -math.cos(spread_rad),
            sign * math.sin(spread_rad),
            0.0,
        )

        # Upper leg
        ul = _segment_tube(
            (hip_x, hip_y, hip_z),
            (hip_x + anthro.upper_leg_len * leg_dir[0],
             hip_y + anthro.upper_leg_len * leg_dir[1],
             hip_z + anthro.upper_leg_len * leg_dir[2]),
            anthro.upper_leg_diam,
        )
        knee = (
            hip_x + anthro.upper_leg_len * leg_dir[0],
            hip_y + anthro.upper_leg_len * leg_dir[1],
            hip_z + anthro.upper_leg_len * leg_dir[2],
        )
        ll = _segment_tube(
            knee,
            (knee[0] + anthro.lower_leg_len * leg_dir[0],
             knee[1] + anthro.lower_leg_len * leg_dir[1],
             knee[2] + anthro.lower_leg_len * leg_dir[2]),
            anthro.lower_leg_diam,
        )
        ankle = (
            knee[0] + anthro.lower_leg_len * leg_dir[0],
            knee[1] + anthro.lower_leg_len * leg_dir[1],
            knee[2] + anthro.lower_leg_len * leg_dir[2],
        )

        parts[f"upper_leg_{side}"] = ul
        parts[f"lower_leg_{side}"] = ll
        parts[f"_ankle_{side}"] = ankle

    return parts


# ---------------------------------------------------------------------------
# Structure — spine yoke, LE spar, TE spar, telescoping tips
# ---------------------------------------------------------------------------

def build_structure(deploy: float, pilot_parts: dict,
                     arch: Architecture = Architecture(),
                     anthro: Anthro = Anthro()) -> dict:
    parts = {}

    # Spine yoke — runs along the back of the torso (z = +torso_t/2 + small offset)
    z_back = anthro.torso_t / 2 + 0.012
    spine = _segment_tube(
        (arch.shoulder_pivot_x - 0.05, 0, z_back),
        (arch.te_hub_x + 0.05, 0, z_back),
        arch.spine_yoke_diam,
        arch.spine_yoke_diam - 0.008,
    )
    parts["spine_yoke"] = spine

    # LE spar — runs from shoulder pivot along the arm direction. We replicate
    # the arm direction vector and place a stiff CFRP tube alongside the arm
    # (on the underside, z ≈ shoulder z but the spar carries load).
    arm_total_len = anthro.upper_arm_len + anthro.forearm_len + anthro.hand_len * 0.5

    a = phase_A(deploy)
    arm_yaw_rad = math.radians((1 - a) * arch.arm_stow_sweep_deg + a * arch.arm_deploy_sweep_deg)
    arm_pitch_rad = math.radians((1 - a) * arch.arm_stow_dive_deg + a * arch.arm_deploy_dive_deg)
    cos_p, sin_p = math.cos(arm_pitch_rad), math.sin(arm_pitch_rad)
    cos_y, sin_y = math.cos(arm_yaw_rad), math.sin(arm_yaw_rad)

    for side in ("right", "left"):
        sign = +1 if side == "right" else -1
        arm_dir = (-sin_y * cos_p, sign * cos_y * cos_p, -sin_p)
        sh_x = arch.shoulder_pivot_x
        sh_y = sign * anthro.shoulder_y
        sh_z = arch.shoulder_pivot_z
        wr_x = sh_x + arm_total_len * arm_dir[0]
        wr_y = sh_y + arm_total_len * arm_dir[1]
        wr_z = sh_z + arm_total_len * arm_dir[2]

        # LE spar runs along the arm but offset in the +z direction by a small amount
        # so it sits on TOP of the arm (visible alongside, doesn't intersect arm cylinder)
        offset_z = anthro.upper_arm_diam / 2 + arch.le_spar_diam / 2 + 0.005
        spar_in = (sh_x, sh_y, sh_z + offset_z)
        spar_out = (wr_x, wr_y, wr_z + offset_z)
        parts[f"le_spar_{side}"] = _segment_tube(
            spar_in, spar_out, arch.le_spar_diam, arch.le_spar_diam - 2 * arch.le_spar_wall
        )

        # Wrist tip extension — telescoping, 3 stages, in the same direction
        # as the arm spar (along the LE).
        b = phase_B(deploy)
        ext_total = arch.wrist_ext_total_len * (0.05 + 0.95 * b)   # stowed: 5% out (collapsed), deployed: full
        # OD per stage: linearly interpolated from root to tip
        stage_lens = ext_total / arch.wrist_ext_n_stages
        stage_x_dir = arm_dir
        cur = spar_out
        for i in range(arch.wrist_ext_n_stages):
            od = arch.wrist_ext_od_root + (arch.wrist_ext_od_tip - arch.wrist_ext_od_root) * (i + 0.5) / arch.wrist_ext_n_stages
            wall = 0.0020
            stage_in = cur
            stage_out = (
                cur[0] + stage_lens * stage_x_dir[0],
                cur[1] + stage_lens * stage_x_dir[1],
                cur[2] + stage_lens * stage_x_dir[2],
            )
            parts[f"wrist_ext_{side}_stage{i+1}"] = _segment_tube(
                stage_in, stage_out, od, od - 2 * wall
            )
            cur = stage_out

        # Save wingtip endpoint
        parts[f"_le_tip_{side}"] = cur

    # TE spar — body-mounted hub at the lower back, deploys SPANWISE (along ±y),
    # with a telescoping section that extends to the wingtip TE position.
    # The TE root is at the spine end; deployed direction is approximately
    # along +y for right side, but with a small -x component to match TE sweep.
    p = Planform()
    te_sweep_rad = math.radians(p.sweep_te_deg)
    for side in ("right", "left"):
        sign = +1 if side == "right" else -1
        te_root = (arch.te_hub_x, 0, arch.te_hub_z)
        # Direction: laterally outward (sign * +y) with -x component for TE sweep
        # NOTE: at deploy=0, the TE is folded with the spine yoke (small lateral extent).
        a_phase = phase_A(deploy)
        b_phase = phase_B(deploy)
        deploy_amt = a_phase * 0.5 + b_phase * 0.5
        te_dir = (-math.sin(te_sweep_rad), sign * math.cos(te_sweep_rad), 0.0)
        # Total TE spar length is half-span / cos(TE_sweep)
        full_te_len = arch.te_spar_total_len / math.cos(te_sweep_rad)
        cur = te_root
        n_stages = arch.te_spar_n_stages
        deployed_len = full_te_len * (0.05 + 0.95 * deploy_amt)
        stage_len = deployed_len / n_stages
        for i in range(n_stages):
            od_frac = (i + 0.5) / n_stages
            od = arch.te_spar_diam_root + (arch.te_spar_od_tip - arch.te_spar_diam_root) * od_frac
            wall = 0.0018
            stage_in = cur
            stage_out = (
                cur[0] + stage_len * te_dir[0],
                cur[1] + stage_len * te_dir[1],
                cur[2] + stage_len * te_dir[2],
            )
            parts[f"te_spar_{side}_stage{i+1}"] = _segment_tube(
                stage_in, stage_out, od, od - 2 * wall
            )
            cur = stage_out
        parts[f"_te_tip_{side}"] = cur

    return parts


# ---------------------------------------------------------------------------
# Wing skin — lofted between LE and TE control points
# ---------------------------------------------------------------------------

def build_skin(deploy: float, structure_parts: dict,
                arch: Architecture = Architecture(),
                anthro: Anthro = Anthro()) -> dict:
    """Wing OML — a single lofted surface between LE and TE control points
    at the deploy state. At deploy=0 the surface degenerates to nearly
    nothing (collapsed against the body)."""
    parts = {}
    d = phase_D(deploy)
    if d < 0.05:
        return parts

    p = Planform()

    # Define LE and TE control points at multiple span stations, interpolated
    # between the body (centerline) and the wingtip.
    # LE at root: at the shoulder pivot point on the centerline (chest LE)
    # LE at outboard: along the wrist tip extension, from wrist outward
    # TE at root: at the body, far aft (root TE = pilot's mid-thigh region)
    # TE at outboard: at the TE spar tip

    le_tip_right = structure_parts.get("_le_tip_right")
    le_tip_left = structure_parts.get("_le_tip_left")
    te_tip_right = structure_parts.get("_te_tip_right")
    te_tip_left = structure_parts.get("_te_tip_left")
    if not all([le_tip_right, le_tip_left, te_tip_right, te_tip_left]):
        return parts

    # Build a swept lofted wing using cosine interpolation
    n = 8
    le_root = (arch.shoulder_pivot_x, 0, arch.shoulder_pivot_z)
    te_root = (arch.te_hub_x, 0, arch.te_hub_z)

    def section_pts(eta: float, side: int) -> list[tuple[float, float, float]]:
        """Return airfoil section coordinates at fractional half-span eta on side."""
        # LE x, y, z at this eta — interpolate between root (eta=0) and tip (eta=1)
        if side == +1:
            le_tip = le_tip_right
            te_tip = te_tip_right
        else:
            le_tip = le_tip_left
            te_tip = te_tip_left

        le = (
            le_root[0] * (1 - eta) + le_tip[0] * eta,
            le_root[1] * (1 - eta) + le_tip[1] * eta,
            le_root[2] * (1 - eta) + le_tip[2] * eta,
        )
        te = (
            te_root[0] * (1 - eta) + te_tip[0] * eta,
            te_root[1] * (1 - eta) + te_tip[1] * eta,
            te_root[2] * (1 - eta) + te_tip[2] * eta,
        )
        chord = math.sqrt(sum((le[i] - te[i]) ** 2 for i in range(3)))
        if chord < 1e-6:
            return []

        # Build a thin airfoil shape parametrically, twisted aligned LE→TE vector
        # We'll use a simplified flat-plate-with-thickness section (NACA 0010-like
        # but very thin so the rendered skin fills the wing volume).
        thickness = 0.10 * chord * d        # scaled by skin tension factor
        # 8 perimeter points around the section
        n_perim = 16
        # x_chord interpolation from LE (0) to TE (1)
        section = []
        for i in range(n_perim):
            t = i / n_perim
            # 0 to 0.5 is upper, 0.5 to 1.0 is lower
            if t < 0.5:
                x_c = 2 * t           # 0 → 1
                z_c = +thickness / 2 * math.sin(math.pi * x_c)
            else:
                x_c = 2 * (1 - t)     # 1 → 0
                z_c = -thickness / 2 * math.sin(math.pi * x_c)
            # 3D: position = LE + x_c * (TE - LE) + z_c * up_vector
            wx = le[0] + x_c * (te[0] - le[0])
            wy = le[1] + x_c * (te[1] - le[1])
            wz = le[2] + x_c * (te[2] - le[2]) + z_c
            section.append((wx, wy, wz))
        return section

    # Build right wing skin via loft
    for side in (+1, -1):
        wires = []
        for i in range(n + 1):
            eta = i / n
            pts = section_pts(eta, side)
            if not pts:
                continue
            vecs = [cq.Vector(*p_) for p_ in pts]
            wires.append(cq.Wire.makePolygon(vecs, forConstruction=False, close=True))
        if len(wires) >= 2:
            try:
                solid = cq.Solid.makeLoft(wires, ruled=False)
                parts[f"skin_{'right' if side > 0 else 'left'}"] = cq.Workplane(obj=solid)
            except Exception:
                pass

    return parts


# ---------------------------------------------------------------------------
# Top-level driver
# ---------------------------------------------------------------------------

def build_full(deploy: float = 1.0):
    pilot = build_pilot(deploy)
    structure = build_structure(deploy, pilot)
    skin = build_skin(deploy, structure)
    full = {}
    full.update({k: v for k, v in pilot.items() if not k.startswith("_")})
    full.update({k: v for k, v in structure.items() if not k.startswith("_")})
    full.update(skin)
    return full


def export_parts(parts: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, wp in parts.items():
        try:
            solids = list(wp.solids().vals())
            if not solids:
                # Try .val() compound path
                try:
                    val = wp.val()
                    cq.exporters.export(val, str(out_dir / f"{name}.stl"),
                                         tolerance=0.001, angularTolerance=0.5)
                    continue
                except Exception:
                    continue
            comp = cq.Compound.makeCompound(solids)
            cq.exporters.export(comp, str(out_dir / f"{name}.stl"),
                                tolerance=0.001, angularTolerance=0.5)
        except Exception as exc:
            print(f"    skip {name}: {exc}")


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    parts_dir = out_dir / "parts"
    parts_dir.mkdir(parents=True, exist_ok=True)

    print("# MANTA — corrected architecture")
    print()

    # Build at deploy = 1 (fully deployed) — these are the parts the viewer
    # will animate.
    print("  Building DEPLOYED state for the per-part STLs...")
    parts_deployed = build_full(deploy=1.0)
    export_parts(parts_deployed, parts_dir / "deployed")

    print("  Building STOWED state...")
    parts_stowed = build_full(deploy=0.0)
    export_parts(parts_stowed, parts_dir / "stowed")

    # Sanity bbox
    print()
    deployed_solids = []
    for wp in parts_deployed.values():
        deployed_solids.extend(wp.solids().vals())
    if deployed_solids:
        full_d = cq.Compound.makeCompound(deployed_solids)
        cq.exporters.export(full_d, str(out_dir / "full_deployed.stl"),
                            tolerance=0.001, angularTolerance=0.5)
        bb = full_d.BoundingBox()
        print(f"  Deployed bbox (m): "
              f"x [{bb.xmin:+.3f},{bb.xmax:+.3f}], "
              f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}], "
              f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}]")

    stowed_solids = []
    for wp in parts_stowed.values():
        stowed_solids.extend(wp.solids().vals())
    if stowed_solids:
        full_s = cq.Compound.makeCompound(stowed_solids)
        cq.exporters.export(full_s, str(out_dir / "full_stowed.stl"),
                            tolerance=0.001, angularTolerance=0.5)
        bb = full_s.BoundingBox()
        print(f"  Stowed bbox (m):   "
              f"x [{bb.xmin:+.3f},{bb.xmax:+.3f}], "
              f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}], "
              f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}]")

    # Off-body z-thickness (stowed)
    if stowed_solids:
        bb = full_s.BoundingBox()
        torso_back_z = Anthro().torso_t / 2
        thickness_off_body = bb.zmax - torso_back_z
        print()
        print(f"  Stowed-package thickness off body (z above torso back): "
              f"{thickness_off_body * 1000:.0f} mm")
        print(f"  BRIEF v1 target was 150 mm. New architecture: structure runs along arms/legs,")
        print(f"  not on top of the rig — the off-body thickness is dominated by spine yoke "
              f"({Architecture().spine_yoke_diam * 1000:.0f} mm) plus harness, not a wing-package stack.")


if __name__ == "__main__":
    main()
