"""
Lifting-line for swept, tapered, twisted wings (Prandtl-style placement).

Method
------
Single-chord-row horseshoe lattice with the bound vortex along the panel c/4
line and the control point at panel midpan ON the c/4 line. Induced velocity
at each control point comes from all horseshoes' trailing legs (the bound
segment of the same panel passes through its control point and contributes
zero in the limit). Section lift coupling closes the system:

    Cl_section_i  =  a0 · (α_geom_i  −  α_0  +  α_ind_i)

with α_ind from the trailing wake and Cl_section from Kutta-Joukowski
(2 Γ_i / V∞ c_i). In the 2D limit this reproduces a0 (e.g. 2π/rad), and in
the AR→∞ limit it recovers Helmbold:

    CL_α  =  a0 / (1 + a0 / (π·AR·e))

For swept wings the bound segments follow the c/4 sweep line, which is what
distinguishes this from pure Prandtl LLT (where bound vortices are along the
y-axis). Result: ~1 % span-loading accuracy vs. a higher-fidelity multi-row
VLM, ~1° in induced angle (Bertin & Smith, *Aerodynamics for Engineers*, §7;
Anderson, *Fundamentals of Aerodynamics*, §5.3 + §5.6).

What this code does
-------------------
- Discretizes both wing halves into N panels each with cosine spacing.
- Builds the (2N × 2N) influence matrix using Biot-Savart for the bound
  segment + two semi-infinite trailing legs aligned with +x.
- Couples to a 2D section lift-curve slope (5.7 /rad default; passed in)
  via Cl_section = a₀·(α_geom + α_induced − α₀); this lets the solver
  recover the right CL_α slope rather than the bare 2π/rad inviscid value.
- Solves for circulation Γ(y), returns span loading, induced drag, total CL.

Limitations (and what this code is *not*)
-----------------------------------------
- One chordwise panel: cannot capture chordwise loading or compressibility.
- Linear: stall is *not* modeled here. Sectional Cl is unbounded; check
  alpha values against `polar_analytic.alpha_stall_deg` outside this module.
- Wake is rigid in +x: no wake roll-up.
- For load-bearing safety claims, AVL or VLM with multi-row chord paneling
  remains required. This code is the first-cut analysis with traceable
  algebra; AVL is the verification that follows.

Validation
----------
`tests/test_weissinger.py` checks:
  - Rectangular AR=8 wing, untwisted: span loading deviation from elliptical
    less than 1.5 %; CL_α / a0 ratio matches Helmbold within 2 %.
  - Symmetric loading on a symmetric input.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

X_HAT = np.array([1.0, 0.0, 0.0])


# -------------------------------------------------------------------------
# Biot-Savart primitives
# -------------------------------------------------------------------------

def _bs_segment(P: np.ndarray, A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Velocity at P from a unit-circulation finite vortex segment A→B."""
    r1 = P - A
    r2 = P - B
    r1xr2 = np.cross(r1, r2)
    denom = np.einsum("...i,...i->...", r1xr2, r1xr2)
    # Avoid division by zero on collinear points; those contributions are zero.
    safe = denom > 1e-14
    denom = np.where(safe, denom, 1.0)
    r1n = np.linalg.norm(r1, axis=-1, keepdims=True)
    r2n = np.linalg.norm(r2, axis=-1, keepdims=True)
    inner = np.einsum("...i,...i->...", B - A, r1 / r1n - r2 / r2n)
    v = r1xr2 * (inner[..., None] / (4.0 * math.pi * denom[..., None]))
    return np.where(safe[..., None], v, 0.0)


def _bs_semi_inf(P: np.ndarray, A: np.ndarray, u_hat: np.ndarray, sign: float) -> np.ndarray:
    """Velocity at P from a unit-circulation semi-infinite vortex starting at A,
    extending in direction u_hat, with circulation = sign * 1.

    Biot-Savart for a filament going in direction u_hat with vorticity in that
    direction: dl × r → u_hat × (P − element). The differential point on the
    filament is A + s·u_hat; cross product with (P − that) reduces to u_hat × r1
    (the s·u_hat term is parallel to u_hat and drops out).
    """
    r1 = P - A
    uxr1 = np.cross(np.broadcast_to(u_hat, r1.shape), r1)
    denom = np.einsum("...i,...i->...", uxr1, uxr1)
    safe = denom > 1e-14
    denom = np.where(safe, denom, 1.0)
    r1n = np.linalg.norm(r1, axis=-1)
    factor = sign * (1.0 + np.einsum("...i,...i->...", r1, np.broadcast_to(u_hat, r1.shape)) / r1n) / (4.0 * math.pi * denom)
    v = uxr1 * factor[..., None]
    return np.where(safe[..., None], v, 0.0)


def _horseshoe_velocity(P: np.ndarray, A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Velocity at P from unit-circulation horseshoe (bound A→B, trailing legs
    from A and B to +∞ in +x direction).

    All inputs may be broadcast: typical use is P shape (M, 1, 3), A and B
    shape (1, N, 3) so output has shape (M, N, 3).
    """
    v_bound = _bs_segment(P, A, B)
    v_trail_B = _bs_semi_inf(P, B, X_HAT, sign=+1.0)
    v_trail_A = _bs_semi_inf(P, A, X_HAT, sign=-1.0)
    return v_bound + v_trail_B + v_trail_A


# -------------------------------------------------------------------------
# Wing model
# -------------------------------------------------------------------------

@dataclass
class WingModel:
    """Geometry callbacks supplied by the planform / configuration module.

    All callbacks accept y in [−b/2, b/2] (meters) and return scalar values.

    chord_at(y)        : local chord length, m
    x_le_at(y)         : leading-edge x-coordinate, m (0 at root LE)
    twist_deg_at(y)    : geometric twist, degrees, signed (washout positive
                         means tip is at lower α than root → returned negative)
    section_alpha_0_deg: 2D zero-lift angle, degrees (constant across span here;
                         can be made y-dependent by extending this callback API)
    section_a0_per_rad : 2D lift-curve slope, /rad
    """
    span: float
    chord_at: Callable[[float], float]
    x_le_at: Callable[[float], float]
    twist_deg_at: Callable[[float], float]
    section_alpha_0_deg: float = -1.0
    section_a0_per_rad: float = 5.7


@dataclass
class Result:
    """Output of a single Weissinger solve at a given alpha."""
    alpha_deg: float
    y: np.ndarray            # control-point y stations, m
    chord: np.ndarray        # local chord at y, m
    Gamma: np.ndarray        # circulation per panel, m^2/s (V_inf-normalized: actually Γ/V∞)
    cl_section: np.ndarray   # local section Cl
    span_load: np.ndarray    # cl·c, normalized loading m
    alpha_induced_deg: np.ndarray
    CL: float
    CDi: float
    e: float                 # span efficiency
    # Aerodynamic-center reference
    cm_about_apex: float     # pitching-moment coefficient about wing apex (root LE),
                             # nondim by S·MAC; useful for neutral-point fits


# -------------------------------------------------------------------------
# Solver
# -------------------------------------------------------------------------

def solve(
    wing: WingModel,
    alpha_deg: float,
    n_panels_per_side: int = 40,
    S_ref: float | None = None,
    mac_ref: float | None = None,
) -> Result:
    """Solve the Weissinger lifting-line for `wing` at angle of attack `alpha_deg`.

    Parameters
    ----------
    wing : WingModel
    alpha_deg : float
        Freestream angle of attack relative to the *root chord*.
    n_panels_per_side : int
        Cosine-spaced panels on each half-wing.
    S_ref, mac_ref : optional reference area and chord for nondimensionalization.
        Defaults to integrated chord*span and area-weighted chord (MAC).
    """
    b = wing.span
    half_b = b / 2.0
    N = n_panels_per_side
    Ntot = 2 * N

    # Cosine spacing on full span: y_edge_k = (b/2)·cos(theta_k) with theta from 0 to π,
    # giving y from +b/2 (k=0) down to −b/2 (k=Ntot). Reverse to ascending y.
    theta = np.linspace(math.pi, 0.0, Ntot + 1)
    y_edges = half_b * np.cos(theta)            # ascending -b/2 → +b/2
    y_mid = 0.5 * (y_edges[:-1] + y_edges[1:])
    dy = np.diff(y_edges)

    # Panel endpoints at c/4 line (bound vortex placement)
    chord_e = np.array([wing.chord_at(y) for y in y_edges])
    x_le_e = np.array([wing.x_le_at(y) for y in y_edges])
    A_pts = np.column_stack([x_le_e[:-1] + 0.25 * chord_e[:-1], y_edges[:-1], np.zeros(Ntot)])
    B_pts = np.column_stack([x_le_e[1:]  + 0.25 * chord_e[1:],  y_edges[1:],  np.zeros(Ntot)])

    # Control points at c/4 line of midspan station — co-located with the
    # bound vortex line so the panel's own bound segment induces zero
    # velocity at its CP and the section coupling stays uncontaminated.
    chord_m = np.array([wing.chord_at(y) for y in y_mid])
    x_le_m = np.array([wing.x_le_at(y) for y in y_mid])
    P_pts = np.column_stack([x_le_m + 0.25 * chord_m, y_mid, np.zeros(Ntot)])

    # Influence matrix W[i, j] = z-component of induced velocity at P_i due to
    # unit-circulation horseshoe j. We're inside the Z=0 plane so the normal
    # direction at the control point is the local airfoil-section normal,
    # which to leading order in small angles equals +ẑ.
    P_b = P_pts[:, None, :]
    A_b = A_pts[None, :, :]
    B_b = B_pts[None, :, :]
    V = _horseshoe_velocity(P_b, A_b, B_b)   # shape (Ntot, Ntot, 3)
    Wz = V[:, :, 2]                           # (Ntot, Ntot)

    # Section lift-curve coupling: enforce
    #   2D thin-airfoil result Cl = a0·(α_geom − α_0 + α_ind)
    # at each control point. For a Weissinger horseshoe with Γ_j and chord c_i,
    # the *local* section Cl is 2·Γ_i/(V∞·c_i) (Kutta-Joukowski + thin airfoil).
    # Setting that equal to a0·(α_eff − α_0):
    #
    #   2·Γ_i / (V∞ c_i) = a0·( (α + twist_i − α_0) + Σ_j (Wz_ij·Γ_j)/V∞ )
    #
    # Move Γ terms to LHS:
    #   (2/(c_i)) Γ_i − a0·Σ_j Wz_ij Γ_j = a0·V∞·(α + twist_i − α_0)
    #
    # which is a linear system in Γ_i / V∞.
    a0 = wing.section_a0_per_rad
    alpha_0 = math.radians(wing.section_alpha_0_deg)
    twist_rad = np.array([math.radians(wing.twist_deg_at(y)) for y in y_mid])
    alpha = math.radians(alpha_deg)

    # Build coefficient matrix
    LHS = np.diag(2.0 / chord_m) - a0 * Wz
    RHS = a0 * (alpha + twist_rad - alpha_0)
    Gamma_norm = np.linalg.solve(LHS, RHS)   # = Γ / V∞

    # Section Cl from local circulation
    cl_sec = 2.0 * Gamma_norm / chord_m

    # Induced angle at each control point. Convention: α_ind = w_induced / V∞,
    # so a downwash (w < 0) gives α_ind < 0, reducing the effective α the
    # section sees. Wz·Γ̂ is exactly the induced w/V∞ from the trailing wake.
    alpha_ind = Wz @ Gamma_norm

    # Total lift coefficient by integration: CL = (2/(V∞ S)) ∫ Γ dy = (2/S) Σ Γ̂ dy
    if S_ref is None:
        # Compute S from the wing geometry
        S_ref = float(np.sum(chord_m * dy))
    CL = float(2.0 * np.sum(Gamma_norm * dy) / S_ref)

    # Induced drag: each section's lift is tilted aft by |α_ind|, so the
    # streamwise contribution per unit span is c·cl·|α_ind| = c·cl·(−α_ind)
    # (since α_ind is negative for downwash). Trefftz-plane integral:
    #     CDi = − (1/S) ∫ cl·c·α_ind dy  =  −(2/S) ∫ Γ̂·α_ind dy
    CDi = float(-2.0 * np.sum(Gamma_norm * alpha_ind * dy) / S_ref)

    # Span efficiency: CDi = CL² / (π AR e)
    AR = b * b / S_ref
    e = float(CL * CL / (math.pi * AR * CDi)) if CDi > 1e-12 else float("inf")

    # Pitching moment about the wing apex (root LE), needed for neutral-point fits.
    # Each panel contributes: dCm = -(x_c4 - x_apex)/MAC_ref * cl·c·dy / S
    # where x_c4 = x_le_m + 0.25·chord_m. Contribution from sectional Cm0
    # (camber) is added externally if you have a 2D Cm polar; left out here so
    # this function reflects pure planform geometry.
    if mac_ref is None:
        # Geometric MAC: ∫c² dy / ∫c dy
        mac_ref = float(np.sum(chord_m * chord_m * dy) / np.sum(chord_m * dy))
    x_c4 = x_le_m + 0.25 * chord_m
    cm_apex = float(-np.sum((x_c4) * cl_sec * chord_m * dy) / (S_ref * mac_ref))

    return Result(
        alpha_deg=alpha_deg,
        y=y_mid,
        chord=chord_m,
        Gamma=Gamma_norm,
        cl_section=cl_sec,
        span_load=cl_sec * chord_m,
        alpha_induced_deg=np.degrees(alpha_ind),
        CL=CL,
        CDi=CDi,
        e=e,
        cm_about_apex=cm_apex,
    )


def alpha_sweep(
    wing: WingModel,
    alphas_deg,
    n_panels_per_side: int = 40,
    S_ref: float | None = None,
    mac_ref: float | None = None,
):
    """Run the solver across a list of alphas. Returns list of Result."""
    return [solve(wing, a, n_panels_per_side, S_ref, mac_ref) for a in alphas_deg]


def neutral_point(
    wing: WingModel,
    alphas_deg=(0.0, 4.0),
    n_panels_per_side: int = 40,
    S_ref: float | None = None,
    mac_ref: float | None = None,
) -> tuple[float, float]:
    """Estimate the wing aerodynamic center / neutral point.

    Method: linear fit of cm_about_apex(α) vs CL(α) at two alphas. The
    neutral-point x-station satisfies dCm/dCL = 0 about that point, so:
        x_NP = -dCm_apex/dCL · MAC_ref
    measured aft of the wing apex (root LE).

    Returns (x_NP_aft_of_apex_meters, mac_ref_used_meters).
    """
    rs = alpha_sweep(wing, list(alphas_deg), n_panels_per_side, S_ref, mac_ref)
    cl0, cl1 = rs[0].CL, rs[1].CL
    cm0, cm1 = rs[0].cm_about_apex, rs[1].cm_about_apex
    if abs(cl1 - cl0) < 1e-9:
        raise ValueError("CL did not change between sweep alphas; pick distinct values.")
    dcm_dcl = (cm1 - cm0) / (cl1 - cl0)
    if mac_ref is None:
        # Re-derive MAC from chords (consistent with solve)
        b = wing.span
        N = n_panels_per_side
        theta = np.linspace(math.pi, 0.0, 2 * N + 1)
        y_edges = (b / 2) * np.cos(theta)
        y_mid = 0.5 * (y_edges[:-1] + y_edges[1:])
        dy = np.diff(y_edges)
        chord_m = np.array([wing.chord_at(y) for y in y_mid])
        mac_ref = float(np.sum(chord_m * chord_m * dy) / np.sum(chord_m * dy))
    x_np = -dcm_dcl * mac_ref
    return x_np, mac_ref
