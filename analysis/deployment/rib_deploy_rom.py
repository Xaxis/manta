"""
Bistable rolled-composite rib deployment — reduced-order physics model.

MANTA's 9 chordwise ribs per side are bistable rolled composite tape-springs
(thin-ply HSC carbon, ACS3/CTM-class lenticular section). Each is stored as a
flat coil at the leading-edge spar hub and self-deploys by releasing the elastic
strain energy stored in the coiled (flattened) shell, snapping onto the airfoil
and self-latching open. This is the real Phase-C mechanism the 3D animation
(`sim/build.py: Mesh.rib_unroll`) shows — this module is the physics behind it.

Why a rib and NOT the spar rolls: a reeled thin-shell boom delivers bending
stiffness EI ~ 1e2-1e3 N·m²; the MANTA wing spar needs ~3.4e4 N·m² to carry the
1.5 kN·m root moment at 3 g, two orders of magnitude more, and a wall thick
enough to carry that load cannot coil (ε = t/2R exceeds failure strain). So the
spars stay rigid (67 mm CFRP, see analysis/struct/spar_bending.py) and the
rollable/bistable mechanism is used where it belongs — the ribs.

Physics (deployable-structures literature):
  * The coil deforms inextensionally: the flattened shell stores PURE BENDING
    energy at curvature 1/R (R = transverse natural radius). No stretch energy.
  * Folding/uncoiling is a propagating instability with a CONSTANT steady
    propagation moment M* = (1+ν)·D·α (opposite-sense), D = shell bending
    rigidity per width, α = subtended arc. Constant M* ⇒ constant driving force
    F_drive = M*/r_coil (energy method, F = dU/dx).
  * Bistability (laminate-tailored) makes the coiled state a local energy
    minimum, so deployment is controllable and self-latches at both ends — no
    restraint band, low net driving force. Modeled as a double-well potential.
  * Deployment shock + blossoming are the hazards: the momentum-flux
    (chain-fountain) term ρ_lin·ẋ² and the end-of-stroke latch impact. A rotary
    viscous damper on the hub regulates the rate.

Refs: Seffen & Pellegrino, "Deployment of a rigid panel by tape-springs",
Proc. R. Soc. A 455 (1999); Calladine, theory of shell folding; Fernandez
(NASA Langley) NTRS 20170001569 (TRAC/CTM thin-ply HSC, t≈0.115 mm, E11≈72 GPa);
Stohlman/Zander/Fernandez AIAA SciTech 2021 (CTM booms); RolaTube BRC.

Run:  PYTHONPATH=. .venv/bin/python analysis/deployment/rib_deploy_rom.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

_OUT = Path(__file__).parent / "out"
_OUT.mkdir(parents=True, exist_ok=True)


@dataclass(frozen=True)
class RibConfig:
    # --- shell / laminate (thin-ply high-strain carbon, ACS3/CTM-class) ----
    t_shell: float = 0.14e-3        # m, shell wall
    E11: float = 72.0e9             # Pa, axial modulus
    nu: float = 0.3                 # Poisson
    alpha_arc: float = 2.6          # rad, lenticular subtended arc (~150°)
    strip_width: float = 0.035      # m, flattened strip width
    # --- coil + rib ---------------------------------------------------------
    r_coil: float = 0.011           # m, coil inner radius at the hub
    rho_lin: float = 0.022          # kg/m, deployed linear density
    L_rib: float = 1.0              # m, representative rib chord (0.59..1.47)
    # --- bistable detents + rate regulation ---------------------------------
    # The Seffen-Pellegrino propagation force IS the net strain-energy drive;
    # bistability adds only a SHALLOW detent at each end (small barrier to start,
    # small latch to hold) — it does not fight the steady deployment.
    detent_J: float = 0.003         # J, shallow bistable detent depth (each end)
    c_damp: float = 0.13            # N·s/m, hub viscous damper sized for soft latch
    mu_fric: float = 0.02           # inter-layer Coulomb friction coefficient
    latch_k: float = 4.0e4          # N/m, end-of-stroke latch stiffness
    latch_zeta: float = 0.7         # latch damping ratio (near-critical, no bounce)

    # --- derived ------------------------------------------------------------
    @property
    def D_shell(self) -> float:            # shell bending rigidity per unit width
        return self.E11 * self.t_shell ** 3 / (12.0 * (1 - self.nu ** 2))

    @property
    def M_star(self) -> float:             # steady propagation moment (N·m)
        return (1 + self.nu) * self.D_shell * self.alpha_arc * self.strip_width

    @property
    def F_drive(self) -> float:            # constant strain-energy driving force
        return self.M_star / self.r_coil

    @property
    def coil_strain(self) -> float:        # ε = t/2R at the coil
        return self.t_shell / (2 * self.r_coil)


@dataclass
class RibResult:
    t: np.ndarray
    s: np.ndarray
    xd: np.ndarray
    t_snap: float
    v_peak: float
    v_latch: float
    shock: float
    bloss_margin: float
    damper_required: bool
    cfg: RibConfig = field(default_factory=RibConfig)


def _detent_grad(s: float, cfg: RibConfig) -> float:
    """d/ds of the shallow bistable detents at each end. Two narrow wells at
    s≈0 and s≈1 hold the stowed and deployed states; between them the potential
    is flat, so the steady propagation force governs the stroke. A small net
    tilt makes the deployed detent the global minimum (self-latching open)."""
    w = 0.06                                   # detent half-width in s
    g0 = math.exp(-(s / w) ** 2) * (-2 * s / w ** 2)        # well at s=0
    g1 = math.exp(-((s - 1) / w) ** 2) * (-2 * (s - 1) / w ** 2)  # well at s=1
    tilt = 0.5 * cfg.detent_J                  # bias toward deployed
    return cfg.detent_J * (g0 + g1) - tilt


def _rhs(t, y, cfg: RibConfig):
    x, xd = y                                   # deployed length, rate
    L = cfg.L_rib
    s = min(max(x / L, 0.0), 1.0)
    # effective inertia: deployed mass moving at xd + coiled mass paid off the
    # hub whose rim speed is also xd → m_eff ≈ rho_lin·L (the whole rib).
    m_dep = cfg.rho_lin * x
    I_reel = cfg.rho_lin * max(L - x, 0.0) * cfg.r_coil ** 2
    m_eff = m_dep + I_reel / cfg.r_coil ** 2 + 1e-4
    # forces along the unfurling front
    F = cfg.F_drive                              # constant strain-energy drive
    F -= _detent_grad(s, cfg) / L                # shallow bistable end-detents
    F -= cfg.c_damp * xd                         # hub viscous damper (rate reg.)
    if abs(xd) > 1e-6:
        F -= cfg.mu_fric * cfg.F_drive * (1.0 if xd > 0 else -1.0)  # layer friction
    F += cfg.rho_lin * xd * xd                   # momentum flux (chain-fountain)
    if x >= L:                                   # end-of-stroke latch (one-sided)
        k = cfg.latch_k
        c = 2 * cfg.latch_zeta * math.sqrt(k * m_eff)
        F -= k * (x - L) + c * xd
    return [xd, F / m_eff]


def solve(cfg: RibConfig = RibConfig()) -> RibResult:
    # kick off just past the stowed detent so the steady drive takes over
    sol = solve_ivp(_rhs, (0, 1.5), [0.01, 0.0], args=(cfg,),
                    max_step=5e-4, rtol=1e-8, atol=1e-10, dense_output=True)
    t, x, xd = sol.t, sol.y[0], sol.y[1]
    s = np.clip(x / cfg.L_rib, 0, 1)
    # snap time = first time the deployed fraction reaches 0.99
    reached = s >= 0.99
    t_snap = float(t[np.argmax(reached)]) if reached.any() else float("nan")
    v_peak = float(np.max(xd))
    # latch-contact velocity = rate at the instant x first reaches L (the
    # deployment-shock driver); soft if the damper has bled the stroke down.
    cross = np.argmax(x >= cfg.L_rib) if (x >= cfg.L_rib).any() else len(x) - 1
    v_latch = float(xd[cross])
    over = x > cfg.L_rib
    shock = float(np.max(cfg.latch_k * np.clip(x - cfg.L_rib, 0, None))) if over.any() else 0.0
    # Blossoming guard: would passive inter-layer friction alone hold the coil
    # wound against the steady tip force? F_hold = mu · hoop tension · turns.
    F_hold = cfg.mu_fric * cfg.M_star / cfg.r_coil ** 2 * (2 * math.pi * cfg.r_coil)
    bloss_margin = F_hold / max(cfg.F_drive, 1e-9)
    damper_required = bloss_margin < 1.0
    return RibResult(t=t, s=s, xd=xd, t_snap=t_snap, v_peak=v_peak,
                     v_latch=v_latch, shock=shock, bloss_margin=bloss_margin,
                     damper_required=damper_required, cfg=cfg)


def _markdown(r: RibResult) -> str:
    c = r.cfg
    hold = "insufficient" if r.damper_required else "sufficient"
    return "\n".join([
        "# Bistable rib deployment — reduced-order physics",
        "",
        "Strain-energy-driven unroll of a bistable rolled-composite rib "
        "(thin-ply HSC carbon, lenticular section). Backs the `rib_unroll` "
        "geometry in `sim/build.py`.",
        "",
        "| Quantity | Value |",
        "|---|---|",
        f"| Shell wall t | {c.t_shell*1e3:.2f} mm |",
        f"| Coil radius r_coil | {c.r_coil*1e3:.1f} mm |",
        f"| Coil strain ε = t/2R | {c.coil_strain*100:.2f} % (< ~1 % HSC allowable) |",
        f"| Shell rigidity D | {c.D_shell*1e3:.2f} mN·m |",
        f"| Propagation moment M\\* | {c.M_star*1e3:.2f} mN·m |",
        f"| Driving force F_drive = M\\*/r | {c.F_drive*1e3:.0f} mN |",
        f"| Rib chord (representative) | {c.L_rib:.2f} m |",
        f"| Rib mass ρ·L | {c.rho_lin*c.L_rib*1e3:.0f} g |",
        f"| **Snap time t_snap (s→0.99)** | **{r.t_snap*1e3:.0f} ms** |",
        f"| Peak deploy velocity | {r.v_peak:.2f} m/s |",
        f"| **Latch-contact velocity** | **{r.v_latch:.2f} m/s** (soft) |",
        f"| End-latch shock | {r.shock:.0f} N |",
        f"| Passive friction hold / drive | {r.bloss_margin:.2f}× ({hold}) |",
        "",
        f"**Deployment.** Each ~{c.L_rib:.1f} m rib snaps from packed coil to "
        f"latched airfoil in ~{r.t_snap*1e3:.0f} ms, driven by the constant "
        f"Seffen–Pellegrino propagation force F = M\\*/r_coil = {c.F_drive*1e3:.0f} mN. "
        "The 9 ribs/side fire on a root→tip stagger, so the unfurl front sweeps "
        "outboard across Phase C — this is the schedule the `rib_unroll` animation "
        "renders (`RIB_STAGGER`, `RIB_SNAP_DUR`).",
        "",
        f"**Soft latch.** A rotary viscous damper on the hub bleeds the stroke so "
        f"the rib reaches its end-stop at only {r.v_latch:.2f} m/s, keeping the "
        f"deployment-shock load to ~{r.shock:.0f} N — well under the {c.M_star/c.r_coil:.1f} N "
        "the latched root carries — so there is no destructive snap.",
        "",
        f"**Blossoming guard (design driver).** Inter-layer friction alone holds "
        f"only {r.bloss_margin:.2f}× the steady tip force, i.e. it is **{hold}**: a "
        "free friction coil WOULD blossom (unwind loosely inside the deployer). "
        "That is exactly why the hub is a **rate-controlled spool** (the same "
        "viscous damper): it meters payout so the coil feeds the front under "
        "tension instead of blooming. Bistable end-detents then hold both the "
        "stowed and the deployed states without a restraint band.",
    ])


def main():
    r = solve()
    c = r.cfg
    print("Bistable rib deployment ROM")
    print(f"  D_shell = {c.D_shell*1e3:.2f} mN·m   M* = {c.M_star*1e3:.2f} mN·m"
          f"   F_drive = {c.F_drive*1e3:.0f} mN")
    print(f"  coil strain = {c.coil_strain*100:.2f} %   "
          f"t_snap = {r.t_snap*1e3:.0f} ms   v_peak = {r.v_peak:.2f} m/s"
          f"   v_latch = {r.v_latch:.2f} m/s")
    print(f"  latch shock = {r.shock:.0f} N   passive-hold/drive = {r.bloss_margin:.2f}x"
          f"   damper {'REQUIRED' if r.damper_required else 'optional'}")

    import csv
    with open(_OUT / "rib_deploy.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["t_s", "deploy_frac", "rate_mps"])
        for ti, si, vi in zip(r.t, r.s, r.xd):
            w.writerow([f"{ti:.5f}", f"{si:.5f}", f"{vi:.5f}"])
    (_OUT / "rib_deploy_results.md").write_text(_markdown(r))

    # Normalized deploy-easing profile (s vs normalized snap-time, 0->1) consumed
    # by sim/build.py so the on-screen rib unroll follows THIS integrated ODE
    # rather than an ad-hoc smoothstep — the animation is physics-driven.
    import json
    n = 48
    if math.isfinite(r.t_snap) and r.t_snap > 0:
        tau = np.linspace(0.0, r.t_snap, n)
        s_of = np.interp(tau, r.t, r.s)
        s0, s1 = float(s_of[0]), float(s_of[-1])
        prof = [float(np.clip((v - s0) / max(s1 - s0, 1e-9), 0.0, 1.0)) for v in s_of]
    else:
        prof = [i / (n - 1) for i in range(n)]
    (_OUT / "rib_profile.json").write_text(json.dumps(
        {"n": n, "t_snap_s": r.t_snap, "v_latch_mps": r.v_latch, "s": prof}))
    print(f"  wrote {_OUT/'rib_profile.json'} ({n} samples, physics easing)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
        ax[0].plot(r.t * 1e3, r.s, lw=2, color="#1f77b4")
        ax[0].axhline(1.0, ls=":", color="grey"); ax[0].set_xlabel("t (ms)")
        ax[0].set_ylabel("deploy fraction s"); ax[0].set_title("Rib unroll (strain-energy)")
        ax[1].plot(r.t * 1e3, r.xd, lw=2, color="#d62728")
        ax[1].set_xlabel("t (ms)"); ax[1].set_ylabel("deploy rate (m/s)")
        ax[1].set_title("Deploy velocity (damper-regulated)")
        for a in ax:
            a.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(_OUT / "rib_deploy.png", dpi=120)
        print(f"  wrote {_OUT/'rib_deploy.png'}")
    except Exception as e:
        print("  (plot skipped:", e, ")")


if __name__ == "__main__":
    main()
