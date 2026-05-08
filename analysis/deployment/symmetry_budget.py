"""
Left-right deployment symmetry budget.

Question: does the BRIEF architecture (single CO2 valve sequencing both sides
through a matched-impedance manifold, passive tape-spring rib snap-through)
hold left-right deploy timing variance below 10 ms 3-σ across the operational
envelope?

Approach
--------
Each contributor to the per-side full-deploy time is modeled as an independent
random variable with an explicit distribution. We Monte-Carlo many trials,
take the difference between left and right total deploy times, and report
the 3-σ of |Δt|.

The contributors and their distributions are deliberately *conservative*
first-cut estimates — when laboratory characterization comes in (each from
the bench-test program), tighten them and re-run.

Contributors
~~~~~~~~~~~~

1. **CO2 cartridge fill mass.** Datasheet ±2 % at 20 °C; at 0 °C the
   regulator+orifice flow downstream sees ~10 % less mass-flow rate. Per
   side independently.
2. **Valve actuation latency.** The two sides share *one* solenoid valve,
   so the *bulk* valve-open is common-mode and cancels in Δt. What remains
   is a small differential opening (mechanical asymmetry of the manifold
   ports).
3. **Manifold flow impedance.** Matched-impedance design targets ±2 %
   between sides; field manufacturing variance widens this.
4. **Spar telescope friction (cold/wet).** Stage friction varies with
   temperature, ice/water ingress, and bearing wear. Independent per side.
5. **Tape-spring snap-through dispersion.** Bistable shells have a snap
   energy with shot-to-shot variance; we assume it scales the per-rib
   unfurl time. With 9 ribs/side and independent dispersions, the per-side
   total has its own variance via central limit.

Output
------
- Console: a table of contributors and their 3-σ contribution, plus the
  combined 3-σ vs the 10 ms BRIEF gate.
- out/symmetry_budget.csv: trial-level data (Δt) for plotting.
- out/symmetry_histogram.png: histogram of |Δt|.
- out/sensitivity.csv: 3-σ vs each contributor scaled 0.5×, 1×, 2×.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


N_TRIALS = 50_000
GATE_MS = 10.0


@dataclass(frozen=True)
class Contributor:
    name: str
    sigma_ms: float
    common_mode_fraction: float = 0.0  # 0 = fully independent per side; 1 = cancels in Δt
    description: str = ""


CONTRIBUTORS: tuple[Contributor, ...] = (
    Contributor(
        "CO2 cartridge fill mass / temp",
        sigma_ms=2.5,
        common_mode_fraction=0.0,
        description="Datasheet ±2 % fill mass; cold cartridges flow slower. "
                    "Per-side independent (separate cartridges).",
    ),
    Contributor(
        "Valve port differential",
        sigma_ms=0.8,
        common_mode_fraction=0.0,
        description="Bulk valve open time is common-mode and cancels. "
                    "Residual is mechanical asymmetry of manifold ports.",
    ),
    Contributor(
        "Manifold flow impedance match",
        sigma_ms=1.5,
        common_mode_fraction=0.0,
        description="Matched-impedance target ±2 % between sides; "
                    "field manufacturing variance widens this.",
    ),
    Contributor(
        "Spar telescope friction (cold / wet)",
        sigma_ms=2.0,
        common_mode_fraction=0.0,
        description="Stage friction with water/ice ingress. Independent per side.",
    ),
    Contributor(
        "Tape-spring snap dispersion (per-rib summed)",
        sigma_ms=1.2,
        common_mode_fraction=0.0,
        description="Bistable shell snap energy variance, central-limit-summed "
                    "across 9 ribs/side.",
    ),
)


def _per_side_jitter(c: Contributor, n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Return left and right per-side jitter contributions (ms).

    Common-mode portion is the same value for both sides; differential
    portion is independent.
    """
    cm_sigma = c.sigma_ms * np.sqrt(c.common_mode_fraction)
    diff_sigma = c.sigma_ms * np.sqrt(1 - c.common_mode_fraction)
    cm = rng.normal(0.0, cm_sigma, size=n) if cm_sigma > 0 else np.zeros(n)
    left = cm + (rng.normal(0.0, diff_sigma, size=n) if diff_sigma > 0 else 0.0)
    right = cm + (rng.normal(0.0, diff_sigma, size=n) if diff_sigma > 0 else 0.0)
    return left, right


def run_monte_carlo(contribs: tuple[Contributor, ...] = CONTRIBUTORS,
                    n: int = N_TRIALS, seed: int = 42) -> dict:
    rng = np.random.default_rng(seed)
    left_total = np.zeros(n)
    right_total = np.zeros(n)
    per_contrib_3sigma = []
    for c in contribs:
        left, right = _per_side_jitter(c, n, rng)
        left_total += left
        right_total += right
        dt = left - right
        per_contrib_3sigma.append((c.name, 3 * dt.std()))

    dt_total = left_total - right_total
    abs_dt = np.abs(dt_total)
    return {
        "dt_total_ms": dt_total,
        "abs_dt_ms": abs_dt,
        "three_sigma_ms": float(3 * dt_total.std()),
        "p99_ms": float(np.quantile(abs_dt, 0.99)),
        "p999_ms": float(np.quantile(abs_dt, 0.999)),
        "max_ms": float(abs_dt.max()),
        "per_contrib_3sigma": per_contrib_3sigma,
    }


def sensitivity_sweep() -> list[tuple]:
    """For each contributor, scale its sigma by 0.5×, 1×, 2× and re-run."""
    rows = []
    for i, target in enumerate(CONTRIBUTORS):
        for scale in (0.5, 1.0, 2.0):
            scaled = list(CONTRIBUTORS)
            scaled[i] = Contributor(
                name=target.name,
                sigma_ms=target.sigma_ms * scale,
                common_mode_fraction=target.common_mode_fraction,
                description=target.description,
            )
            r = run_monte_carlo(tuple(scaled), n=20_000, seed=42 + i * 7)
            rows.append((target.name, scale, r["three_sigma_ms"]))
    return rows


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    print("# MANTA deployment symmetry budget — Monte Carlo")
    print()
    res = run_monte_carlo()
    print("## Contributors")
    print()
    print("| Contributor | σ (ms) | common-mode fraction | 3-σ contribution to Δt (ms) |")
    print("|---|---|---|---|")
    for c, (_, three) in zip(CONTRIBUTORS, res["per_contrib_3sigma"]):
        print(f"| {c.name} | {c.sigma_ms:.2f} | {c.common_mode_fraction:.2f} | {three:.2f} |")
    print()
    print(f"## Combined  ({N_TRIALS:,} trials)")
    print()
    print(f"  3-σ |Δt|   :  **{res['three_sigma_ms']:.2f} ms**")
    print(f"  P99  |Δt|  :  {res['p99_ms']:.2f} ms")
    print(f"  P99.9 |Δt| :  {res['p999_ms']:.2f} ms")
    print(f"  max |Δt|   :  {res['max_ms']:.2f} ms")
    print()
    gate_status = "PASS" if res['three_sigma_ms'] <= GATE_MS else "FAIL"
    margin = GATE_MS - res['three_sigma_ms']
    print(f"  vs BRIEF gate ({GATE_MS} ms 3-σ): **{gate_status}** "
          f"(margin = {margin:+.2f} ms)")
    print()

    # Save trial CSV
    with (out_dir / "symmetry_trials.csv").open("w") as f:
        f.write("dt_ms,abs_dt_ms\n")
        # subset for file size
        for v, av in zip(res["dt_total_ms"][:5000], res["abs_dt_ms"][:5000]):
            f.write(f"{v:.4f},{av:.4f}\n")

    # Sensitivity
    print("## Sensitivity sweep — 3-σ of |Δt| as each contributor scales")
    print()
    print("| Contributor | scale | 3-σ |Δt| (ms) | vs nominal |")
    print("|---|---|---|---|")
    sens = sensitivity_sweep()
    nominal = res["three_sigma_ms"]
    with (out_dir / "sensitivity.csv").open("w") as f:
        f.write("contributor,scale,three_sigma_ms\n")
        for name, scale, ts in sens:
            f.write(f"\"{name}\",{scale},{ts:.4f}\n")
            print(f"| {name} | {scale:.1f}× | {ts:.2f} | {ts - nominal:+.2f} |")

    # ---- Closing the budget: alternate architecture proposals ------------
    print()
    print("## Closing the budget — alternate architectures")
    print()
    print("If the nominal architecture cannot close, what alternates do?")
    print()

    # Option A: shared CO2 reservoir → CO2 contributor becomes mostly common-mode
    altA = list(CONTRIBUTORS)
    altA[0] = Contributor(
        "CO2 (shared reservoir, common-mode)",
        sigma_ms=2.5,
        common_mode_fraction=0.85,
    )
    rA = run_monte_carlo(tuple(altA), n=20_000, seed=101)

    # Option B: active per-side flow modulation → all contributors halved
    altB = tuple(
        Contributor(name=c.name, sigma_ms=c.sigma_ms * 0.5,
                    common_mode_fraction=c.common_mode_fraction)
        for c in CONTRIBUTORS
    )
    rB = run_monte_carlo(altB, n=20_000, seed=102)

    # Option C: mechanical-spring primary (CO2 only unlocks) → drop CO2 + valve, manifold; keep friction + snap
    altC = (
        Contributor("Mechanical unlock differential (replaces CO2/valve/manifold)",
                    sigma_ms=0.5, common_mode_fraction=0.0),
        CONTRIBUTORS[3],  # spar friction
        CONTRIBUTORS[4],  # tape-spring snap
    )
    rC = run_monte_carlo(altC, n=20_000, seed=103)

    print("| Option | 3-σ |Δt| (ms) | vs gate ({} ms) | Status |".format(GATE_MS))
    print("|---|---|---|---|")
    for label, r in [
        ("A — shared CO2 reservoir (common-mode 0.85)", rA),
        ("B — active per-side flow modulation (all σ × 0.5)", rB),
        ("C — mechanical-spring primary, CO2 unlock only", rC),
    ]:
        status = "PASS" if r['three_sigma_ms'] <= GATE_MS else "FAIL"
        margin = GATE_MS - r['three_sigma_ms']
        print(f"| {label} | {r['three_sigma_ms']:.2f} | {margin:+.2f} | {status} |")
    print()

    # Histogram plot
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.hist(res["abs_dt_ms"], bins=80, color="tab:blue", edgecolor="white")
        ax.axvline(res['three_sigma_ms'], color="red", linestyle="--",
                   label=f"3-σ = {res['three_sigma_ms']:.2f} ms")
        ax.axvline(GATE_MS, color="black", linestyle="-",
                   label=f"BRIEF gate = {GATE_MS} ms")
        ax.set_xlabel("|Δt|  (ms)")
        ax.set_ylabel("trials")
        ax.set_title(f"MANTA left-right deploy symmetry — {N_TRIALS:,} trials")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / "symmetry_histogram.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
