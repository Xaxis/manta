#!/usr/bin/env bash
# Sync PNG plot artifacts from the analysis tree into the site's public dir.
# Run from the project root or from within site/.

set -euo pipefail
cd "$(dirname "$0")/.."   # site/

repo_root="$(cd .. && pwd)"
target="public/img"
mkdir -p "$target"

declare -a mappings=(
  "$repo_root/analysis/aero/weissinger/out/span_loading.png|span_loading.png"
  "$repo_root/analysis/aero/weissinger/out/induced_polar.png|induced_polar.png"
  "$repo_root/analysis/aero/lift_drag/out/glide_polar.png|glide_polar.png"
  "$repo_root/analysis/aero/trim/out/washout_plot.png|washout_plot.png"
  "$repo_root/analysis/struct/out/bending_curves.png|bending_curves.png"
  "$repo_root/analysis/struct/out/budget_pie_brief.png|budget_pie_brief.png"
  "$repo_root/analysis/struct/out/budget_pie_sized.png|budget_pie_sized.png"
  "$repo_root/analysis/deployment/out/symmetry_histogram.png|symmetry_histogram.png"
  "$repo_root/analysis/deployment/out/drogue_descent.png|drogue_descent.png"
  "$repo_root/analysis/deployment/out/rib_deploy.png|rib_deploy.png"
  "$repo_root/analysis/deployment/out/membrane_sag.png|membrane_sag.png"
  "$repo_root/fcs/sim/out/scenario_pilot_overcmd.png|scenario_pilot_overcmd.png"
  "$repo_root/fcs/sim/out/scenario_cg_shift_50mm.png|scenario_cg_shift_50mm.png"
  "$repo_root/sim/out/flight_dynamics.png|flight_dynamics.png"
  "$repo_root/sim/out/hero_deployed.png|hero_deployed.png"
  "$repo_root/sim/out/hero_top.png|hero_top.png"
  "$repo_root/sim/out/hero_mid_deploy.png|hero_mid_deploy.png"
  "$repo_root/sim/out/hero_stowed.png|hero_stowed.png"
  "$repo_root/sim/out/hero_control.png|hero_control.png"
  "$repo_root/sim/out/hero_flow.png|hero_flow.png"
  "$repo_root/sim/out/hero_coils_mid.png|hero_coils.png"
)

for m in "${mappings[@]}"; do
  src="${m%%|*}"
  dst="${m##*|}"
  if [[ -f "$src" ]]; then
    cp "$src" "$target/$dst"
    echo "  ✓ $dst"
  else
    echo "  ✗ MISSING: $src" >&2
  fi
done

echo "Done. Assets in $target/"
