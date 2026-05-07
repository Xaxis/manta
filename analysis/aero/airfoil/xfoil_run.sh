#!/usr/bin/env bash
# XFOIL polar generator for MANTA candidate airfoils.
#
# Usage:    ./xfoil_run.sh
# Requires: xfoil on PATH, candidate airfoil .dat files in airfoils/ subdir
#
# Output:   polars/<airfoil>_Re<RE>.csv with columns alpha,Cl,Cd,Cdp,Cm,Top_xtr,Bot_xtr
#
# The Re sweep covers the operational range (1×10⁶ to 2×10⁶ root, 0.7×10⁶ tip)
# with one extra point at 5×10⁵ to characterize the low-Re tip near stall.

set -euo pipefail

cd "$(dirname "$0")"
mkdir -p polars airfoils

if ! command -v xfoil >/dev/null 2>&1; then
    echo "ERROR: xfoil not on PATH." >&2
    echo "Install: see http://web.mit.edu/drela/Public/web/xfoil/ or 'brew install xfoil-tap/xfoil/xfoil'" >&2
    exit 1
fi

# Candidate airfoils — drop matching .dat files into airfoils/ before running.
# Sources:
#   MH 78:  https://www.mh-aerotools.de/airfoils/  (download .dat coordinate file)
#   MH 60:  https://m-selig.ae.illinois.edu/ads/coord_database.html  or Hepperle
AIRFOILS=(MH78 MH60)
RE_VALUES=(500000 700000 1000000 1500000 2000000)

ALPHA_START=-6
ALPHA_END=18
ALPHA_STEP=0.5
NCRIT=9                # transition criterion (Drela default; e^N method)
ITERS=200

for af in "${AIRFOILS[@]}"; do
    if [[ ! -f "airfoils/${af}.dat" ]]; then
        echo "  airfoils/${af}.dat MISSING — skipping ${af}." >&2
        echo "  Download from mh-aerotools.de or UIUC database, save as airfoils/${af}.dat" >&2
        continue
    fi
    for re in "${RE_VALUES[@]}"; do
        out="polars/${af}_Re${re}.dat"
        echo "  → ${af} at Re=${re}"
        xfoil <<EOF >/dev/null
PLOP
G F

LOAD airfoils/${af}.dat
${af}
PANE
OPER
VISC ${re}
ITER ${ITERS}
VPAR
N ${NCRIT}

PACC
${out}

ASEQ ${ALPHA_START} ${ALPHA_END} ${ALPHA_STEP}
PACC

QUIT
EOF
        # Convert XFOIL .dat to CSV for downstream tooling
        if [[ -f "${out}" ]]; then
            python3 - "${out}" "polars/${af}_Re${re}.csv" <<'PYEOF'
import sys
in_path, out_path = sys.argv[1], sys.argv[2]
with open(in_path) as f:
    lines = [l.rstrip() for l in f if l.strip()]
# XFOIL header is ~12 lines; data table starts after the dashed separator.
data_start = 0
for i, l in enumerate(lines):
    if l.lstrip().startswith("---"):
        data_start = i + 1
        break
header = "alpha,Cl,Cd,Cdp,Cm,Top_xtr,Bot_xtr"
with open(out_path, "w") as f:
    f.write(header + "\n")
    for l in lines[data_start:]:
        cols = l.split()
        if len(cols) < 7: continue
        f.write(",".join(cols[:7]) + "\n")
print(f"  wrote {out_path}")
PYEOF
        fi
    done
done

echo "Done. Polars in polars/."
