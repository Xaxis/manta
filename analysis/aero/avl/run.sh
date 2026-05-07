#!/usr/bin/env bash
# AVL driver for MANTA wing.
# Runs trim + stability + alpha sweep, writes outputs to out/.
#
# Requires:
#   - avl on PATH (Drela/Youngren AVL 3.x)
#   - airfoils/MH78.dat present (download from mh-aerotools.de)

set -euo pipefail
cd "$(dirname "$0")"

if ! command -v avl >/dev/null 2>&1; then
    echo "ERROR: avl not on PATH." >&2
    echo "  See https://web.mit.edu/drela/Public/web/avl/ to obtain AVL." >&2
    exit 1
fi

if [[ ! -f airfoils/MH78.dat ]]; then
    echo "WARNING: airfoils/MH78.dat missing; AVL will fall back to flat-plate sections." >&2
    echo "  Download MH78 .dat from https://www.mh-aerotools.de/airfoils/ for accurate results." >&2
fi

mkdir -p out

# AVL accepts a sequence of single-character commands. We script:
#   oper         — open the operating-point menu
#   m / mn 0     — set Mach 0
#   c1 / pm 0    — constrain CL via Cm = 0 trim (tailless trim search)
#   a / a 0..10  — sweep alpha
#   x            — solve
#   ft / fn      — write totals + run case
#   st           — stability derivatives
#   sb           — body-axis stability derivatives
#   w            — write Treffts plane (span loading)
#
# We loop over alpha values and dump per-alpha output to out/.

ALPHAS=(-2 0 2 4 6 8 10)

for ALPHA in "${ALPHAS[@]}"; do
    echo "→ alpha = ${ALPHA}°"
    avl manta.avl <<EOF >/dev/null
oper
a a ${ALPHA}
x
ft
out/totals_a${ALPHA}.txt
st
out/stab_a${ALPHA}.txt

quit
EOF
done

# Trimmed case: solve for the alpha that gives Cm = 0 (tailless trim)
echo "→ trim solve (Cm = 0)"
avl manta.avl <<EOF >/dev/null
oper
c1
pm 0
x
ft
out/totals_trim.txt
st
out/stab_trim.txt
w
out/treffts_trim.txt

quit
EOF

# Stability with mass case (eigenvalue analysis)
if [[ -f manta.mass ]]; then
    echo "→ stability with mass case"
    avl manta.avl <<EOF >/dev/null
mass manta.mass
mset 1
oper
c1
pm 0
x
mode
N
W
out/modes_trim.txt

quit
EOF
fi

echo "Done. Outputs in out/."
