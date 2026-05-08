# MANTA — top-level driver for analyses and CAD generation.
# Targets are deliberately simple wrappers so anyone can reproduce a result.

PYTHON := python3
VENV   := .venv
ACT    := . $(VENV)/bin/activate &&

.PHONY: help venv install aero aero-weissinger aero-trim aero-ld struct struct-bending struct-budget cad cad-wing cad-spars test clean

help:
	@echo "MANTA make targets:"
	@echo "  make venv             create .venv and install requirements (no AVL/XFOIL/FreeCAD)"
	@echo "  make install          alias for venv"
	@echo ""
	@echo "  make aero             run the full aero analysis pipeline (geometry → weissinger → trim → L/D)"
	@echo "  make aero-weissinger  Weissinger lifting-line + span loading"
	@echo "  make aero-trim        tailless trim + washout iteration"
	@echo "  make aero-ld          glide polar + L/D vs V"
	@echo ""
	@echo "  make struct           run structural pipeline (spar bending + mass budget)"
	@echo "  make struct-bending   spar bending + sizing recommendation"
	@echo "  make struct-budget    component mass roll-up + sensitivity sweep"
	@echo ""
	@echo "  make cad              regenerate all CAD artifacts (STEP + STL)"
	@echo "  make cad-wing         regenerate wing OML only"
	@echo "  make cad-spars        regenerate spar set (BRIEF + sized configs)"
	@echo ""
	@echo "  make test             pytest validation across analysis/"
	@echo "  make avl              optional: run AVL on the deck (requires avl on PATH)"
	@echo "  make xfoil            optional: run XFOIL polars (requires xfoil on PATH)"

venv install:
	$(PYTHON) -m venv $(VENV)
	$(ACT) pip install --upgrade pip
	$(ACT) pip install -r requirements.txt

aero: aero-weissinger aero-trim aero-ld

aero-weissinger:
	$(ACT) python analysis/aero/weissinger/run.py

aero-trim:
	$(ACT) python analysis/aero/trim/run.py

aero-ld:
	$(ACT) python analysis/aero/lift_drag/glide_polar.py

struct: struct-bending struct-budget

struct-bending:
	$(ACT) python analysis/struct/spar_bending.py

struct-budget:
	$(ACT) python analysis/struct/mass_budget.py

cad: cad-wing cad-spars

cad-wing:
	$(ACT) python cad/wing/build.py

cad-spars:
	$(ACT) python cad/spars/build.py

test:
	$(ACT) python -m pytest analysis/ -v

avl:
	@command -v avl >/dev/null 2>&1 || { echo "AVL not on PATH. Install it (athena vortex lattice) and re-run."; exit 1; }
	cd analysis/aero/avl && ./run.sh

xfoil:
	@command -v xfoil >/dev/null 2>&1 || { echo "XFOIL not on PATH. Install it and re-run."; exit 1; }
	cd analysis/aero/airfoil && ./run.sh

clean:
	rm -rf $(VENV) **/__pycache__ analysis/aero/**/out
