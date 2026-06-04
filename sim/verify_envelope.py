"""Standalone geometry verifier (no bpy). Recomputes the NACA-4412 wing skin +
center_fair fairing exactly as build.py does, generates the pilot body envelope
(torso loft + deltoids + head/helmet loft) exactly as build_frame does, and
checks at span stations y in {0,0.10,0.18,0.24} and two deploy states whether
every body point (+ suit thickness) stays BELOW the faired upper skin and ABOVE
the faired lower skin. Reports worst (most negative) clearance in mm + location.
Also confirms the fairing is exactly 0 at LE (xc=0) and TE (xc=1).
"""
import math

# ---------- constants copied verbatim from build.py ----------
PLAN_S = 6.5
PLAN_B = 6.3
PLAN_TAPER = 0.4
PLAN_SWEEP_DEG = 25.0
PLAN_WASHOUT_DEG = 7.0
HALF_SPAN_FULL = PLAN_B / 2.0
CHORD_ROOT = 2.0 * PLAN_S / (PLAN_B * (1.0 + PLAN_TAPER))
CHORD_TIP = PLAN_TAPER * CHORD_ROOT
TAN_SWEEP = math.tan(math.radians(PLAN_SWEEP_DEG))
X_ROOT_LE = 0.95
Z_WING = 0.10

AF_M, AF_P = 0.04, 0.40
AF_T_SKIN = 0.12
N_CHORD = 13

N_RIBS = 9
BILLOW = 0.14
RIB_ETA_IN, RIB_ETA_OUT = 0.20, 0.96

FAIR_Y_CORE = 0.26
FAIR_Y_OUT = 0.74

SHOULDER_HALF = 0.20
HIP_HALF = 0.145
UPPER_ARM, FOREARM, HAND = 0.32, 0.27, 0.17
THIGH, SHANK, FOOT = 0.43, 0.42, 0.20
TORSO_LEN = 0.52
NECK_LEN, HEAD_LEN = 0.10, 0.20
X_SHOULDER = 0.45
Z_BODY = Z_WING + 0.03

ARM_DIR_DEP = None  # set below via vnorm
RING_SEG = 10
SPHERE_RINGS = 6
SPHERE_SEG = 10

SUIT_T = 0.012  # suit/skin thickness to add to the body envelope


# ---------- helpers copied verbatim ----------
def vadd(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def vsub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vscale(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def vlerp(a, b, t): return (a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t, a[2]*(1-t)+b[2]*t)
def vdot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def vlen(a): return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
def vnorm(a):
    L = vlen(a) or 1e-9
    return (a[0]/L, a[1]/L, a[2]/L)
def vcross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

ARM_DIR_DEP = vnorm((0.33, 0.945, -0.02))
ARM_DIR_STOW = vnorm((0.55, 0.18, -0.55))
LEG_DIR_DEP = vnorm((-0.74, 0.63, -0.06))
LEG_DIR_STOW = vnorm((-0.92, 0.12, -0.30))
ARM_REACH = UPPER_ARM + FOREARM
LEG_REACH = THIGH + SHANK
WRIST_Y_DEP = SHOULDER_HALF + ARM_REACH * ARM_DIR_DEP[1]
ANKLE_Y_DEP = HIP_HALF + LEG_REACH * LEG_DIR_DEP[1]
INBOARD_DEP = max(WRIST_Y_DEP, ANKLE_Y_DEP)


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t*t*(3-2*t)


def naca4_surface(xc, m, p, t):
    xc = min(max(xc, 0.0), 1.0)
    yt = 5*t*(0.2969*math.sqrt(xc) - 0.1260*xc - 0.3516*xc**2
              + 0.2843*xc**3 - 0.1015*xc**4)
    if xc < p:
        yc = m/(p*p)*(2*p*xc - xc*xc)
        dyc = 2*m/(p*p)*(p-xc)
    else:
        yc = m/((1-p)**2)*((1-2*p)+2*p*xc-xc*xc)
        dyc = 2*m/((1-p)**2)*(p-xc)
    theta = math.atan(dyc)
    return yc + yt*math.cos(theta), yc - yt*math.cos(theta)


def _pbump(x, c, pl, tp, h):
    d = abs(x-c)
    if d < pl:
        return h
    if d < pl+tp:
        return h*0.5*(1+math.cos(math.pi*(d-pl)/tp))
    return 0.0


def center_fair(px, xc, y, surf):
    ry = abs(y)
    if ry >= FAIR_Y_OUT:
        return 0.0
    fy = 1.0 if ry <= FAIR_Y_CORE else smoothstep(
        (FAIR_Y_OUT - ry)/(FAIR_Y_OUT - FAIR_Y_CORE))
    seal = smoothstep(xc/0.05)*smoothstep((1.0-xc)/0.05)
    if surf == "u":
        off = max(_pbump(px, 0.17, 0.34, 0.40, 0.100),
                  _pbump(px, 0.70, 0.12, 0.34, 0.108))
        return off*fy*seal
    off = max(_pbump(px, 0.12, 0.26, 0.34, 0.140),
              _pbump(px, 0.70, 0.07, 0.18, 0.055))
    return -off*fy*seal


# planform
def inboard_reach(sp):
    def reach(d_stow, d_dep, r, base):
        return base + r*vnorm(vlerp(d_stow, d_dep, sp))[1]
    return max(reach(ARM_DIR_STOW, ARM_DIR_DEP, ARM_REACH, SHOULDER_HALF),
               reach(LEG_DIR_STOW, LEG_DIR_DEP, LEG_REACH, HIP_HALF))


def half_span(prog):
    return inboard_reach(prog["spread"]) + (HALF_SPAN_FULL - INBOARD_DEP)*prog["tip"]


def planform_chord(ay_full):
    eta = min(ay_full/HALF_SPAN_FULL, 1.0)
    return CHORD_ROOT - (CHORD_ROOT-CHORD_TIP)*eta


def chord_scale(prog):
    return 0.42 + 0.58*prog["spread"]


def le_point(y):
    return (X_ROOT_LE - abs(y)*TAN_SWEEP, y, Z_WING)


def chord_at(y, prog):
    return planform_chord(abs(y))*chord_scale(prog)


def _rot_y(point, pivot, ang):
    dx, dz = point[0]-pivot[0], point[2]-pivot[2]
    c, s = math.cos(ang), math.sin(ang)
    return (pivot[0]+c*dx+s*dz, point[1], pivot[2]-s*dx+c*dz)


def rib_eta_frac(k):
    return RIB_ETA_IN + (k+0.5)/N_RIBS*(RIB_ETA_OUT-RIB_ETA_IN)


# ---------- the faired skin z at a world (bx, by), for surf u/l ----------
def skin_z(bx, by, prog, surf):
    """World z of the faired wing skin at chord-x bx, span by. Mirrors
    wing_surface() exactly: NACA offset*chord*scallop, center_fair, then the
    washout twist rotation about the quarter-chord. Because the twist rotation
    shifts a point's world-x (and the tall canopy bump is rotated noticeably),
    we build the full rotated skin polyline at this station and interpolate the
    z at world-x = bx -- the rigorous height-field value (no chord-inversion
    approximation)."""
    le = le_point(by)
    c = chord_at(by, prog)
    ay = abs(by)
    eta = min(ay/HALF_SPAN_FULL, 1.0)
    twist = -math.radians(PLAN_WASHOUT_DEG)*eta
    qc = (le[0]-0.25*c, by, Z_WING)
    hs = half_span(prog)
    billow_eff = BILLOW*min(1.0, hs/HALF_SPAN_FULL)
    rel = ay/hs if hs > 0 else 0.0
    bill_gate = 0.0 if ay <= FAIR_Y_CORE else smoothstep(
        (ay-FAIR_Y_CORE)/(FAIR_Y_OUT-FAIR_Y_CORE))
    scallop = 1.0 + billow_eff*bill_gate*math.cos(math.pi*rel*N_RIBS)**2

    pts = []
    NS = 801
    for i in range(NS):
        xc = i/(NS-1)
        up, lo = naca4_surface(xc, AF_M, AF_P, AF_T_SKIN)
        off = up if surf == "u" else lo
        px = le[0]-xc*c
        pz = Z_WING + off*c*scallop + center_fair(px, xc, by, surf)
        p = _rot_y((px, by, pz), qc, twist)
        pts.append((p[0], p[2]))
    pts.sort()
    if bx <= pts[0][0]:
        return pts[0][1]
    if bx >= pts[-1][0]:
        return pts[-1][1]
    lo_i, hi_i = 0, len(pts)-1
    while hi_i - lo_i > 1:
        mid = (lo_i+hi_i)//2
        if pts[mid][0] <= bx:
            lo_i = mid
        else:
            hi_i = mid
    x0, z0 = pts[lo_i]; x1, z1 = pts[hi_i]
    f = (bx-x0)/((x1-x0) or 1e-9)
    return z0 + (z1-z0)*f


# ---------- pilot body envelope (verbatim loft / sphere math) ----------
def _orthoframe(ax):
    world_up = (0.0, 0.0, 1.0)
    if abs(vdot(world_up, ax)) > 0.97:
        world_up = (1.0, 0.0, 0.0)
    up = vnorm(vsub(world_up, vscale(ax, vdot(world_up, ax))))
    side = vnorm(vcross(ax, up))
    return up, side


def loft_verts(centers, radii):
    """Return all ring vertices of a loft (matches Mesh.loft)."""
    n = len(centers)
    out = []
    for i in range(n):
        ax = vnorm(vsub(centers[min(i+1, n-1)], centers[max(i-1, 0)]))
        up, side = _orthoframe(ax)
        ry, rz = radii[i]
        cen = centers[i]
        for k in range(RING_SEG):
            a = 2*math.pi*k/RING_SEG
            cc, ss = math.cos(a), math.sin(a)
            out.append(vadd(cen, vadd(vscale(side, cc*ry), vscale(up, ss*rz))))
    return out


def sphere_verts(center, r):
    out = []
    for i in range(SPHERE_RINGS+1):
        phi = math.pi*i/SPHERE_RINGS
        z = math.cos(phi); rr = math.sin(phi)
        for j in range(SPHERE_SEG):
            th = 2*math.pi*j/SPHERE_SEG
            out.append((center[0]+r*rr*math.cos(th),
                        center[1]+r*rr*math.sin(th),
                        center[2]+r*z))
    return out


def pilot_fk(prog, lean=(0.0, 0.0, 0.0)):
    sp = prog["spread"]
    sh_c = vadd((X_SHOULDER, 0.0, Z_BODY), lean)
    hip_c = vadd((X_SHOULDER-TORSO_LEN, 0.0, Z_BODY-0.02), lean)
    N = {}
    for side, sgn in (("R", 1.0), ("L", -1.0)):
        sh = vadd((X_SHOULDER, sgn*SHOULDER_HALF, Z_BODY), lean)
        N[side] = dict(sh=sh)
    N["sh_c"] = sh_c
    N["hip_c"] = hip_c
    N["neck"] = vadd(sh_c, (0.07, 0, 0.040))
    N["head"] = vadd((X_SHOULDER+0.29, 0.0, Z_BODY+0.048), lean)
    return N


def body_envelope_verts(prog):
    """Every vertex of the body parts that occupy the cockpit span (torso loft,
    deltoid spheres, neck+head/helmet loft). Returns list of (x,y,z)."""
    N = pilot_fk(prog)
    sh_c, hip_c = N["sh_c"], N["hip_c"]
    verts = []
    # torso loft (verbatim from build_frame)
    torso_line = [vadd(sh_c, (0.02, 0, 0.01)),
                  vlerp(sh_c, hip_c, 0.26), vlerp(sh_c, hip_c, 0.52),
                  vlerp(sh_c, hip_c, 0.76), hip_c]
    torso_rad = [(0.205, 0.105), (0.180, 0.118), (0.140, 0.105),
                 (0.135, 0.102), (0.165, 0.110)]
    verts += [(v, "torso") for v in loft_verts(torso_line, torso_rad)]
    # deltoid spheres
    for s in ("R", "L"):
        verts += [(v, "deltoid") for v in sphere_verts(N[s]["sh"], 0.066)]
    # neck loft (helmet)
    verts += [(v, "neck") for v in loft_verts([sh_c, N["neck"]],
                                              [(0.078, 0.072), (0.054, 0.052)])]
    # head loft (helmet)
    head = N["head"]
    verts += [(v, "head") for v in loft_verts(
        [N["neck"], vadd(head, (-0.04, 0, -0.01)), head, vadd(head, (0.085, 0, 0.0))],
        [(0.054, 0.052), (0.076, 0.082), (0.078, 0.084), (0.040, 0.046)])]
    return verts


# ---------- run the checks ----------
def check(prog, label):
    verts = body_envelope_verts(prog)
    worst_u = (1e9, None)   # (clearance, info) upper: body must be BELOW skin
    worst_l = (1e9, None)   # lower: body must be ABOVE skin
    # also bin worst per requested span station
    stations = [0.0, 0.10, 0.18, 0.24]
    per_st = {s: (1e9, None, 1e9, None) for s in stations}
    for (v, part) in verts:
        bx, by, bz = v
        # only check inside the faired cockpit span (|y| < FAIR_Y_OUT) where the
        # fairing exists; outboard of that the limbs are checked elsewhere.
        if abs(by) >= FAIR_Y_OUT:
            continue
        zu = skin_z(bx, by, prog, "u")
        zl = skin_z(bx, by, prog, "l")
        # upper clearance: how far below the upper skin the suited body top is
        cu = (zu - (bz + SUIT_T))*1000.0   # mm ; negative => pierces up
        # lower clearance: how far above the lower skin the suited body bottom is
        cl = ((bz - SUIT_T) - zl)*1000.0   # mm ; negative => pierces down
        if cu < worst_u[0]:
            worst_u = (cu, (part, bx, by, bz, zu))
        if cl < worst_l[0]:
            worst_l = (cl, (part, bx, by, bz, zl))
        # nearest requested station bin
        s = min(stations, key=lambda S: abs(abs(by)-S))
        wu, iu, wl, il = per_st[s]
        if cu < wu:
            wu, iu = cu, (part, bx, by, bz)
        if cl < wl:
            wl, il = cl, (part, bx, by, bz)
        per_st[s] = (wu, iu, wl, il)

    print(f"\n=== {label}  hs={half_span(prog):.3f} m  spread={prog['spread']:.2f} tip={prog['tip']:.2f} ===")
    cu0, info = worst_u
    print(f"  WORST UPPER clearance: {cu0:+.1f} mm  "
          f"(part={info[0]} at x={info[1]:.3f} y={info[2]:.3f} bodyZ={info[3]:.3f} skinZu={info[4]:.3f})")
    cl0, info = worst_l
    print(f"  WORST LOWER clearance: {cl0:+.1f} mm  "
          f"(part={info[0]} at x={info[1]:.3f} y={info[2]:.3f} bodyZ={info[3]:.3f} skinZl={info[4]:.3f})")
    print("  per nearest span-station |y| (mm, neg=pierce):")
    for s in stations:
        wu, iu, wl, il = per_st[s]
        us = f"{wu:+.1f}@{iu[0]}" if iu else "n/a"
        ls = f"{wl:+.1f}@{il[0]}" if il else "n/a"
        print(f"    y~{s:.2f}: upper {us:>16}   lower {ls:>16}")
    return cu0, cl0


# deploy states: full span and ~half span
PROG_FULL = {"spread": 1.0, "tip": 1.0, "rib": 1.0, "flap": 1.0}
# find a tip fraction giving hs ~ half of full. spread=1 -> inboard_reach fixed.
ir = inboard_reach(1.0)
# hs = ir + (HALF_SPAN_FULL - INBOARD_DEP)*tip ; want hs ~ HALF_SPAN_FULL/2
target = HALF_SPAN_FULL/2.0
tip_half = (target - ir)/(HALF_SPAN_FULL - INBOARD_DEP)
tip_half = max(0.0, min(1.0, tip_half))
PROG_HALF = {"spread": 1.0, "tip": tip_half, "rib": 1.0, "flap": 1.0}

print("inboard_reach(spread=1) =", round(ir, 4),
      " INBOARD_DEP =", round(INBOARD_DEP, 4),
      " HALF_SPAN_FULL =", round(HALF_SPAN_FULL, 4))
print("tip fraction for half span =", round(tip_half, 4),
      " -> hs =", round(half_span(PROG_HALF), 4))

r1 = check(PROG_FULL, "FULLY DEPLOYED (hs=full)")
r2 = check(PROG_HALF, "MID DEPLOY (hs~half)")

# ---------- LE/TE seal check ----------
print("\n=== FAIRING SEAL CHECK (must be exactly 0 at xc=0 LE and xc=1 TE) ===")
maxseal = 0.0
for surf in ("u", "l"):
    for y in (0.0, 0.10, 0.18, 0.24):
        for xc in (0.0, 1.0):
            le = le_point(y); c = chord_at(y, PROG_FULL)
            px = le[0]-xc*c
            f = center_fair(px, xc, y, surf)
            maxseal = max(maxseal, abs(f))
            print(f"  surf={surf} y={y:.2f} xc={xc:.0f}  fairing={f:+.6e} m")
# also sample very close to LE/TE to confirm continuity toward 0
print("  near-LE/TE samples (xc=0.01, 0.99) at y=0:")
for xc in (0.01, 0.99):
    le = le_point(0.0); c = chord_at(0.0, PROG_FULL)
    px = le[0]-xc*c
    print(f"    xc={xc} u={center_fair(px, xc, 0.0, 'u'):+.6e}  "
          f"l={center_fair(px, xc, 0.0, 'l'):+.6e}")
print(f"  max |fairing| at xc in {{0,1}} = {maxseal:.3e} m  "
      f"({'SEALED' if maxseal == 0.0 else 'NOT SEALED'})")

print("\n=== SUMMARY ===")
print(f"  full-deploy: worst upper {r1[0]:+.1f} mm, worst lower {r1[1]:+.1f} mm")
print(f"  mid-deploy : worst upper {r2[0]:+.1f} mm, worst lower {r2[1]:+.1f} mm")
worst = min(r1[0], r1[1], r2[0], r2[1])
print(f"  GLOBAL worst clearance = {worst:+.1f} mm  "
      f"({'PIERCES' if worst < 0 else 'clear'})")
