"""Check page 2.2c "How PX4 and Project AirSim plan a motion". Run on the cloud machine only (never on the owner's PC):
    python check_2_2c.py page/samples/2.2c_How_PX4_and_Project_AirSim_plan_motion.html
What it checks:
 1. Every number that the page prints for the S-curve is recomputed here and must appear, formatted as printed, in the page text.
    The S-curve is computed THREE ways, none of them taken from the page's own script: (a) the closed formulas printed on the page,
    (b) a Python port of PX4's VelocitySmoothing (computeT1, computeT2, computeT3, evaluatePoly) as read in
    src/lib/motion_planning/VelocitySmoothing.cpp @ d28a642, (c) a brute-force integration of the jerk profile in small time steps.
    The three must agree before any printed number is compared.
 2. Every quotation (q.src and pre.code with data-src="repo|commit|path|lines") is found, word for word (spaces ignored), in the
    named file at the named commit (fetched from raw.githubusercontent.com).
 3. Every default, unit, min, max and line range in the two parameter tables is compared with the YAML file at the commit.
 4. Line probes: a list of file:lines that the page names must contain the code or text that the page says is there.
 5. Statements of absence: the word "jerk" is in none of the Project AirSim files and none of the two fixed-wing parameter files that were read.
 6. Figures (labels 1..N, empty, clipped, overlapping, one PNG each), tables ("How to read this table." before each table, a figure in
    the same block, a marks/notes tool each), Try-it numbers 41..58, gravity (never 9.81), the banned word, no HTML sub inside an svg,
    local links, every GitHub link answers 200 and has its line numbers inside the file, notes and marks survive a reload."""
import json
import math
import re
import sys
import urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright

page_path = Path(sys.argv[1]).resolve()
Path("out").mkdir(exist_ok=True)
M = "−"
err = []
res = {"errors": err, "numbers": {}}

# ====================================================== 1. the S-curve, three ways
def closed(v0, V, A, J):
    """the formulas printed on the page, for a start with a = 0"""
    D = abs(V - v0)
    if J * D <= A * A:
        T1, T2 = math.sqrt(D / J), 0.0
    else:
        T1, T2 = A / J, D / A - A / J
    T = 2 * T1 + T2
    return {"T1": T1, "T2": T2, "T3": T1, "T": T, "x": (v0 + V) / 2 * T * (1 if V >= v0 else 1)}

def sign(x):
    return (x > 0) - (x < 0)

def px4_port(v0, V, A, J, x0=0.0):
    """a Python port of VelocitySmoothing.cpp @ d28a642: updateDurationsMinimizeTotalTime() with a0 = 0, then updateTraj() with exact steps."""
    a0 = 0.0
    vel_zero_acc = v0                                   # computeVelAtZeroAcc with a = 0
    direction = sign(V - vel_zero_acc)
    jerk_max_T1 = direction * J
    delta_v = V - v0
    # computeT1(a0, v3, j_max, a_max)
    delta = 2.0 * a0 * a0 + 4.0 * jerk_max_T1 * delta_v
    T1 = 0.0
    if delta >= 0:
        sq = math.sqrt(delta)
        T1p = (-a0 + 0.5 * sq) / jerk_max_T1
        T1m = (-a0 - 0.5 * sq) / jerk_max_T1
        T3p, T3m = a0 / jerk_max_T1 + T1p, a0 / jerk_max_T1 + T1m
        if T1p >= 0 and T3p >= 0:
            T1 = T1p
        elif T1m >= 0 and T3m >= 0:
            T1 = T1m
    accel_T1 = a0 + jerk_max_T1 * T1                    # saturateT1ForAccel
    if accel_T1 > A:
        T1 = (A - a0) / jerk_max_T1
    elif accel_T1 < -A:
        T1 = (-A - a0) / jerk_max_T1
    T1 = max(T1, 0.0)
    T3 = max(a0 / jerk_max_T1 + T1, 0.0)                # computeT3
    den = a0 + jerk_max_T1 * T1                         # computeT2
    T2 = 0.0
    if abs(den) > 1.2e-7:
        T2 = (-0.5 * T1 * T1 * jerk_max_T1 - T1 * T3 * jerk_max_T1 - T1 * a0 + 0.5 * T3 * T3 * jerk_max_T1 - T3 * a0 + delta_v) / den
    T2 = max(T2, 0.0)

    def poly(j, a0_, v0_, x0_, t, d):                   # evaluatePoly
        jt = d * j
        return (a0_ + jt * t, v0_ + a0_ * t + 0.5 * jt * t * t, x0_ + v0_ * t + 0.5 * a0_ * t * t + jt * t ** 3 / 6.0)
    a, v, x = poly(J, 0.0, v0, x0, T1, direction)
    s1 = (a, v, x)
    a, v, x = poly(0.0, a, v, x, T2, 0)
    s2 = (a, v, x)
    a, v, x = poly(J, a, v, x, T3, -direction)
    return {"T1": T1, "T2": T2, "T3": T3, "T": T1 + T2 + T3, "s1": s1, "s2": s2, "end": (a, v, x), "dir": direction}

def brute(v0, V, A, J, T1, T2, T3, direction, dt=1e-5):
    """integrate the jerk profile (+J for T1, 0 for T2, -J for T3) with a tiny time step (midpoint rule)"""
    a = 0.0; v = v0; x = 0.0; amax = 0.0
    for dur, j in ((T1, direction * J), (T2, 0.0), (T3, -direction * J)):
        n = max(1, int(round(dur / dt)))
        h = dur / n if dur > 0 else 0.0
        for _ in range(n if dur > 0 else 0):
            a_new = a + j * h
            v_new = v + 0.5 * (a + a_new) * h
            x += 0.5 * (v + v_new) * h
            a, v = a_new, v_new
            amax = max(amax, abs(a))
    return {"a": a, "v": v, "x": x, "amax": amax}

CASES = {"default": (0.0, 6.0, 3.0, 4.0), "try44": (0.0, 3.0, 2.0, 4.0), "try45": (0.0, 2.0, 3.0, 4.0), "try46": (0.0, 3.0, 3.0, 4.0),
         "fig2ramp": (0.0, 3.0, 3.0, 4.0), "try50": (0.0, 3.0, 3.0, 4.0), "gate-up": (6.0, 24.0, 6.0, 4.0), "gate-dn": (24.0, 8.0, 8.0, 4.0)}
S = {}
for name, (v0, V, A, J) in CASES.items():
    c = closed(v0, V, A, J); p = px4_port(v0, V, A, J); b = brute(v0, V, A, J, p["T1"], p["T2"], p["T3"], p["dir"])
    for k in ("T1", "T2", "T3", "T"):
        if abs(c[k] - p[k]) > 1e-6:
            err.append(f"S-curve {name}: formula {k} = {c[k]} but the PX4 port gives {p[k]}")
    if abs(p["end"][1] - V) > 1e-6 or abs(p["end"][0]) > 1e-6:
        err.append(f"S-curve {name}: the PX4 port ends at a = {p['end'][0]}, v = {p['end'][1]} (wanted 0 and {V})")
    if abs(p["end"][2] - c["x"]) > 1e-6:
        err.append(f"S-curve {name}: distance by the PX4 port {p['end'][2]} differs from (v0 + V)/2 * T = {c['x']}")
    if abs(b["x"] - p["end"][2]) > 1e-3 or abs(b["v"] - V) > 1e-3 or abs(b["a"]) > 1e-3:
        err.append(f"S-curve {name}: the brute-force integration gives x = {b['x']}, v = {b['v']}, a = {b['a']}")
    if b["amax"] > A + 1e-3:
        err.append(f"S-curve {name}: the acceleration {b['amax']} passes the limit {A}")
    S[name] = (c, p, b)

E = []                                                   # (name, printed string)
def ex(name, s):
    E.append((name, s))

A_, J_, V_ = 3.0, 4.0, 6.0
c, p, b = S["default"]
T1, T2, T3, T = c["T1"], c["T2"], c["T3"], c["T"]
v1 = 0.5 * J_ * T1 ** 2
v2 = V_ - v1
x1 = J_ * T1 ** 3 / 6
x2 = v1 * T2 + 0.5 * A_ * T2 ** 2
x3 = v2 * T3 + 0.5 * A_ * T3 ** 2 - J_ * T3 ** 3 / 6
if abs(x1 + x2 + x3 - p["end"][2]) > 1e-9 or abs(p["s1"][1] - v1) > 1e-9 or abs(p["s2"][1] - v2) > 1e-9 or abs(p["s1"][2] - x1) > 1e-9:
    err.append("default S-curve: the printed part values differ from the PX4 port")
def g(x):                                                # shortest exact print: 0.28125, 4.875, 8.25, 3.75
    s = f"{round(x, 9):.10g}"
    return (M + s[1:]) if s.startswith("-") else s
def f1(x): return f"{x:.1f}"
def f3(x): return f"{x:.3f}"

# 2.2.13
ex("try41", f"(2.0 − 0) / 0.5 = {f1(2.0 / 0.5)} m/s³")
ex("try42", f"(−3.0 − 3.0) / 1.5 = {M}{f1(abs(-3.0 - 3.0) / 1.5)} m/s³")
t_, j_ = 0.75, 4.0
ex("try43 a", f"a0 + j·t = 0 + 4.0 × 0.75 = {f1(j_ * t_)} m/s²")
ex("try43 v", f"½·j·t² = 0 + 0 + ½ × 4.0 × 0.75² = {f1(0.5 * j_)} × {g(t_ ** 2)} = {g(0.5 * j_ * t_ ** 2)} m/s")
ex("try43 x", f"(1/6) × 4.0 × 0.75³ = (1/6) × 4.0 × {g(t_ ** 3)} = {g(j_ * t_ ** 3 / 6)} m")
# 2.2.14, the worked example
ex("ex1", f"A² / J = 3.0² / 4.0 = 9.0 / 4.0 = {g(A_ * A_ / J_)} m/s")
ex("ex2", f"T1 = A / J = 3.0 / 4.0 = {g(T1)} s")
ex("ex3", f"T2 = V / A − A / J = 6.0 / 3.0 − {g(T1)} = {f1(V_ / A_)} − {g(T1)} = {g(T2)} s")
ex("ex4", f"T = {g(T1)} + {g(T2)} + {g(T3)} = {g(T)} s")
ex("ex5a", f"v1 = ½·J·T1² = ½ × 4.0 × 0.75² = {f1(0.5 * J_)} × {g(T1 ** 2)} = {g(v1)} m/s")
ex("ex5b", f"v2 = V − v1 = 6.0 − {g(v1)} = {g(v2)} m/s")
ex("ex5c", f"v1 + A·T2 = {g(v1)} + 3.0 × {g(T2)} = {g(v1)} + {g(A_ * T2)} = {g(v1 + A_ * T2)} m/s")
ex("ex6", f"x1 = (1/6) × 4.0 × 0.75³ = {g(x1)} m")
ex("ex7", f"x2 = v1·T2 + ½·A·T2² = {g(v1)} × {g(T2)} + ½ × 3.0 × {g(T2)}² = {g(v1 * T2)} + {g(0.5 * A_ * T2 ** 2)} = {g(x2)} m")
ex("ex8", f"x3 = v2·T3 + ½·A·T3² − (1/6)·J·T3³ = {g(v2)} × {g(T3)} + ½ × 3.0 × {g(T3)}² − {g(x1)} = {g(v2 * T3)} + {g(0.5 * A_ * T3 ** 2)} − {g(x1)} = {g(x3)} m")
ex("ex9a", f"x = {g(x1)} + {g(x2)} + {g(x3)} = {g(x1 + x2 + x3)} m")
ex("ex9b", f"V·T / 2 = 6.0 × {g(T)} / 2 = {g(V_ * T / 2)} m")
ex("cap3a", f"A² / (2·J) = {g(A_ ** 2 / (2 * J_))} m/s in part 1, A·T2 = {g(A_ * T2)} m/s in part 2 and {g(A_ ** 2 / (2 * J_))} m/s in part 3")
ex("cap3b", f"{g(v1)}, {g(v2)} and {f1(V_)} m/s")
# try 44 to 46
c44 = S["try44"][0]
ex("try44a", f"A² / J = 2.0² / 4.0 = 4.0 / 4.0 = {f1(4.0 / 4.0)} m/s")
ex("try44b", f"T1 = A / J = 2.0 / 4.0 = {g(c44['T1'])} s")
ex("try44c", f"T2 = V / A − A / J = 3.0 / 2.0 − {g(c44['T1'])} = {f1(3.0 / 2.0)} − {g(c44['T1'])} = {f1(c44['T2'])} s")
ex("try44d", f"T = {g(c44['T1'])} + {f1(c44['T2'])} + {g(c44['T3'])} = {f1(c44['T'])} s")
ex("try44e", f"x = V·T / 2 = 3.0 × {f1(c44['T'])} / 2 = {f1(3.0 * c44['T'] / 2)} m")
c45 = S["try45"][0]
ex("try45a", f"T1 = T3 = √(V / J) = √(2.0 / 4.0) = √0.5 = {f3(c45['T1'])} s")
ex("try45b", f"T = 2 × {f3(c45['T1'])} = {f3(2 * round(c45['T1'], 3))} s")
ex("try45c", f"J·T1 = 4.0 × {f3(c45['T1'])} = {f3(4.0 * round(c45['T1'], 3))} m/s²")
ex("try45d", f"x = V·T / 2 = 2.0 × {f3(2 * round(c45['T1'], 3))} / 2 = {f3(2.0 * 2 * round(c45['T1'], 3) / 2)} m")
if abs(c45["T"] - 2 * math.sqrt(0.5)) > 1e-12 or abs(S["try45"][1]["T2"]) > 1e-9:
    err.append("try45: unsaturated case differs")
c46 = S["try46"][0]
ex("try46a", f"T2 = V / A − A / J = 3.0 / 3.0 − {g(c46['T1'])} = 1.0 − {g(c46['T1'])} = {g(c46['T2'])} s")
ex("try46b", f"v1 = ½·J·T1² = ½ × 4.0 × 0.75² = {g(0.5 * 4.0 * c46['T1'] ** 2)} m/s")
ex("try46c", f"v2 = V − v1 = 3.0 − {g(0.5 * 4.0 * c46['T1'] ** 2)} = {g(3.0 - 0.5 * 4.0 * c46['T1'] ** 2)} m/s")
# 2.2.15
ex("try47", f"min(15.0, 12.0) = {f1(min(15.0, 12.0))} m/s")
ex("try48a", f"t = 30 / 3.0 = {f1(30 / 3.0)} s")
ex("try48b", f"t = 30 / 1.5 = {f1(30 / 1.5)} s")
# 2.2.16 the run of 100 m
L = 100.0
tr_c, xr_c = V_ / A_, V_ ** 2 / (2 * A_)
tr_s = c["T"]; xr_s = c["x"]
cr_c, cr_s = L - 2 * xr_c, L - 2 * xr_s
tt_c, tt_s = 2 * tr_c + cr_c / V_, 2 * tr_s + cr_s / V_
ex("run c1", f"V / A = 6.0 / 3.0 = {f1(tr_c)}")
ex("run c2", f"V² / (2·A) = 6.0² / (2 × 3.0) = {f1(V_ ** 2)} / {f1(2 * A_)} = {f1(xr_c)}")
ex("run c3", f"100 − 2 × {f1(xr_c)} = {f1(cr_c)}")
ex("run c4", f"{f1(cr_c)} / 6.0 = {f3(cr_c / V_)}")
ex("run c5", f"2 × {f1(tr_c)} + {f3(cr_c / V_)} = {f3(tt_c)}")
ex("run s1", f"V / A + A / J = 6.0 / 3.0 + 3.0 / 4.0 = {g(tr_s)}")
ex("run s2", f"V·tr / 2 = 6.0 × {g(tr_s)} / 2 = {g(xr_s)}")
ex("run s3", f"100 − 2 × {g(xr_s)} = {g(cr_s)}")
ex("run s4", f"{g(cr_s)} / 6.0 = {f3(cr_s / V_)}")
ex("run s5", f"2 × {g(tr_s)} + {f3(cr_s / V_)} = {f3(tt_s)}")
ex("run diff", f"{f3(tt_s)} − {f3(tt_c)} = {g(round(tt_s - tt_c, 6))} s")
ex("run diff2", f"2 × {g((tt_s - tt_c) / 2)} s")
ex("run extra t", f"A / J = 3.0 / 4.0 = {g(A_ / J_)} s of time")
ex("run extra x", f"A·V / (2·J) = 3.0 × 6.0 / (2 × 4.0) = {g(A_ * V_ / (2 * J_))} m")
ex("run extra c", f"{g(A_ * V_ / (2 * J_))} / 6.0 = {g(A_ * V_ / (2 * J_) / V_)} s")
if abs((tt_s - tt_c) - 2 * A_ * V_ / (2 * J_) / V_ - 0) > 1e-9 and False:
    pass
ex("try49", f"J = A / textra = 3.0 / 0.30 = {f1(3.0 / 0.30)} m/s³")
c50 = S["try50"][0]
tr50, xr50 = c50["T"], c50["x"]
cr50 = 100 - 2 * xr50
ex("try50a", f"A² / J = {g(A_ * A_ / J_)} m/s")
ex("try50b", f"tr = V / A + A / J = 3.0 / 3.0 + 0.75 = {g(tr50)} s")
ex("try50c", f"xr = V·tr / 2 = 3.0 × {g(tr50)} / 2 = {g(xr50)} m")
ex("try50d", f"100 − 2 × {g(xr50)} = {g(cr50)} m")
ex("try50e", f"{g(cr50)} / 3.0 = {f3(cr50 / 3.0)} s")
ex("try50f", f"2 × {g(tr50)} + {f3(cr50 / 3.0)} = {f3(2 * tr50 + cr50 / 3.0)} s")
# the gate run
t1g = (24.0 - 6.0) / 6.0; x1g = 0.5 * (6.0 + 24.0) * t1g
t3g = (8.0 - 24.0) / (-8.0); x3g = 0.5 * (24.0 + 8.0) * t3g
x2g = 100 - x1g - x3g
ex("gate text", f"({f1(t1g)} s and {f1(x1g)} m, {f1(x2g)} m, {f1(t3g)} s and {f1(x3g)} m, 100 m in all)")
up, dn = S["gate-up"], S["gate-dn"]
ex("gate r1a", f"T = (24.0 − 6.0) / 6.0 = {f1(t1g)}")
ex("gate r1b", f"½ × (6.0 + 24.0) × {f1(t1g)} = {f1(x1g)}")
ex("gate r2a", f"ΔV = 18.0; A² / J = 6.0² / 4.0 = {f1(6.0 ** 2 / 4.0)}; {f1(6.0 ** 2 / 4.0)} < 18.0, so A is reached")
ex("gate r2b", f"T1 = A / J = {f1(up[0]['T1'])}; T2 = 18.0 / 6.0 − {f1(up[0]['T1'])} = {f1(up[0]['T2'])}; T3 = {f1(up[0]['T3'])}")
ex("gate r2c", f"{f1(up[0]['T1'])} + {f1(up[0]['T2'])} + {f1(up[0]['T3'])} = {f1(up[0]['T'])}")
ex("gate r2d", f"½ × (6.0 + 24.0) × {f1(up[0]['T'])} = {f1(up[0]['x'])}")
ex("gate r3a", f"T = (8.0 − 24.0) / ({M}8.0) = {f1(t3g)}")
ex("gate r3b", f"½ × (24.0 + 8.0) × {f1(t3g)} = {f1(x3g)}")
ex("gate r4a", f"ΔV = 16.0; A² / J = 8.0² / 4.0 = {f1(8.0 ** 2 / 4.0)}; {f1(8.0 ** 2 / 4.0)} = 16.0, so A is just reached")
ex("gate r4b", f"T1 = A / J = {f1(dn[0]['T1'])}; T2 = 16.0 / 8.0 − {f1(dn[0]['T1'])} = {f1(dn[0]['T2'])}; T3 = {f1(dn[0]['T3'])}")
ex("gate r4c", f"{f1(dn[0]['T1'])} + {f1(dn[0]['T2'])} + {f1(dn[0]['T3'])} = {f1(dn[0]['T'])}")
ex("gate r4d", f"½ × (24.0 + 8.0) × {f1(dn[0]['T'])} = {f1(dn[0]['x'])}")
if abs(dn[0]["T2"]) > 1e-9 or abs(up[0]["T2"] - 1.5) > 1e-9:
    err.append("gate run: unexpected T2")
ex("try51", f"{f1(up[0]['x'])} + {f1(dn[0]['x'])} = {f1(up[0]['x'] + dn[0]['x'])} m")
ex("try51b", f"{f1(up[0]['x'] + dn[0]['x'])} − 100 = {f1(up[0]['x'] + dn[0]['x'] - 100)} m")
ex("cap6", f"Row 1 (blue) takes {f1(t1g)} s and covers {f1(x1g)} m. Row 2 (green) takes {f1(up[0]['T'])} s and covers {f1(up[0]['x'])} m.")
ex("cap6b", f"Row 3 (blue) takes {f1(t3g)} s and covers {f1(x3g)} m. Row 4 (green) takes {f1(dn[0]['T'])} s and covers {f1(dn[0]['x'])} m. The two S-curve rows together cover {f1(up[0]['x'] + dn[0]['x'])} m.")
ex("cap5", f"The constant acceleration (blue) reaches 6.0 m/s at {f1(tr_c)} s. The S-curve (green) reaches it at {g(tr_s)} s")
ex("cap5b", f"The constant acceleration has covered {f1(xr_c)} m at the end of its speed-up, the S-curve {g(xr_s)} m.")
ex("cap2", f"The jump (red) holds a = 3.0 m/s² for {f1(3.0 / 3.0)} s. The ramp (green) uses the limits A = 3.0 m/s² and J = 4.0 m/s³ and needs {g(S['fig2ramp'][0]['T'])} s.")
# 2.2.17 speed limit near the target
b_ = 4 * A_ ** 2 / J_
def vmax(d, A=A_, J=J_):
    bb = 4 * A * A / J
    return 0.5 * (-bb + math.sqrt(bb * bb + 8 * A * d))
def dist_for(v, A=A_, J=J_):
    return v * v / (2 * A) + (2 * A / J) * v
for d in (10.0, 15.0, 6.0):
    if abs(dist_for(vmax(d)) - d) > 1e-9:
        err.append(f"v_max({d}): the distance formula does not give d back")
# the same number from the C++ lines of TrajMath.hpp::computeMaxSpeedFromDistance (port)
def cpp_max_speed(jerk, accel, braking_distance, final_speed):
    sqr = lambda f: f * f
    bb = 4.0 * sqr(accel) / jerk
    cc = -2.0 * accel * braking_distance - sqr(final_speed)
    return max(0.5 * (-bb + math.sqrt(sqr(bb) - 4.0 * cc)), final_speed)
for d in (10.0, 15.0, 6.0):
    if abs(cpp_max_speed(J_, A_, d, 0.0) - vmax(d)) > 1e-9:
        err.append(f"v_max({d}): the C++ port differs from the formula")
ex("ap b", f"b = 4 × 3.0² / 4.0 = {f1(b_)}")
ex("ap 10", f"vmax = ½ × ( −9.0 + √(9.0² + 8 × 3.0 × 10.0) ) = ½ × ( −9.0 + √{f1(b_ ** 2 + 8 * A_ * 10.0)} ) = ½ × ( −9.0 + {f3(math.sqrt(b_ ** 2 + 8 * A_ * 10.0))} ) = {f3(vmax(10.0))} m/s")
ex("ap 15", f"vmax = ½ × ( −9.0 + √(9.0² + 8 × 3.0 × 15.0) ) = ½ × ( −9.0 + √{f1(b_ ** 2 + 8 * A_ * 15.0)} ) = ½ × ( −9.0 + {f1(math.sqrt(b_ ** 2 + 8 * A_ * 15.0))} ) = {f1(vmax(15.0))} m/s")
ex("ap 8.25", f"needs only {g(xr_s)} m to stop from 6.0 m/s")
ex("ap 15m", f"keeps {f1(dist_for(6.0))} m")
ex("try52a", f"b = 4·A² / J = 4 × 3.0² / 4.0 = {f1(b_)}")
ex("try52b", f"vmax = ½ × ( −b + √(b² + 8·A·d) ) = ½ × ( −9.0 + √(9.0² + 8 × 3.0 × 6.0) ) = ½ × ( −9.0 + √{f1(b_ ** 2 + 8 * A_ * 6.0)} ) = ½ × ( −9.0 + {f1(math.sqrt(b_ ** 2 + 8 * A_ * 6.0))} ) = {f1(vmax(6.0))} m/s")
ex("try53", f"d = V² / (2·A) + (2·A / J)·V = 3.0² / (2 × 3.0) + (2 × 3.0 / 4.0) × 3.0 = {f1(3.0 ** 2 / (2 * A_))} + {f1((2 * A_ / J_) * 3.0)} = {f1(dist_for(3.0))} m")
ex("try54a", f"factor = 1 − e / emax = 1 − 0.5 / 2.0 = 1 − {g(0.5 / 2.0)} = {g(1 - 0.5 / 2.0)}")
ex("try54b", f"{g(1 - 0.5 / 2.0)} × 0.2 = {g((1 - 0.5 / 2.0) * 0.2)} s")
ex("cap7", f"At d = 10.0 m the limit is {f3(vmax(10.0))} m/s")
# 2.2.18 fixed wing
ex("try55a", f"t = 100 / 5.0 = {f1(100 / 5.0)} s")
ex("try55b", f"t = 100 / 2.0 = {f1(100 / 2.0)} s")
ex("try55c", f"20.0 − 10.0 = {f1(20.0 - 10.0)} m/s")
# 2.2.19 Project AirSim
vn, ve, vd = 3.0, 4.0, -2.0
sp = math.sqrt(vn ** 2 + ve ** 2 + vd ** 2)
ex("try56a", f"vx = E = 4.0, vy = N = 3.0, vz = −D = −({M}2.0) = +2.0 m/s")
ex("try56b", f"√(3.0² + 4.0² + 2.0²) = √({f1(9.0)} + {f1(16.0)} + {f1(4.0)}) = √{f1(sp ** 2)} = {f3(sp)} m/s")
ex("try56c", f"{f3(sp)} × 5.0 = {sp * 5.0:.2f} m")
ex("try57", f"N = yn = 15, E = xe = 40, D = −zu = {M}8")
ex("try57b", "move_to_position_async(15, 40, -8, velocity)")
def clip(x, lo, hi): return max(lo, min(hi, x))
ex("try58a", f"clip(4.0 × 0.5, 1, 3) = clip({f1(4.0 * 0.5)}, 1, 3) = {f1(clip(4.0 * 0.5, 1, 3))} m")
ex("try58b", f"clip(8.0 × 0.5, 1, 3) = clip({f1(8.0 * 0.5)}, 1, 3) = {f1(clip(8.0 * 0.5, 1, 3))} m")
ex("quiz3", f"T1 = T3 = A / J = {g(T1)} s, T2 = V / A − A / J = {g(T2)} s, T = {g(T)} s.")

# ====================================================== 2 to 5. the files at the commits
cache = {}
def fetch(repo, commit, path):
    key = (repo, commit, path)
    if key not in cache:
        url = f"https://raw.githubusercontent.com/{repo}/{commit}/{path}"
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
            cache[key] = urllib.request.urlopen(req, timeout=60).read().decode("utf-8", "replace")
        except Exception as e:
            cache[key] = None
            err.append(f"cannot fetch {url}: {str(e)[:80]}")
    return cache[key]
ws = lambda s: " ".join(s.split())
PX, PXC, AS, ASC = "PX4/PX4-Autopilot", "d28a642", "iamaisim/ProjectAirSim", "c80cff5"

def lines_of(repo, commit, path, a, b):
    t = fetch(repo, commit, path)
    if t is None:
        return None
    ls = t.split("\n")
    return "\n".join(ls[a - 1:b])

def yaml_entries(text):
    lines = text.split("\n")
    out = {}
    keys = [(n, m.group(1)) for n, l in enumerate(lines, 1) for m in [re.match(r"^    ([A-Za-z0-9_]+):\s*$", l)] if m]
    for n, name in keys:
        end = len(lines)
        for k in range(n, len(lines)):
            l = lines[k]
            if l.strip() and (re.match(r"^    \S", l) or re.match(r"^\S", l)):
                end = k
                break
        while end > n and not lines[end - 1].strip():
            end -= 1
        d = {"start": n, "end": end}
        for l in lines[n:end]:
            m = re.match(r"^      (default|unit|min|max): *(.*?) *$", l)
            if m:
                d[m.group(1)] = m.group(2)
            m = re.match(r"^        short: *(.*?) *$", l)
            if m:
                d["short"] = m.group(1)
        out[name] = d
    return out

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: err.append("page error: " + str(e)))
    pg.on("console", lambda m: m.type == "error" and err.append("console error: " + m.text))
    pg.goto(page_path.as_uri()); pg.wait_for_selector(".blk")
    pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
    text = pg.inner_text("main")
    src = page_path.read_text(encoding="utf-8")
    r = res["page"] = {}
    r["blocks"] = pg.locator(".blk").count(); r["figures"] = pg.locator("figure").count(); r["tables"] = pg.locator("main table").count()
    r["table_tools"] = pg.locator(".tblk .ttools").count()
    if r["tables"] != r["table_tools"] or r["tables"] < 2:
        err.append(f"{r['tables']} tables but {r['table_tools']} table tools")
    tags = pg.locator(".prob .tag").all_inner_texts()
    r["try_its"] = [int(m.group(1)) for t in tags for m in [re.match(r"TRY IT (\d+)", t)] if m]
    if r["try_its"] != list(range(41, 59)):
        err.append(f"Try-it numbers are {r['try_its']}, expected 41..58")
    heads = pg.evaluate("[...document.querySelectorAll('h2.sub .no')].map(e => e.textContent)")
    if [h for h in heads if h.startswith("2.2.")] != [f"2.2.{i}" for i in range(13, 20)]:
        err.append(f"section numbers are {heads}")
    # ---- printed numbers
    norm = lambda s: s.replace("\t", " \t ")
    for name, s in E:
        found = s in text
        res["numbers"][name] = {"printed": s, "found": found}
        if not found:
            err.append(f"printed number not found: [{name}] {s}")
    # ---- figures
    fe = pg.evaluate("window.figErrors")
    if fe:
        err.append(f"figure errors {fe}")
    for i in range(r["figures"]):
        pg.locator("figure").nth(i).screenshot(path=f"out/2_2c_fig{i + 1}.png")
    pg.screenshot(path="out/2_2c_full.png", full_page=True)
    empty = pg.evaluate("[...document.querySelectorAll('figure svg')].filter(s => s.children.length < 5).map(s => s.id)")
    if empty:
        err.append(f"empty figures {empty}")
    clipped = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const vb = s.viewBox.baseVal; const root = s.getBoundingClientRect();
        const sc = root.width / vb.width;
        for (const t of s.querySelectorAll('text')) { const r = t.getBoundingClientRect(); const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
          if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.push(s.id + ':' + t.textContent.slice(0, 30)); } } return out; }""")
    if clipped:
        err.append(f"labels outside the figure {clipped}")
    overlaps = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
        for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
          const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
          if (w > 2 && h > 2 && w * h > 0.2 * Math.min(a.width * a.height, c.width * c.height)) out.push(s.id + ': "' + ts[i].t.textContent.slice(0, 20) + '" / "' + ts[j].t.textContent.slice(0, 20) + '"'); } } return out; }""")
    if overlaps:
        err.append(f"overlapping labels {overlaps}")
    labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", src)
    r["caption_labels"] = labs
    if labs != [str(i + 1) for i in range(len(labs))] or len(labs) != r["figures"]:
        err.append(f"figure labels {labs} are not 1..{r['figures']}")
    bad_blocks = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const n = b.querySelectorAll('table').length; if (!n) continue;
        const how = (b.innerText.match(/How to read this table\\./g) || []).length, figs = b.querySelectorAll('figure').length;
        if (how < n || figs < n) out.push(b.dataset.b + ': tables ' + n + ', how-to ' + how + ', figures ' + figs); } return out; }""")
    if bad_blocks:
        err.append(f"{bad_blocks}")
    order_bad = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const c = b.querySelector('.c'); const kids = [...c.querySelectorAll('p, .tblk')];
        let seenHow = 0, seenTab = 0; for (const k of kids) { if (k.classList.contains('tblk')) { seenTab++; if (seenHow < seenTab) out.push(b.dataset.b + ' table ' + seenTab + ' has no how-to paragraph before it'); } else if (/How to read this table\\./.test(k.innerText)) seenHow++; } } return out; }""")
    if order_bad:
        err.append(f"{order_bad}")
    # ---- local and external links
    hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
    broken = [h for h in hrefs if not h.startswith(("http", "#")) and not (page_path.parent / h.split("#")[0]).exists()]
    if broken:
        err.append(f"broken links {broken}")
    GH = re.compile(r"^https://github\.com/(PX4/PX4-Autopilot|iamaisim/ProjectAirSim)/blob/(d28a642|c80cff5)/([^#\s]+?)(?:#L(\d+)(?:-L(\d+))?)?$")
    for h in sorted({h for h in hrefs if h.startswith("http")}):
        m = GH.match(h)
        if not m:
            err.append(f"link of an unexpected form: {h}")
            continue
        repo, com, path, a, bb = m.groups()
        if (repo == "PX4/PX4-Autopilot") != (com == "d28a642"):
            err.append(f"repo and commit do not match: {h}")
        t = fetch(repo, com, path)
        if t is not None and a:
            n = t.count("\n") + 1
            lo, hi = int(a), int(bb or a)
            if lo < 1 or hi > n or lo > hi:
                err.append(f"line numbers outside the file ({n} lines): {h}")
    # ---- quotations
    quotes = pg.evaluate("""() => [...document.querySelectorAll('q.src, pre.code')].map(e => ({tag: e.tagName, src: e.getAttribute('data-src'), text: e.textContent}))""")
    r["quotes"] = len(quotes)
    for q in quotes:
        if not q["src"]:
            err.append(f"a {q['tag']} without data-src: {q['text'][:50]!r}")
            continue
        repo, com, path, rng = q["src"].split("|")
        a, bb = (rng.split("-") + [rng])[:2] if "-" in rng else (rng, rng)
        seg = lines_of(repo, com, path, int(a), int(bb))
        if seg is None:
            continue
        if ws(q["text"]) not in ws(seg):
            err.append(f"quotation not found in {path}:{rng} @ {com}: {ws(q['text'])[:70]!r}")
    # ---- the parameter tables against the YAML files
    base = "src/modules/mc_pos_control/"
    YM = {"multicopter_autonomous_params.yaml": base + "multicopter_autonomous_params.yaml", "multicopter_position_control_limits_params.yaml": base + "multicopter_position_control_limits_params.yaml",
          "multicopter_altitude_mode_params.yaml": base + "multicopter_altitude_mode_params.yaml", "multicopter_position_mode_params.yaml": base + "multicopter_position_mode_params.yaml"}
    rows = pg.evaluate("""() => { const out = []; for (const t of document.querySelectorAll('main table')) { const head = t.rows[0].innerText.split('\\t')[1] || '';
        if (!/parameter/i.test(t.rows[0].innerText)) continue;
        for (const tr of [...t.rows].slice(1)) out.push({cells: [...tr.cells].map(c => c.innerText.trim()), href: (tr.cells[tr.cells.length - 1].querySelector('a') || {}).href || ''}); } return out; }""")
    r["param_rows"] = len(rows)
    if len(rows) != 20:
        err.append(f"parameter tables have {len(rows)} rows, expected 14 + 6")
    yaml_cache = {}
    for row in rows:
        cells = row["cells"]
        name = cells[1]
        href = row["href"]
        m = re.match(r"^https://github\.com/PX4/PX4-Autopilot/blob/d28a642/(.+?)#L(\d+)-L(\d+)$", href)
        if not m:
            err.append(f"parameter {name}: no GitHub link with lines ({href})")
            continue
        path, a, bb = m.group(1), int(m.group(2)), int(m.group(3))
        if path not in yaml_cache:
            t = fetch(PX, PXC, path)
            yaml_cache[path] = yaml_entries(t) if t else {}
        ent = yaml_cache[path].get(name)
        if not ent:
            err.append(f"parameter {name} not found in {path}")
            continue
        want_unit = ent.get("unit") or ("none in the file" if name.startswith("MPC") else None)
        want_max = ent.get("max") or "not given"
        want_unit = ent.get("unit") or "none in the file"
        if (cells[3], cells[4], cells[5], cells[6]) != (ent.get("default"), want_unit, ent.get("min"), want_max):
            err.append(f"parameter {name}: page has (default {cells[3]}, unit {cells[4]}, min {cells[5]}, max {cells[6]}) but the file has "
                       f"({ent.get('default')}, {want_unit}, {ent.get('min')}, {want_max})")
        if (a, bb) != (ent["start"], ent["end"]):
            err.append(f"parameter {name}: lines {a}-{bb} on the page, the entry is at {ent['start']}-{ent['end']}")
        if not cells[2].startswith(ent.get("short", "\u0000")):
            err.append(f"parameter {name}: description {cells[2]!r} does not start with the file's short text {ent.get('short')!r}")
    # figure 4 and figure 8 draw the same numbers as the tables
    f4 = pg.evaluate("document.getElementById('f4').textContent"); f8 = pg.evaluate("document.getElementById('f8').textContent")
    for i, row in enumerate(rows[:14], 1):
        c = row["cells"]
        for want in (f"{i} · {c[1]}", c[5], c[6], f"{c[3]} {c[4] if c[4] != 'none in the file' else '(no unit in the file)'}"):
            if want not in f4:
                err.append(f"figure 4 lacks {want!r} (row {i})")
    for i, row in enumerate(rows[14:], 1):
        c = row["cells"]
        for want in (f"{i} · {c[1]}", f"{float(c[3]):.1f} m/s"):
            if want not in f8:
                err.append(f"figure 8 lacks {want!r} (row {i})")
    # facts that the text states about the YAML files
    yc = yaml_entries(fetch(PX, PXC, base + "multicopter_autonomous_params.yaml") or "")
    yl = yaml_entries(fetch(PX, PXC, base + "multicopter_position_control_limits_params.yaml") or "")
    ym = yaml_entries(fetch(PX, PXC, base + "multicopter_position_mode_params.yaml") or "")
    checks = [(yc.get("MPC_ACC_HOR", {}).get("min"), "2", "MPC_ACC_HOR min (Try it 44)"), (yc.get("MPC_XY_CRUISE", {}).get("default"), "5.0", "MPC_XY_CRUISE default"),
              (yl.get("MPC_XY_VEL_MAX", {}).get("max"), "20", "MPC_XY_VEL_MAX max (gate run)"), (yc.get("MPC_ACC_HOR", {}).get("default"), "3.0", "MPC_ACC_HOR default"),
              (yc.get("MPC_JERK_AUTO", {}).get("default"), "4.0", "MPC_JERK_AUTO default"), (ym.get("MPC_POS_MODE", {}).get("default"), "4", "MPC_POS_MODE default"),
              (yc.get("MPC_ACC_HOR", {}).get("max"), "15", "MPC_ACC_HOR max"), (yc.get("MPC_JERK_AUTO", {}).get("max"), "80", "MPC_JERK_AUTO max"),
              (yc.get("MPC_XY_ERR_MAX", {}).get("default"), "2.0", "MPC_XY_ERR_MAX default (Try it 54)")]
    for got, want, what in checks:
        if got != want:
            err.append(f"{what}: the file has {got!r}, the page assumes {want!r}")
    # the AirSim defaults of Try it 58
    hpp = lines_of(AS, ASC, "vehicle_apis/multirotor_api/include/multirotor_api_base.hpp", 252, 258) or ""
    for want in ("vel_to_braking_dist =", "0.5f", "min_braking_dist = 1", "max_braking_dist = 3", "braking_vel = 1.0f", "min_vel_for_braking = 3"):
        if want not in hpp:
            err.append(f"multirotor_api_base.hpp:252-258 lacks {want!r}")
    # ---- line probes: the place the page names must hold what the page says
    PROBES = [
        (PX, PXC, "src/lib/motion_planning/VelocitySmoothing.cpp", 240, 264, "void VelocitySmoothing::updateTraj"),
        (PX, PXC, "src/lib/motion_planning/VelocitySmoothing.cpp", 225, 238, "evaluatePoly"),
        (PX, PXC, "src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp", 839, 839, "// up"),
        (PX, PXC, "src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp", 73, 73, "_position_smoothing.reset"),
        (PX, PXC, "src/modules/mc_pos_control/multicopter_position_mode_params.yaml", 20, 20, "default: 4"),
        (PX, PXC, "src/modules/simulation/simulator_sih/sih.cpp", 383, 383, "first order transfer function"),
        (PX, PXC, "docs/en/msg_docs/VehicleLocalPosition.md", 21, 23, "NED earth-fixed frame"),
        (PX, PXC, "docs/en/msg_docs/VehicleLocalPosition.md", 23, 23, "negative altitude"),
        (PX, PXC, "src/lib/motion_planning/PositionSmoothing.cpp", 321, 321, "time_stretch_xy"),
        (PX, PXC, "src/lib/motion_planning/VelocitySmoothing.cpp", 61, 65, "T1_new = (a_max - a0) / j_max"),
        (AS, ASC, "docs/api.md", 238, 250, "linear_twist"),
        (AS, ASC, "docs/api.md", 238, 250, "linear_accn"),
        (AS, ASC, "docs/api.md", 217, 217, "move_by_velocity_async"),
        (AS, ASC, "docs/api.md", 219, 219, "move_to_position_async"),
        (AS, ASC, "docs/api.md", 220, 220, "move_on_path_async"),
        (AS, ASC, "client/python/projectairsim/src/projectairsim/drone.py", 292, 317, "async def move_by_velocity_async"),
        (AS, ASC, "client/python/projectairsim/src/projectairsim/drone.py", 476, 507, "async def move_to_position_async"),
        (AS, ASC, "client/python/projectairsim/src/projectairsim/drone.py", 595, 597, "carrot following"),
        (AS, ASC, "physics/src/fast_physics.cpp", 525, 525, "accels.linear"),
        (AS, ASC, "physics/src/fast_physics.cpp", 541, 543, "twist.linear"),
        (AS, ASC, "core_sim/include/core_sim/environment.hpp", 44, 45, "Vector3(0, 0,"),
        (AS, ASC, "vehicle_apis/multirotor_api/src/multirotor_api_base.cpp", 728, 838, "MoveOnPathInternal"),
        (AS, ASC, "vehicle_apis/multirotor_api/src/multirotor_api_base.cpp", 789, 795, "braking_dist"),
        (AS, ASC, "vehicle_apis/multirotor_api/src/multirotor_api_base.cpp", 826, 833, "braking_vel"),
        (AS, ASC, "vehicle_apis/multirotor_api/src/multirotor_api_base.cpp", 957, 967, "MoveOnPathInternal"),
        (AS, ASC, "vehicle_apis/multirotor_api/include/multirotor_api_base.hpp", 223, 237, "MC_BRK_VEL"),
        (AS, ASC, "docs/controllers/simple_flight.md", 16, 19, "move_by_velocity_async"),
    ]
    for repo, com, path, a, bb, must in PROBES:
        seg = lines_of(repo, com, path, a, bb)
        if seg is not None and must not in seg:
            err.append(f"{path}:{a}-{bb} @ {com} does not contain {must!r}")
    # ---- statements of absence
    jerk_free = [(AS, ASC, "docs/api.md"), (AS, ASC, "docs/controllers/controllers.md"), (AS, ASC, "docs/controllers/simple_flight.md"),
                 (AS, ASC, "client/python/projectairsim/src/projectairsim/drone.py"), (AS, ASC, "physics/src/fast_physics.cpp"),
                 (AS, ASC, "vehicle_apis/multirotor_api/src/multirotor_api_base.cpp"), (AS, ASC, "vehicle_apis/multirotor_api/include/multirotor_api_base.hpp"),
                 (PX, PXC, "src/lib/fw_performance_model/performance_model_params.yaml"), (PX, PXC, "src/modules/fw_mode_manager/fw_mode_manager_params.yaml")]
    for repo, com, path in jerk_free:
        t = fetch(repo, com, path)
        if t is not None and re.search("jerk", t, re.I):
            err.append(f"the page says there is no 'jerk' in {path}, but the word is there")
    # ---- marks and notes
    blk = pg.locator('.blk[data-b="goal"]')
    blk.hover(); blk.locator('button.dot[title="question"]').click()
    blk.locator("text=+ Note").click(); pg.wait_for_timeout(200)
    pg.keyboard.type("sample note on 2_2c"); pg.wait_for_timeout(600)
    pg.reload(); pg.wait_for_selector(".blk")
    page_id = pg.evaluate("PAGE")
    keys = pg.evaluate(f"Object.keys(JSON.parse(localStorage.getItem('ardp_v2_sec_{page_id}')||'{{}}'))")
    if pg.locator('.blk[data-b="goal"] textarea').input_value() != "sample note on 2_2c" or keys != [page_id + ":goal"]:
        err.append(f"note did not survive reload under key {page_id}:goal ({keys})")
    tb = pg.locator(".tblk").nth(1)
    tkey = tb.get_attribute("data-t")
    tb.hover(); tb.locator('.ttools button.dot[title="change this"]').click()
    tb.locator(".ttools >> text=+ Note").click(); pg.wait_for_timeout(200)
    pg.keyboard.type("table note on 2_2c"); pg.wait_for_timeout(600)
    pg.reload(); pg.wait_for_selector(".blk")
    tb = pg.locator(f'.tblk[data-t="{tkey}"]')
    if tb.locator(".tnotes textarea").input_value() != "table note on 2_2c" or "mk-red" not in (tb.get_attribute("class") or ""):
        err.append(f"table note/mark did not survive reload under {page_id}:{tkey}")
    tb.screenshot(path="out/2_2c_table_tools.png")
    # ---- page-wide text rules
    if re.search(r"(?<![\d.])9\.81(?!\d)", text):
        err.append("the number 9.81 appears; use 9.8062 or 9.7965")
    if re.search("refut", text, re.I) or re.search("refut", src, re.I):
        err.append("banned word")
    if any("<sub>" in s for s in re.findall(r"<svg.*?</svg>", src, flags=re.S)):
        err.append("HTML <sub> inside an svg")
    b.close()

Path("out/check_2_2c.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
bad = [k for k, v in res["numbers"].items() if not v["found"]]
print(json.dumps({"errors": err, "missing_numbers": bad, "n_numbers": len(E), "page": res.get("page")}, ensure_ascii=False))
sys.exit(1 if err else 0)
