"""Check section 2.2 (pages 2.2a and 2.2b): recompute every number independently from the page's own rounded inputs and
require it, formatted as printed, in the page text. Also: figures (labels 1..N in order, no figure error, one PNG each),
tables (a "How to read this table." paragraph and a figure in the same block, a marks/notes tool each), Try-it numbers 1..40 across
both pages, gravity (never 9.81), the banned word, code quotes against the reference data, GitHub links, and the notes/marks test.
Run on the cloud machine (never on the owner's PC):
python check_2_2.py page/samples/2.2a_Position_velocity_acceleration_on_a_line.html page/samples/2.2b_Three_dimensions_laps_falling_fixed_wing_simulators.html"""
import json
import math
import re
import sys
import urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright

page_paths = [Path(a).resolve() for a in sys.argv[1:]]
Path("out").mkdir(exist_ok=True)
sq = math.sqrt
M = "−"                                             # the minus sign printed on the pages
def fm(x, d=0):                                     # fixed decimals, true minus sign
    s = f"{x:.{d}f}"
    return (M + s[1:]) if s.startswith("-") and float(s) != 0 else s.lstrip("-")
def sg(x, d=1):                                     # explicit plus or minus
    s = fm(abs(x), d)
    return ("+" if x >= 0 else M) + s
def gg(x):                                          # shortest exact print (0.41, 4.0401, 2.25)
    s = f"{round(x, 9):.10g}"
    return (M + s[1:]) if s.startswith("-") else s
def th(n):                                          # thousands with a space
    return f"{n:,}".replace(",", " ")

g, gD = 9.8062, 9.7965                              # WGS84 values of 2.1.2: home field (45 deg N, 0 m) and Denver (39.74 deg N, 1600 m)
rho, rhoD = 1.225, 1.049                            # 2.1.5
E = []                                              # (name, printed string)
def ex(name, s):
    E.append((name, s))

# ================= page 2.2a =================
ex("A pos disp", f"40 − 10 = +{40 - 10} m")
d1 = abs(70 - 10) + abs(40 - 70)
ex("A pos dist", f"|70 − 10| + |40 − 70| = {abs(70-10)} + {abs(40-70)} = {d1} m")
ex("A try1 disp", f"25 − ({M}15) = +{25 + 15} m")
ex("A vel avg", f"(40 − 10) / 12 = 30 / 12 = {30 / 12:.1f} m/s")
ex("A vel speed", f"d / Δt = 90 / 12 = {90 / 12:.1f} m/s")
ex("A vel leg1", f"(70 − 10) / 6 = {(70 - 10) / 6:.0f} m/s")
ex("A vel leg2", f"(40 − 70) / 6 = {fm((40 - 70) / 6)} m/s")
for te in (4, 3, 2.5, 2.1):
    dx, dt = te * te - 4, te - 2
    ex(f"A secant dx {te}", f"{gg(te*te)} − 4 = {gg(dx)}")
    ex(f"A secant v {te}", f"{gg(dx)} / {gg(dt)} = {gg(dx / dt)}")
ex("A tangent", "v(2) = 4 m/s")
ex("A try3", f"(80 − 20) / 4 = {(80-20)/4:.0f} m/s")
ex("A try5 x", f"2.01² = {gg(2.01**2)} m")
ex("A try5 dx", f"{gg(2.01**2)} − 4 = {gg(2.01**2 - 4)} m")
ex("A try5 v", f"{gg(2.01**2 - 4)} / 0.01 = {gg((2.01**2 - 4) / 0.01)} m/s")
ex("A try6 v", f"108 / 3.6 = {108/3.6:.0f} m/s")
# ===== LADDER items of sections 2.2.1 and 2.2.2 (every printed number recomputed here from the inputs that the page prints) =====
pre_errors = []
def need(cond, msg):                                 # an independent consistency test of the exercise itself
    if not cond:
        pre_errors.append("ladder self-check failed: " + msg)
def pz(v):                                           # a number in a subtraction: negative numbers in brackets
    return f"({fm(v)})" if v < 0 else f"{v:g}"
def sgg(v):                                          # shortest print with an explicit plus
    return ("+" if v > 0 else "") + gg(v)
# ---- Try it 2: the flight log (speed meter 10 m/s steady, entries every 2 s)
T2, X2, sp2 = [0, 2, 4, 6, 8, 10], [20, 0, -20, -10, 10, 30], 10
steps2 = [abs(X2[i + 1] - X2[i]) for i in range(5)]
ex("A try2 dx", f"{X2[-1]} − {X2[0]} = +{X2[-1] - X2[0]} m")
ex("A try2 dlog", " + ".join(f"|{fm(X2[i + 1])} − {pz(X2[i])}|" for i in range(5)) + " = " + " + ".join(str(s) for s in steps2) + f" = {sum(steps2)} m")
d2 = sp2 * T2[-1]
ex("A try2 d", f"d = v·t = {sp2} × {T2[-1]} = {d2} m")
ex("A try2 gap", f"{d2} − {sum(steps2)} = {d2 - sum(steps2)} m")
ex("A try2 full", f"{sp2} × 2 = {sp2 * 2} m")
bad_step = [i for i in range(5) if steps2[i] != sp2 * 2]
need(bad_step == [2], f"the only short step is {bad_step}")
gap2 = X2[3] - X2[2]; b2 = (sp2 * 2 - gap2) / 2
ex("A try2 b", f"b + (b + {gap2}) = {sp2 * 2}, so b = {b2:.0f} m")
ex("A try2 turn", f"{fm(X2[2])} − {b2:.0f} = {fm(X2[2] - b2)} m")
ex("A try2 tt", f"{b2:.0f} / {sp2} = {b2 / sp2:.1f} s")
ex("A try2 chk", f"2 × {b2:.0f} = {2 * b2:.0f} m")
ex("A try2 chk2", f"{b2:.0f} + {b2 + gap2:.0f} = {sp2 * 2} m")
ex("A try2 chk3", f"{sum(steps2)} + {d2 - sum(steps2)} = {d2} m")
# the exercise is consistent: a path with the steady speed, one turn at -25 m and t = 4.5 s reproduces the whole log and the 100 m
def xpath(t):
    tt = b2 / sp2 + T2[2]
    return X2[2] - sp2 * (t - T2[2]) if t <= tt else X2[2] - b2 + sp2 * (t - tt)
need(all(abs(xpath(t) - x) < 1e-9 for t, x in zip(T2[2:], X2[2:])), "the hidden-turn path does not reproduce the log")
need(abs((b2 + (b2 + gap2)) - sp2 * 2) < 1e-9, "the turn step is not 20 m long")
# ---- Try it 2a: two origins (gates 60 m apart; start 10, turn 50, end 35 from gate A)
G2, s2a, t2a, e2a = 60, 10, 50, 35
l1, l2 = abs(t2a - s2a), abs(e2a - t2a)
ex("A try2a dx", f"{e2a} − {s2a} = +{e2a - s2a} m")
ex("A try2a legs", f"|{t2a} − {s2a}| = {l1} m")
ex("A try2a legs2", f"|{e2a} − {t2a}| = {l2} m")
ex("A try2a d", f"{l1} + {l2} = {l1 + l2} m")
ex("A try2a x1", f"{G2} − {s2a} = {G2 - s2a} m")
ex("A try2a x2", f"{G2} − {e2a} = {G2 - e2a} m")
ex("A try2a dx2", f"{G2 - e2a} − {G2 - s2a} = {fm((G2 - e2a) - (G2 - s2a))} m")
ex("A try2a turn", f"{G2} − {t2a} = {G2 - t2a} m")
ex("A try2a legs3", f"|{G2 - t2a} − {G2 - s2a}| = {abs((G2 - t2a) - (G2 - s2a))} m")
ex("A try2a legs4", f"|{G2 - e2a} − {G2 - t2a}| = {abs((G2 - e2a) - (G2 - t2a))} m")
ex("A try2a sign", f"{fm(-(e2a - s2a))} = −(+{e2a - s2a})")
ex("A try2a sum1", f"{s2a} + {G2 - s2a} = {G2} m")
ex("A try2a sum2", f"{e2a} + {G2 - e2a} = {G2} m")
ex("A try2a wi", f"{fm(-e2a)} − ({fm(-s2a)}) = {fm(-e2a + s2a)} m")
need(abs((G2 - e2a) - (G2 - s2a)) + 0 == e2a - s2a and abs((G2 - t2a) - (G2 - s2a)) == l1 and abs((G2 - e2a) - (G2 - t2a)) == l2, "frame change keeps the legs")
# ---- Try it 2b: where did the drone turn (x1 = 10, x2 = 40, d = 130, one turn, forward first)
x1b, x2b, db = 10, 40, 130
xtb = (db + x1b + x2b) / 2
ex("A try2b eq", f"2·xturn − {x1b} − {x2b} = {db}")
ex("A try2b sol", f"2·xturn = {db} + {x1b + x2b} = {db + x1b + x2b}, so xturn = {xtb:.0f} m")
ex("A try2b dx", f"{x2b} − {x1b} = +{x2b - x1b} m")
back2b = xtb - x2b
ex("A try2b ident", f"{db} = {x2b - x1b} + 2 × {back2b:.0f}")
two = [10, 75, 25, 40]
legs = [two[i + 1] - two[i] for i in range(3)]
need(sum(abs(l) for l in legs) == db and sum(legs) == x2b - x1b and -sum(l for l in legs if l < 0) == back2b, "the two-turn path")
ex("A try2b two", f"{legs[0]} + {-legs[1]} + {legs[2]} = {sum(abs(l) for l in legs)} m")
ex("A try2b two2", f"{legs[0]} − {-legs[1]} + {legs[2]} = +{sum(legs)} m")
ex("A try2b l1", f"{xtb:.0f} − {x1b} = {xtb - x1b:.0f} m")
ex("A try2b l2", f"{xtb:.0f} − {x2b} = {xtb - x2b:.0f} m")
ex("A try2b l3", f"{xtb - x1b:.0f} + {xtb - x2b:.0f} = {db} m")
ex("A try2b end", f"{x1b} + {xtb - x1b:.0f} − {xtb - x2b:.0f} = {x2b} m")
xwb = (x1b + x2b - db) / 2
ex("A try2b wi", f"{x1b + x2b} − 2·xturn = {db}, and xturn = {fm(xwb)} m")
ex("A try2b wi2", f"({x1b} − ({fm(xwb)})) + ({x2b} − ({fm(xwb)})) = {x1b - xwb:.0f} + {x2b - xwb:.0f} = {(x1b - xwb) + (x2b - xwb):.0f} m")
# ---- Try it 4: the shuttle graph
C4 = [(0, 20), (2, 50), (6, 20), (10, 40)]
v4 = [(C4[i + 1][1] - C4[i][1]) / (C4[i + 1][0] - C4[i][0]) for i in range(3)]
for i in range(3):
    (ta, xa), (tb, xb) = C4[i], C4[i + 1]
    ex(f"A try4 v{i + 1}", f"({xb} − {xa}) / ({tb} − {ta}) = {sgg(v4[i])} m/s")
ex("A try4 avg", f"({C4[-1][1]} − {C4[0][1]}) / {C4[-1][0]} = {sg((C4[-1][1] - C4[0][1]) / C4[-1][0], 1)} m/s")
d4 = sum(abs(C4[i + 1][1] - C4[i][1]) for i in range(3))
ex("A try4 d", f"|{C4[1][1]} − {C4[0][1]}| + |{C4[2][1]} − {C4[1][1]}| + |{C4[3][1]} − {C4[2][1]}| = {abs(C4[1][1]-C4[0][1])} + {abs(C4[2][1]-C4[1][1])} + {abs(C4[3][1]-C4[2][1])} = {d4} m")
ex("A try4 spd", f"{d4} / {C4[-1][0]} = {d4 / C4[-1][0]:.1f} m/s")
ex("A try4 mean", f"({gg(v4[0])} − {gg(-v4[1])} + {gg(v4[2])}) / 3 = {gg(sum(v4))} / 3 = {sum(v4) / 3:.2f} m/s")
dur4 = [C4[i + 1][0] - C4[i][0] for i in range(3)]
wt = [v4[i] * dur4[i] for i in range(3)]
ex("A try4 wmean", f"({gg(v4[0])} × {dur4[0]} − {gg(-v4[1])} × {dur4[1]} + {gg(v4[2])} × {dur4[2]}) / {C4[-1][0]} = ({gg(wt[0])} − {gg(-wt[1])} + {gg(wt[2])}) / {C4[-1][0]} = {sum(wt) / C4[-1][0]:.1f} m/s")
x4 = C4[1][1] + v4[1] * (4 - C4[1][0])
ex("A try4 x4", f"{C4[1][1]} + ({gg(v4[1])}) × (4 − {C4[1][0]}) = {C4[1][1]} − {gg(-v4[1] * (4 - C4[1][0]))} = {x4:.0f} m")
need(abs(sum(wt) / C4[-1][0] - (C4[-1][1] - C4[0][1]) / C4[-1][0]) < 1e-12, "weighted mean differs from displacement over time")
# ---- Try it 5a: the curved x(t)
X5 = lambda t: (t ** 3 - 12 * t ** 2 + 36 * t) / 2
V5 = lambda t: (3 * t ** 2 - 24 * t + 36) / 2
ex("A try5a x2", f"(2³ − 12 × 2² + 36 × 2) / 2 = ({2**3} − {12 * 2**2} + {36 * 2}) / 2 = {X5(2):.0f} m")
ex("A try5a x6", f"(6³ − 12 × 6² + 36 × 6) / 2 = ({6**3} − {12 * 6**2} + {36 * 6}) / 2 = {X5(6):.0f} m")
need(abs(V5(2)) < 1e-12 and abs(V5(6)) < 1e-12 and V5(4) == -6 and all(V5(t / 10) < 0 for t in range(21, 60)) and V5(1.99) > 0 and V5(6.01) > 0, "zero and negative velocity")
need(V5(7.5) > V5(1) > V5(5.5) > V5(3), "ranking D, A, C, B")
need(abs(X5(4) - 8) < 1e-12 and abs(X5(4) + V5(4) * (3.5 - 4) - 11) < 1e-12 and abs(X5(4) + V5(4) * (4.5 - 4) - 5) < 1e-12, "tangent through (3.5, 11) and (4.5, 5)")
ex("A try5a slope", f"({5} − {11}) / ({4.5} − {3.5}) = {fm((5 - 11) / (4.5 - 3.5), 0)} m/s")
ex("A try5a avg", f"(0 − 0) / 6 = 0 m/s")
need(X5(6) == 0 and X5(0) == 0, "x(6) = x(0) = 0")
ex("A try5a dist", f"{X5(2):.0f} + {X5(2):.0f} = {2 * X5(2):.0f} m")
sec = lambda t: (X5(t + 0.01) - X5(t)) / 0.01
ex("A try5a chk", "A " + f"{fm(sec(1), 1)} m/s, B {fm(sec(3), 1)} m/s, C {fm(sec(5.5), 1)} m/s, D {fm(sec(7.5), 1)} m/s and E {fm(sec(4), 1)} m/s")
# ---- Try it 5b: the noisy position log
sg_, a5, dt0 = 0.02, 4.0, 0.02
E5 = lambda dt: 2 * sg_ / dt + a5 * dt / 2
ex("A try5b bias", f"4 × 0.14 / 2 = {a5 * 0.14 / 2:.2f} m/s")
ex("A try5b noise", f"2 × 0.02 / 0.14 = {2 * sg_ / 0.14:.3f} m/s")
ex("A try5b e1", f"2 × 0.02 / 0.02 + 4 × 0.02 / 2 = {2 * sg_ / 0.02:.2f} + {a5 * 0.02 / 2:.2f} = {E5(0.02):.2f} m/s")
ex("A try5b e50", f"2 × 0.02 / 1.00 + 4 × 1.00 / 2 = {2 * sg_ / 1.0:.2f} + {a5 * 1.0 / 2:.2f} = {E5(1.0):.2f} m/s")
dstar = math.sqrt(4 * sg_ / a5)
ex("A try5b dstar", f"Δt² = 4σ / a = 4 × 0.02 / 4 = {4 * sg_ / a5:.2f}, so Δt = {dstar:.3f} s")
ex("A try5b emin", f"2 × √(0.04 × 2) = 2 × √{0.04 * 2:.2f} = {2 * math.sqrt(0.04 * 2):.3f} m/s")
ex("A try5b steps", f"{round(dstar, 3)} / 0.02 = {round(dstar, 3) / 0.02:.2f}")
for n in (6, 7, 8):
    dtn = n * 0.02
    ex(f"A try5b n{n}", f"2 × 0.02 / {dtn:.2f} + 4 × {dtn:.2f} / 2 = {round(2 * sg_ / dtn, 3):.3f} + {round(a5 * dtn / 2, 3):.3f} = {round(2 * sg_ / dtn, 3) + round(a5 * dtn / 2, 3):.3f} m/s")
need(min(range(1, 51), key=lambda n: E5(n * 0.02)) == 7, "best number of steps is 7")
ex("A try5b chk1", f"2 × 0.02 / 0.141 = {2 * sg_ / 0.141:.3f} m/s")
ex("A try5b chk2", f"4 × 0.141 / 2 = {a5 * 0.141 / 2:.3f} m/s")
ex("A try5b ratio", f"2.04 / 0.566 = {2.04 / 0.566:.1f}")
ex("A try5b c1", f"0.02 / 0.14 = {sg_ / 0.14:.3f} m/s")
ex("A try5b c50", f"0.02 / 1.00 = {sg_ / 1.0:.3f} m/s")
# the exercise itself: the worst case of the forward difference equals E, and the central difference has no bias
xt5 = lambda t, v0=1.5, x0=3.0: x0 + v0 * t + 0.5 * a5 * t * t
vt5 = lambda t, v0=1.5: v0 + a5 * t
for n_ in (1, 7, 50):
    dt_ = n_ * 0.02; t_ = 0.6
    worst = ((xt5(t_ + dt_) + sg_) - (xt5(t_) - sg_)) / dt_ - vt5(t_)
    need(abs(worst - E5(dt_)) < 1e-9, f"worst case differs from E for {n_} steps")
    cen = (xt5(t_ + dt_) - xt5(t_ - dt_)) / (2 * dt_)
    need(abs(cen - vt5(t_)) < 1e-9, "central difference is not exact")
# ---- Try it 6: the spec sheet and the gates
vs6, dx6, tg6 = 108 / 3.6, 45, 1.62
va6 = round(dx6 / tg6, 2)
ex("A try6 vavg", f"{dx6} / {tg6} = {va6:.2f} m/s")
ex("A try6 kmh", f"{va6:.2f} × 3.6 = {va6 * 3.6:.1f} km/h")
ex("A try6 pct", f"({vs6:.0f} − {va6:.2f}) / {vs6:.0f} = {(vs6 - va6) / vs6:.3f}")
ex("A try6 t30", f"{dx6} / {vs6:.0f} = {dx6 / vs6:.2f} s")
ex("A try6 late", f"{tg6} − {dx6 / vs6:.2f} = {tg6 - dx6 / vs6:.2f} s")
ex("A try6 p1", f"{vs6:.0f} × 1.20 = {vs6 * 1.2:.0f} m")
ex("A try6 p2", f"{dx6} − {vs6 * 1.2:.0f} = {dx6 - vs6 * 1.2:.0f} m")
ex("A try6 p3", f"{tg6} − 1.20 = {tg6 - 1.2:.2f} s")
ex("A try6 p4", f"{dx6 - vs6 * 1.2:.0f} / {tg6 - 1.2:.2f} = {(dx6 - vs6 * 1.2) / (tg6 - 1.2):.2f} m/s")
ex("A try6 p5", f"1.20 + {tg6 - 1.2:.2f} = {tg6:.2f} s")
ex("A try6 p6", f"{vs6 * 1.2:.0f} + {dx6 - vs6 * 1.2:.0f} = {dx6} m")
need(abs(1.2 + (dx6 - vs6 * 1.2) / ((dx6 - vs6 * 1.2) / (tg6 - 1.2)) - tg6) < 1e-9 and dx6 / vs6 < tg6, "gate story")
for (v1, v2, dt) in ((2, 6, 2), (10, 4, 3), (-2, -6, 2), (-6, -2, 2)):
    a = (v2 - v1) / dt
    a_s = f"({fm(v2)} − ({fm(v1)})) / {dt}" if v1 < 0 else f"({fm(v2)} − {fm(v1)}) / {dt}"
    ex(f"A sign {v1}->{v2}", f"a = {a_s} = {sg(a)} m/s²")
ex("A try7", f"(25 − 0) / 4.0 = {25/4.0:.2f} m/s²")
ex("A try8", f"(0 − 20) / 5 = {fm((0-20)/5, 1)} m/s²")
ex("A try9", f"{M}3 + 1 × 2 = {fm(-3 + 1*2)} m/s")
ex("A graph a", f"a = (10 − 0) / 5 = {10/5:.0f} m/s²")
ex("A graph dv", f"Δv = 2 × 5 = {2*5} m/s")
ex("A graph dx", f"Δx = ½ × 5 × 10 = {0.5*5*10:.0f} m")
ex("A graph v5", f"slope of the tangent at t = 5 s: v = {2*5} m/s")
ex("A try10 a", f"(12 − 0) / 4 = {12/4:.1f} m/s²")
ex("A try10 dx1", f"½ × 4 × 12 = {0.5*4*12:.0f} m")
ex("A try10 dx2", f"12 × 5 = {12*5} m")
ex("A try10 dx", f"24 + 60 = {24+60} m")
ex("A try11 dv1", f"3 × 2 = +{3*2} m/s")
ex("A try11 v1", f"5 + 6 = {5+6} m/s")
ex("A try11 dv2", f"{M}1 × 4 = {fm(-1*4)} m/s")
ex("A try11 v2", f"11 + ({fm(-4)}) = {11-4} m/s")
v0_, a_, t_ = 4, 2, 3
v_e = v0_ + a_ * t_; dx_e = v0_ * t_ + 0.5 * a_ * t_ ** 2
ex("A eq1", f"v = 4 + 2 × 3 = {v_e} m/s")
ex("A eq2", f"x − x₀ = 4 × 3 + ½ × 2 × 3² = {v0_*t_} + {0.5*a_*t_**2:.0f} = {dx_e:.0f} m")
ex("A eq3", f"v² = 4² + 2 × 2 × 21 = {v0_**2} + {2*a_*dx_e:.0f} = {v0_**2 + 2*a_*dx_e:.0f}, so v = {sq(v0_**2 + 2*a_*dx_e):.0f} m/s")
ex("A eq4", f"x − x₀ = ½ × (4 + 10) × 3 = {0.5*(v0_+v_e)*t_:.0f} m")
ex("A try12", f"0 + 4.0 × 2.5 = {4.0*2.5:.1f} m/s")
ex("A try13", f"0 × 3.0 + ½ × 5.0 × 3.0² = 0 + {0.5*5.0*9.0:.1f} = {0.5*5.0*9.0:.1f} m")
ex("A try14 d", f"(24² − 6²) / (2 × 6) = (576 − 36) / 12 = {(576-36)/12:.1f} m")
ex("A try14 t", f"(24 − 6) / 6 = {(24-6)/6:.1f} s")
ex("A try14 chk", f"½ × (6 + 24) × 3.0 = {0.5*30*3.0:.1f} m")
ex("A try15 a", f"2 × 30 / 4² = 60 / 16 = {60/16:.2f} m/s²")
ex("A try15 v", f"0 + 3.75 × 4 = {3.75*4:.0f} m/s")
ex("A try15 chk", f"½ × (0 + 15) × 4 = {0.5*15*4:.0f} m")
ex("A wrong true v", f"v = ½ × 4 × 4 = {0.5*4*4:.0f} m/s")
ex("A wrong true x", f"x = 64 / 6 = {64/6:.2f} m")
ex("A wrong last v", f"v = a·t = 4 × 4 = {16} m/s")
ex("A wrong last x", f"x = ½·a·t² = ½ × 4 × 4² = {0.5*4*16:.0f} m")
ex("A wrong mean v", f"v = a·t = 2 × 4 = {8} m/s")
ex("A wrong mean x", f"x = ½·a·t² = ½ × 2 × 4² = {0.5*2*16:.0f} m")
ex("A wrong pct", f"wrong by {round((16 - 64/6) / (64/6) * 100)} %")
ex("A try17", f"0.5 + 1.5 + 2.5 + 3.5 = {0.5+1.5+2.5+3.5:.1f} m/s")
# the gate run
t1 = (24.0 - 6.0) / 6.0; x1 = 0.5 * (6.0 + 24.0) * t1
t3 = (8.0 - 24.0) / (-8.0); x3 = 0.5 * (24.0 + 8.0) * t3
x2 = 100 - x1 - x3; t2 = x2 / 24.0; tt = t1 + round(t2, 3) + t3
ex("A run t1", f"(24.0 − 6.0) / 6.0 = {t1:.1f}")
ex("A run x1", f"½ × (6.0 + 24.0) × 3.0 = {x1:.1f}")
ex("A run x2", f"100 − 45.0 − 32.0 = {x2:.1f}")
ex("A run t2", f"23.0 / 24.0 = {t2:.3f}")
ex("A run t3", f"(8.0 − 24.0) / ({M}8.0) = {t3:.1f}")
ex("A run x3", f"½ × (24.0 + 8.0) × 2.0 = {x3:.1f}")
ex("A run tt", f"3.0 + 0.958 + 2.0 = {tt:.3f}")
ex("A run vavg", f"100 / 5.958 = {100/5.958:.2f} m/s")
ex("A run sum", f"45.0 + 23.0 + 32.0 = {x1+x2+x3:.1f}")
for v0b, in ((12,), (24,), (36,)):
    ex(f"A brake d {v0b}", f"{v0b}² / (2 × 8.0) = {v0b**2} / 16 = {v0b**2/16:.1f}")
    ex(f"A brake t {v0b}", f"{v0b} / 8.0 = {v0b/8.0:.1f}")
ex("A try18 d", f"30² / (2 × 10) = 900 / 20 = {900/20:.0f} m")
ex("A try18 t", f"30 / 10 = {30/10:.1f} s")
ex("A try19 a", f"(8² − 24²) / (2 × 40) = (64 − 576) / 80 = {fm((64-576)/80, 1)} m/s²")
ex("A try19 t", f"(8 − 24) / ({M}6.4) = {(8-24)/(-6.4):.1f} s")
ex("A try19 chk", f"½ × (24 + 8) × 2.5 = {0.5*32*2.5:.0f} m")
ex("A try20 t", f"√( 2 × 40 / 5.0 ) = √{2*40/5.0:.0f} = {sq(2*40/5.0):.1f} s")
ex("A try20 v", f"0 + 5.0 × 4.0 = {5.0*4.0:.0f} m/s")
ex("A try21 v2", f"20² + 2 × ({M}8.0) × 30 = 400 − 480 = {fm(400-480)}")
ex("A try21 d", f"400 / 16 = {400/16:.0f} m")
ex("A try22 t", f"3.0 + 2.0 = {3.0+2.0:.1f} s")
ex("A try22 x", f"45.0 + 32.0 = {45.0+32.0:.1f} m")
ex("A try23 avg", f"½ × (6.0 + 8.0) = {0.5*(6.0+8.0):.1f} m/s")
ex("A try23 ph1", f"½ × (6.0 + 24.0) = {0.5*30:.0f} m/s")
ex("A check1", "Δx = +30 m and d = 70 m")
ex("A check4", "36 m and 144 m")

# ================= page 2.2b =================
# 3D example
p0, v0, a3, tt3 = (0, 0, 30), (12, 5, -3), (1.0, 0.5, -0.5), 4.0
rows3 = []
for i in range(3):
    v = v0[i] + a3[i] * tt3; p = p0[i] + v0[i] * tt3 + 0.5 * a3[i] * tt3 ** 2
    rows3.append((v, p))
ex("B 3d vx", f"12 + 1.0 × 4.0 = {rows3[0][0]:.0f}")
ex("B 3d vy", f"5 + 0.5 × 4.0 = {rows3[1][0]:.0f}")
ex("B 3d vz", f"{M}3 + ({M}0.5) × 4.0 = {fm(rows3[2][0])}")
ex("B 3d px", f"0 + 12 × 4.0 + ½ × 1.0 × 4.0² = {12*4.0:.0f} + {0.5*1.0*16:.0f} = {rows3[0][1]:.0f}")
ex("B 3d py", f"0 + 5 × 4.0 + ½ × 0.5 × 4.0² = {5*4.0:.0f} + {0.5*0.5*16:.0f} = {rows3[1][1]:.0f}")
ex("B 3d pz", f"30 + ({M}3) × 4.0 + ½ × ({M}0.5) × 4.0² = 30 − {3*4.0:.0f} − {0.5*0.5*16:.0f} = {rows3[2][1]:.0f}")
s0 = sum(c * c for c in v0); s4 = sum(r[0] ** 2 for r in rows3)
ex("B 3d speed0", f"√(144 + 25 + 9) = √{s0} = {sq(s0):.2f} m/s")
ex("B 3d speed4", f"√(256 + 49 + 25) = √{s4:.0f} = {sq(s4):.2f} m/s")
dp = (rows3[0][1] - 0, rows3[1][1] - 0, rows3[2][1] - 30)
sd = sum(c * c for c in dp)
ex("B 3d disp", f"(56 − 0, 24 − 0, 14 − 30) = ({dp[0]:.0f}, {dp[1]:.0f}, {fm(dp[2])})")
ex("B 3d dlen", f"√(3136 + 576 + 256) = √{sd:.0f} = {sq(sd):.2f} m")
ex("B 3d alen", f"√(1.0² + 0.5² + ({M}0.5)²) = √{1.0+0.25+0.25} = {sq(1.5):.2f} m/s²")
ex("B try24", f"√(3² + 4² + 0²) = √{9+16} = {sq(25):.0f} m/s")
v2 = [v0[i] + a3[i] * 2.0 for i in range(3)]; s2 = sum(c * c for c in v2)
ex("B try25 v", f"(12 + 1.0 × 2.0, 5 + 0.5 × 2.0, {M}3 + ({M}0.5) × 2.0) = ({v2[0]:.0f}, {v2[1]:.0f}, {fm(v2[2])}) m/s")
ex("B try25 speed", f"√(14² + 6² + ({M}4)²) = √(196 + 36 + 16) = √{s2:.0f} = {sq(s2):.2f} m/s")
ex("B try25 x", f"0 + 12 × 2.0 + ½ × 1.0 × 2.0² = {24} + {2} = 26 m")
ex("B try25 y", f"0 + 5 × 2.0 + ½ × 0.5 × 2.0² = {10} + {1} = 11 m")
ex("B try25 z", f"30 + ({M}3) × 2.0 + ½ × ({M}0.5) × 2.0² = 30 − {6} − {1} = 23 m")
ex("B try26 t", f"(17 − 5) / 2 = {(17-5)/2:.1f} s")
ex("B try26 x", f"0 + 10 × 6.0 = {10*6.0:.0f} m")
# lap
lap = [(+5.0, 0, 20, None), (0.0, 20, 20, 3.0), (-4.0, 20, 8, None), (0.0, 8, 8, 5.0), (+6.0, 8, 20, None)]
s_end = 0; t_end = 0
for k, (a, va, vb, tgiven) in enumerate(lap, 1):
    t = tgiven if tgiven is not None else (vb - va) / a
    ds = 0.5 * (va + vb) * t if a != 0 else va * t
    s_end += ds; t_end += t
    if a != 0:
        ex(f"B lap t{k}", f"({vb} − {va}) / {fm(a,1) if a>0 else '(' + fm(a,1) + ')'} = {t:.1f}")
    if a != 0:
        ex(f"B lap ds{k}", f"½ × ({va} + {vb}) × {t:.1f} = {ds:.1f}")
    else:
        ex(f"B lap ds{k}", f"{va} × {t:.1f} = {ds:.1f}")
ex("B lap tot", f"4.0 + 3.0 + 3.0 + 5.0 + 2.0 = {t_end:.1f}")
ex("B lap stot", f"s_total = {s_end:.1f}".replace("s_total", "stotal"))
ex("B lap avg", f"210.0 / 17.0 = {s_end/t_end:.2f} m/s")
ex("B try27", f"4.0 + 3.0 = {4.0+3.0:.1f} s")
ex("B try28", f"(8 − 20) / (15.0 − 4.0) = −12 / 11 = {fm(-12/11, 2)} m/s²")
ex("B try29", f"(20² − 8²) / (2 × 6.0) = (400 − 64) / 12 = {(400-64)/12:.1f} m")
# free fall
for h in (5, 10, 20, 50):
    arg = 2 * h / g; tf = round(sq(arg), 3); v = g * tf
    ex(f"B fall t {h}", f"√(2 × {h} / 9.8062) = √{arg:.4f} = {tf:.3f}")
    ex(f"B fall v {h}", f"9.8062 × {tf:.3f} = {v:.2f}")
arg30 = 90 / g; t30 = round(sq(arg30), 3)
ex("B try30 t", f"√(2 × 45 / 9.8062) = √{arg30:.4f} = {t30:.3f} s")
ex("B try30 v", f"9.8062 × {t30:.3f} = {g*t30:.2f} m/s")
ex("B try30 kmh", f"about {round(g*t30*3.6)} km/h")
arg31 = 90 / gD; t31 = round(sq(arg31), 3)
ex("B try31 t", f"√(2 × 45 / 9.7965) = √{arg31:.4f} = {t31:.3f} s")
ex("B try31 v", f"9.7965 × {t31:.3f} = {gD*t31:.2f} m/s")
ex("B try31 cmp", f"against {g*t30:.2f} m/s at the home field")
ex("B try32 h", f"15² / (2 × 9.8062) = 225 / {2*g:.4f} = {225/(2*g):.2f} m")
tu = round(15 / g, 4)
ex("B try32 tu", f"15 / 9.8062 = {tu:.4f} s")
ex("B try32 t", f"2 × {tu:.4f} = {2*tu:.3f} s")
v33sq = 6 ** 2 + 2 * g * 30
ex("B try33 v2", f"6² + 2 × ({M}9.8062) × ({M}30) = 36 + {2*g*30:.2f} = {v33sq:.2f}")
ex("B try33 v", f"{sq(v33sq):.2f} m/s")
v33 = round(sq(v33sq), 2)
ex("B try33 t", f"({M}{v33:.2f} − 6) / ({M}9.8062) = {(v33 + 6) / g:.3f} s")
# fixed wing
ex("B fw row1 t", f"25.0 / 2.5 = {25.0/2.5:.1f}")
ex("B fw row1 s", f"25.0² / (2 × 2.5) = 625 / 5.0 = {625/5.0:.1f}")
ex("B fw row2 t", f"22.0 / 2.0 = {22.0/2.0:.1f}")
ex("B fw row2 s", f"22.0² / (2 × 2.0) = 484 / 4.0 = {484/4.0:.1f}")
gr, rr = round(gD / g, 4), round(rho / rhoD, 4); prod = round(gr * rr, 4); sr = round(sq(prod), 4); VD = round(25.0 * sr, 1)
ex("B fw ratio", f"25.0 × √({gr:.4f} × {rr:.4f}) = 25.0 × √{prod:.4f} = 25.0 × {sr:.4f} = {VD:.1f} m/s")
aD = round(2.5 * rhoD / rho, 3)
ex("B fw aD", f"2.5 × 1.049 / 1.225 = {aD:.3f} m/s²")
ex("B fw row3 t", f"{VD:.1f} / 2.5 = {VD/2.5:.1f}")
ex("B fw row3 s", f"{VD:.1f}² / (2 × 2.5) = {round(VD**2):.0f} / 5.0 = {VD**2/5.0:.1f}")
ex("B fw row4 t", f"{VD:.1f} / {aD:.3f} = {VD/aD:.1f}")
ex("B fw row4 s", f"{VD:.1f}² / (2 × {aD:.3f}) = {round(VD**2):.0f} / {2*aD:.3f} = {VD**2/(2*aD):.1f}")
ex("B try34 s", f"20² / (2 × 1.6) = 400 / 3.2 = {400/3.2:.1f} m")
ex("B try34 t", f"20 / 1.6 = {20/1.6:.1f} s")
ex("B try35 a", f"−18² / (2 × 100) = −324 / 200 = {fm(-324/200, 2)} m/s²")
ex("B try35 t", f"18 / 1.62 = {18/1.62:.1f} s")
ex("B try37 a", f"27.0² / (2 × 150) = 729 / 300 = {729/300:.2f} m/s²")
# Try it 36 (rewritten 2026-10-20 after the owner's red mark): slowest flight speed, m 2.0 kg, S 0.40 m², C_L,max 1.2
GH, GD = 9.8062, 9.7965                             # the same WGS84 values as g, gD above, kept apart in case g is reused
q36 = 1.225 * 0.40 * 1.2
W36h, W36d = 2.0 * GH, 2.0 * GD
ex("B try36 W", f"W = m·g = 2.0 × {GH} = {W36h:.4f} N")
ex("B try36 vmin", f"√(2 × {W36h:.4f} / (1.225 × 0.40 × 1.2)) = √({2*W36h:.4f} / {q36:.3f}) = √{2*W36h/q36:.2f} = {math.sqrt(2*W36h/q36):.2f} m/s")
q36d = 1.049 * 0.40 * 1.2
ex("B try36 Wd", f"W = m·g = 2.0 × {GD} = {W36d:.3f} N")
ex("B try36 vmind", f"√(2 × {W36d:.3f} / (1.049 × 0.40 × 1.2)) = √({2*W36d:.3f} / {q36d:.5f}) = √{2*W36d/q36d:.2f} = {math.sqrt(2*W36d/q36d):.2f} m/s")
vh, vd = round(math.sqrt(2*W36h/q36), 2), round(math.sqrt(2*W36d/q36d), 2)
ex("B try36 ratio", f"{vd:.2f} / {vh:.2f} = {vd/vh:.3f}")
ex("B try36 whatif", f"√(2 × 2.5 × {GH} / {q36:.3f}) = √({5*GH:.3f} / {q36:.3f}) = √{5*GH/q36:.2f} = {math.sqrt(5*GH/q36):.2f} m/s")
ex("B try36 sqrt", f"√1.25 = {math.sqrt(1.25):.3f}")
# Try it 37 (rewritten): still air, headwind, tailwind
a37 = 2.141
ex("B try37 s", f"27.0² / (2 × {a37}) = 729 / {2*a37:.3f} = {729/(2*a37):.1f} m")
ex("B try37 over", f"{729/(2*a37):.1f} − 150 = {round(729/(2*a37),1)-150:.1f} m")
vg = math.sqrt(2*a37*150)
ex("B try37 vg", f"√(2 × {a37} × 150) = √{2*a37*150:.1f} = {vg:.2f} m/s")
ex("B try37 w", f"27.0 − {vg:.2f} = {27.0-round(vg,2):.2f} m/s")
ex("B try37 tail", f"29.0² / (2 × {a37}) = 841 / {2*a37:.3f} = {841/(2*a37):.1f} m")
ex("B try37 check", f"{round(vg,2):.2f}² / {2*a37:.3f} = {round(vg,2)**2:.1f} / {2*a37:.3f} = {round(round(vg,2)**2,1)/(2*a37):.2f} m")
ex("B try37 head2", f"25.0² / (2 × {a37}) = 625 / {2*a37:.3f} = {625/(2*a37):.1f} m")
ex("B try37 diff", f"{841/(2*a37):.1f} − {625/(2*a37):.1f} = {round(841/(2*a37),1)-round(625/(2*a37),1):.1f} m")
# simulator
steps_exact = [(0.5 * 2 * t * t) for t in (0, 0.5, 1.0, 1.5, 2.0)]
x_air = [0]; x_sih = [0]; x_semi = [0]; x_euler = [0]; v = 0.0; dt = 0.5
for k in range(4):
    xa = x_air[-1] + (v + 0.5 * 2 * dt) * dt
    vn = v + 2 * dt
    xs = x_sih[-1] + 0.5 * (v + vn) * dt
    xm = x_semi[-1] + vn * dt
    xe = x_euler[-1] + v * dt
    x_air.append(xa); x_sih.append(xs); x_semi.append(xm); x_euler.append(xe); v = vn
for i in range(5):
    t = i * dt
    ex(f"B sim row {i}", "\t".join([f"{t:.1f}", f"{steps_exact[i]:.2f}", f"{x_air[i]:.2f}", f"{x_sih[i]:.2f}", f"{x_semi[i]:.2f}", f"{x_euler[i]:.2f}"]))
ex("B sim err", "\t".join(["error at 2.0 s", "", f"{x_air[-1]-4:.2f}", f"{x_sih[-1]-4:.2f}", f"+{x_semi[-1]-4:.2f}", f"{M}{abs(x_euler[-1]-4):.2f}"]))
ex("B sim err formula", f"±½·a·Δt·t = ½ × 2 × 0.5 × 2.0 = {0.5*2*0.5*2.0:.1f} m")
ex("B sim step1 r1", f"0 + (0 + ½ × 2 × 0.5) × 0.5 = {(0 + 0.5*2*0.5)*0.5:.2f} m")
ex("B sim step1 r2", f"0 + ½ × (0 + 1) × 0.5 = {0.5*1*0.5:.2f} m")
ex("B sim step1 r3", f"0 + 1 × 0.5 = {1*0.5:.2f} m")
ex("B sim sih dt", f"1/250 = {1/250:.3f} s")
x38 = 0.25 * sum(0.5 * k for k in range(8))
ex("B try38 x", f"0.25 × (0 + 0.5 + 1.0 + 1.5 + 2.0 + 2.5 + 3.0 + 3.5) = 0.25 × {sum(0.5*k for k in range(8)):.0f} = {x38:.1f} m")
ex("B try38 err", f"3.5 − 4.0 = {fm(x38 - 4.0, 1)} m")
ex("B try38 formula", f"½ × 2 × 0.25 × 2.0 = {0.5*2*0.25*2.0:.1f} m")
# ramp a = t, dt = 1
va = 0.0; vs = 0.0; air = []; sih = []
for k in range(4):
    va = va + 0.5 * (k + (k + 1)) * 1; vs = vs + k * 1
    air.append(va); sih.append(vs)
ex("B try39 air", f"v₁ = 0 + ½ × (0 + 1) × 1 = {air[0]:.1f}")
ex("B try39 air4", f"v₄ = {air[2]:.1f} + ½ × (3 + 4) × 1 = {air[3]:.1f} m/s")
ex("B try39 sih4", f"v₄ = {sih[2]:.0f} + 3 × 1 = {sih[3]:.0f} m/s")
for lab, dtv in (("a", 0.001), ("b", 1 / 250), ("c", 0.003), ("d", 0.020)):
    n = 17 / dtv
    ex(f"B try40 {lab}", th(round(n)))
ex("B try40 c", f"N = 17 / 0.003 = {th(int(17/0.003))}.{round(17/0.003*10) % 10}")
# problems
ta, xa_ = 16 / 4.0, 0.5 * 16 * 4.0; tb, xb_ = (12 - 16) / (-4.0), 0.5 * (16 + 12) * 1.0; xc = 80 - xa_ - xb_; tc = xc / 16
ex("B P1 acc", f"t = (v − v₀) / a = (16 − 0) / 4.0 = {ta:.1f} s")
ex("B P1 acc x", f"½ × (0 + 16) × 4.0 = {xa_:.1f} m")
ex("B P1 brk", f"t = (12 − 16) / ({M}4.0) = {tb:.1f} s")
ex("B P1 brk x", f"½ × (16 + 12) × 1.0 = {xb_:.1f} m")
ex("B P1 cruise", f"80 − 32.0 − 14.0 = {xc:.1f} m")
ex("B P1 tc", f"34.0 / 16 = {tc:.3f} s")
ex("B P1 tot", f"4.0 + 2.125 + 1.0 = {ta+tc+tb:.3f} s")
ex("B P1 avg", f"80 / 7.125 = {80/7.125:.2f} m/s")
ex("B P2 out", f"15 − 5 = 10 m/s")
ex("B P2 t out", f"90 / 10 = {90/10:.1f} s")
ex("B P2 t back", f"90 / 20 = {90/20:.1f} s")
ex("B P2 tot", f"9.0 + 4.5 = {9.0+4.5:.1f} s")
ex("B P2 nowind", f"2 × 90 / 15 = {2*90/15:.1f} s")
ex("B P2 gain", "costs 1.5 s")
t3a = sq(2 * 30 / 7.34)
ex("B P3 arg", f"√( 2 × 30 / 7.34 ) = √{2*30/7.34:.4f} = {round(t3a,3):.3f} s")
ex("B P3 v", f"7.34 × {round(t3a,3):.3f} = {7.34*round(t3a,3):.2f} m/s")
W0 = 1.5 * g; aE = round((60 - W0) / 1.5, 2)
ex("B P3 W", f"W = 1.5 × 9.8062 = {W0:.3f} N".replace("W = ", "") if False else f"1.5 × 9.8062 = {W0:.3f} N")
ex("B P3 aE", f"(60 − 14.709) / 1.5 = {aE:.2f} m/s²")
ex("B P3 tE", f"√(2 × 30 / {aE:.2f}) = √{60/aE:.4f} = {sq(60/aE):.3f} s")
ex("B P4 a", f"(6² − 30²) / (2 × 50) = (36 − 900) / 100 = {fm((36-900)/100, 2)} m/s²")
ex("B P4 t", f"(6 − 30) / ({M}8.64) = {(6-30)/(-8.64):.2f} s")
ex("B P4 v2", f"30² + 2 × ({M}8.0) × 50 = 900 − 800 = {900-800}")
ex("B P4 x", f"(36 − 900) / (2 × ({M}8.0)) = {(36-900)/(2*-8.0):.1f} m")
ex("B P4 earlier", f"54.0 − 50 = {54.0-50:.1f} m")
ex("B P5 a", f"a = V² / (2·s) = 27.0² / (2 × 160) = 729 / 320 = {729/320:.2f} m/s²".replace("a = ", "") if False else f"27.0² / (2 × 160) = 729 / 320 = {729/320:.2f} m/s²")
tfall = lambda h, gg_: sq(2 * h / gg_)
t6 = round(tfall(80, g), 3)
ex("B P6 t", f"√(2 × 80 / 9.8062) = √{160/g:.4f} = {t6:.3f} s")
ex("B P6 x", f"20 × {t6:.3f} = {20*t6:.2f} m")
vz = round(g * t6, 2)
ex("B P6 vz", f"{M}9.8062 × {t6:.3f} = {M}{vz:.2f} m/s")
ex("B P6 v", f"√(20² + {vz:.2f}²) = √{400 + vz**2:.2f} = {sq(400 + vz**2):.2f} m/s")
ex("B quiz 3", f"9.8062 × 3.0 = {g*3:.2f} m/s")
ex("B quiz 3b", f"½ × 9.8062 × 3.0² = {0.5*g*9:.2f} m")

# ===== run in the browser =====
res = {"errors": [], "pages": {}, "numbers": {}}
text_by_page = {}
tags_seen = []
banned = re.compile(r"refut", re.I)
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    for page_path in page_paths:
        tag = page_path.name.split("_")[0].replace(".", "_")           # 2_2a, 2_2b
        r = res["pages"][page_path.name] = {}
        pg.goto(page_path.as_uri())
        pg.wait_for_selector(".blk")
        pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
        text = pg.inner_text("main")
        text_by_page[tag] = text
        r["blocks"] = pg.locator(".blk").count()
        r["figures"] = pg.locator("figure").count()
        tags = pg.locator(".prob .tag").all_inner_texts()
        r["try_its"] = [m.group(1) + m.group(2) for t in tags for m in [re.match(r"TRY IT (\d+)([a-z]?)\b", t)] if m]         # "7", "2a"
        # ladder checks (sections 2.2.1 and 2.2.2 of page 2.2a): the level in every tag, a figure in every medium and hard item,
        # every solution step of a medium or hard item starts with "Why this step:"
        if page_path.name.startswith("2.2a"):
            lv = {m.group(1) + m.group(2): m.group(3) for t in tags for m in [re.match(r"TRY IT (\d+)([a-z]?) · (WARM-UP|MEDIUM|HARD) ·", t)] if m}
            r["levels"] = lv
            want = {"1": "WARM-UP", "2": "MEDIUM", "2a": "MEDIUM", "2b": "HARD", "3": "WARM-UP", "4": "MEDIUM", "5": "WARM-UP", "5a": "MEDIUM", "5b": "HARD", "6": "MEDIUM"}
            if lv != want:
                res["errors"].append(f"{page_path.name}: levels {lv} differ from the plan {want}")
            think1 = [t for t in tags if t.startswith("THINK 1 ")]
            if think1 != ["THINK 1 · HARD · TWO LEGS, TWO SPEEDS"]:
                res["errors"].append(f"{page_path.name}: tag of Think 1 is {think1}")
            lad = pg.evaluate("""() => { const out = []; for (const p of document.querySelectorAll('.prob.lv')) { const tag = p.querySelector('.tag').innerText; const m = tag.match(/TRY IT (\\d+[a-z]?) · (WARM-UP|MEDIUM|HARD)/); if (!m || m[2] === 'WARM-UP') continue;
                const id = m[1]; const nfig = p.querySelectorAll('figure svg').length; const lis = [...p.querySelectorAll(':scope > details > ol > li')];
                const notwhy = lis.filter(l => !l.innerText.trim().startsWith('Why this step:')).length;
                out.push({id, level: m[2], figures: nfig, steps: lis.length, steps_without_why: notwhy}); } return out; }""")
            r["ladder_items"] = lad
            for it in lad:
                if it["figures"] < 1 or it["steps"] < 3 or it["steps_without_why"]:
                    res["errors"].append(f"{page_path.name}: ladder item {it}")
            if sorted(it["id"] for it in lad) != ["2", "2a", "2b", "4", "5a", "5b", "6"]:
                res["errors"].append(f"{page_path.name}: medium/hard items are {[it['id'] for it in lad]}")
        r["problems"] = [int(m.group(1)) for t in tags for m in [re.match(r"PROBLEM (\d+)", t)] if m]
        r["tables"] = pg.locator("main table").count()
        r["table_tools"] = pg.locator(".tblk .ttools").count()
        if r["tables"] != r["table_tools"]:
            res["errors"].append(f"{page_path.name}: {r['tables']} tables but {r['table_tools']} table tools")
        # every figure runs without error and is photographed
        fe = pg.evaluate("window.figErrors")
        if fe:
            res["errors"].append(f"{page_path.name}: figure errors {fe}")
        for i in range(r["figures"]):
            pg.locator("figure").nth(i).screenshot(path=f"out/{tag}_fig{i}.png")
        pg.screenshot(path=f"out/{tag}_full.png", full_page=True)
        # an svg with nothing drawn would be an empty figure
        empty = pg.evaluate("[...document.querySelectorAll('figure svg')].filter(s => s.children.length < 5).map(s => s.id)")
        if empty:
            res["errors"].append(f"{page_path.name}: empty figures {empty}")
        # text that leaves the svg viewBox (clipped labels)
        clipped = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const vb = s.viewBox.baseVal; const root = s.getBoundingClientRect();
            const sc = root.width / vb.width;
            for (const t of s.querySelectorAll('text')) { const r = t.getBoundingClientRect(); const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
              if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.push(s.id + ':' + t.textContent.slice(0, 30)); } } return out; }""")
        r["clipped_labels"] = clipped
        if clipped:
            res["errors"].append(f"{page_path.name}: labels outside the figure {clipped}")
        # overlapping labels inside one figure: two text boxes that overlap by more than a third of the smaller one
        overlaps = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
            for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
              const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
              if (w > 2 && h > 2 && w * h > 0.33 * Math.min(a.width * a.height, c.width * c.height)) out.push(s.id + ': "' + ts[i].t.textContent.slice(0, 20) + '" / "' + ts[j].t.textContent.slice(0, 20) + '"'); } } return out; }""")
        r["overlapping_labels"] = overlaps
        if overlaps:
            res["errors"].append(f"{page_path.name}: overlapping labels {overlaps}")
        # figure caption labels run 1..N in order
        src = page_path.read_text(encoding="utf-8")
        labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", src)
        r["caption_labels"] = labs
        if labs != [str(i + 1) for i in range(len(labs))] or len(labs) != r["figures"]:
            res["errors"].append(f"{page_path.name}: figure labels {labs} are not 1..{r['figures']}")
        if re.search(r"Figures? \d+ and \d+", text) and "Figures are numbered" not in text:
            res["errors"].append(f"{page_path.name}: a 'Figures N and M' list must be checked by hand")
        # blocks with tables: one 'How to read this table.' paragraph per table, and at least one figure
        bad_blocks = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const n = b.querySelectorAll('table').length; if (!n) continue;
            const how = (b.innerText.match(/How to read this table\\./g) || []).length, figs = b.querySelectorAll('figure').length;
            if (how < n || figs < n) out.push(b.dataset.b + ': tables ' + n + ', how-to ' + how + ', figures ' + figs); } return out; }""")
        r["blocks_missing_howto_or_figure"] = bad_blocks
        if bad_blocks:
            res["errors"].append(f"{page_path.name}: {bad_blocks}")
        # the how-to paragraph must come before its table
        order_bad = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const c = b.querySelector('.c'); const kids = [...c.querySelectorAll('p, .tblk')];
            let seenHow = 0, seenTab = 0; for (const k of kids) { if (k.classList.contains('tblk')) { seenTab++; if (seenHow < seenTab) out.push(b.dataset.b + ' table ' + seenTab + ' has no how-to paragraph before it'); } else if (/How to read this table\\./.test(k.innerText)) seenHow++; } } return out; }""")
        if order_bad:
            res["errors"].append(f"{page_path.name}: {order_bad}")
        # local links
        hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
        r["broken_links"] = [h for h in hrefs if not h.startswith(("http", "#")) and "autopilot_map" not in h and not (page_path.parent / h.split("#")[0]).exists()]
        r["autopilot_map_links"] = sorted({h for h in hrefs if "autopilot_map" in h})
        if r["broken_links"]:
            res["errors"].append(f"{page_path.name}: broken links {r['broken_links']}")
        r["external_links"] = sorted({h for h in hrefs if h.startswith("http")})
        # marks and notes
        blk = pg.locator('.blk[data-b="goal"]')
        blk.hover(); blk.locator('button.dot[title="question"]').click()
        blk.locator("text=+ Note").click(); pg.wait_for_timeout(200)
        pg.keyboard.type("sample note on " + tag); pg.wait_for_timeout(600)
        pg.reload(); pg.wait_for_selector(".blk")
        r["note_after_reload"] = pg.locator('.blk[data-b="goal"] textarea').input_value()
        page_id = pg.evaluate("PAGE")
        r["ls_keys"] = pg.evaluate(f"Object.keys(JSON.parse(localStorage.getItem('ardp_v2_sec_{page_id}')||'{{}}'))")
        if r["note_after_reload"] != "sample note on " + tag or r["ls_keys"] != [page_id + ":goal"]:
            res["errors"].append(f"{page_path.name}: note did not survive reload under key {page_id}:goal")
        tb = pg.locator(".tblk").nth(1)
        tkey = page_id + ":" + tb.get_attribute("data-t")
        tb.hover(); tb.locator('.ttools button.dot[title="change this"]').click()
        tb.locator(".ttools >> text=+ Note").click(); pg.wait_for_timeout(200)
        pg.keyboard.type("table note on " + tag); pg.wait_for_timeout(600)
        pg.reload(); pg.wait_for_selector(".blk")
        tb = pg.locator(f'.tblk[data-t="{tkey.split(":", 1)[1]}"]')
        r["table_note_after_reload"] = tb.locator(".tnotes textarea").input_value()
        if r["table_note_after_reload"] != "table note on " + tag or "mk-red" not in (tb.get_attribute("class") or ""):
            res["errors"].append(f"{page_path.name}: table note/mark did not survive reload under {tkey}")
        tb.screenshot(path=f"out/{tag}_table_tools.png")
        # page-wide checks on the text
        if re.search(r"(?<![\d.])9\.81(?!\d)", text):
            res["errors"].append(f"{page_path.name}: the number 9.81 appears; use 9.8062 or 9.7965")
        if banned.search(text) or banned.search(src):
            res["errors"].append(f"{page_path.name}: banned word")
        if "<sub>" in re.sub(r"<svg.*?</svg>", "", src, flags=re.S) and False:
            pass
        svg_html = re.findall(r"<svg.*?</svg>", src, flags=re.S)
        if any("<sub>" in s for s in svg_html):
            res["errors"].append(f"{page_path.name}: HTML <sub> inside an svg")
    b.close()

alltext = "\n".join(text_by_page.values())
for tag, t in text_by_page.items():
    pass
# Try-it numbers run 1..40 across both pages, problems 1..6
allt_all = [n for p in res["pages"].values() for n in p["try_its"]]
allt = sorted(int(n) for n in allt_all if n.isdigit())
if allt != list(range(1, 41)):
    res["errors"].append(f"Try-it numbers are {allt}, expected 1..40")
if sorted(n for n in allt_all if not n.isdigit()) != ["2a", "2b", "5a", "5b"]:
    res["errors"].append(f"suffixed Try-it tags are {sorted(n for n in allt_all if not n.isdigit())}, expected 2a, 2b, 5a, 5b")
res["errors"] += pre_errors
allp = sorted(n for p in res["pages"].values() for n in p["problems"])
if allp != list(range(1, 7)):
    res["errors"].append(f"Problem numbers are {allp}, expected 1..6")

# printed numbers: 2.2a numbers on page a, 2.2b numbers on page b
ta_ = next(v for k, v in text_by_page.items() if k.endswith("2a")); tb_ = next(v for k, v in text_by_page.items() if k.endswith("2b"))
norm = lambda s: " ".join(s.replace("\t", " \t ").split()) if "\t" not in s else s
for name, s in E:
    t = ta_ if name.startswith("A ") else tb_
    found = s in t
    if not found and "\t" in s:                                  # table rows: cells are tab separated in the page text
        found = s in t.replace(" \t", "\t").replace("\t ", "\t")
    res["numbers"][name] = {"printed": s, "found": found}
bad = [k for k, v in res["numbers"].items() if not v["found"]]
res["missing_numbers"] = bad
res["expected_not_found"] = {k: res["numbers"][k]["printed"] for k in bad}

# code quotes: every <pre class="code"> must be, word for word (spaces ignored), the "code" field of a reference row
ref = {}
for name in ("gazebo", "airsim", "px4sim", "px4ap"):
    f = Path(page_paths[0]).parent.parent / "reference" / f"sim_formulas_{name}.js"
    txt = f.read_text(encoding="utf-8").strip()
    for row in json.loads(re.match(r"^window\.SIM_FORMULAS_[A-Z0-9]+\s*=\s*(\[.*\])\s*;?\s*$", txt, re.S).group(1)):
        ref[row["id"]] = row
ws = lambda s: " ".join(s.split())
need = {"PA-8-01", "PA-10-02", "PA-10-03", "g10-sih-integration", "GZ-64", "px4ap-013"}
res["code_quotes"] = {}
with sync_playwright() as p:
    b = p.chromium.launch(); pg = b.new_page()
    pg.goto(page_paths[-1].as_uri()); pg.wait_for_selector(".blk")
    pres = [ws(x) for x in pg.locator("pre.code").all_inner_texts()]
    links = pg.evaluate("[...document.querySelectorAll('a[href^=\"https://github.com\"]')].map(a => a.href)")
    b.close()
for i in need:
    res["code_quotes"][i] = ws(ref[i]["code"]) in pres
    if not res["code_quotes"][i]:
        res["errors"].append(f"code of reference row {i} is not quoted word for word")
for q in pres:
    if q not in {ws(r["code"]) for r in ref.values()}:
        res["errors"].append(f"a code block is not in the reference data: {q[:60]}")
def ghlink(r):
    m = re.search(r"(\d+)(?:\s*[-–]\s*(\d+))?", str(r["lines"])); a = "#L" + m.group(1) + ("-L" + m.group(2) if m.group(2) else "") if m else ""
    return f"https://github.com/{r['repo']}/blob/{r['commit']}/{r['file']}{a}"
exp_links = {ghlink(ref[i]) for i in need - {"px4ap-013"}} | {"https://github.com/PX4/PX4-Autopilot/blob/d28a642/src/modules/ekf2/EKF/ekf.cpp#L264-L276"}
res["github_links_missing"] = sorted(exp_links - set(links))
if res["github_links_missing"]:
    res["errors"].append(f"github links not on the page: {res['github_links_missing']}")
# every external link on both pages: GitHub links must answer 200 (the others are only recorded)
res["links"] = {}
for r in res["pages"].values():
    for u in r["external_links"]:
        try:
            req = urllib.request.Request(u, headers={"User-Agent": "Mozilla/5.0"})
            res["links"][u] = urllib.request.urlopen(req, timeout=30).status
        except Exception as e:
            res["links"][u] = str(e)[:80]
for u, s in res["links"].items():
    if "github.com" in u and s != 200:
        res["errors"].append(f"GitHub link does not answer 200: {u} -> {s}")

Path("out/check_2_2.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"missing": bad, "errors": res["errors"], "expected_not_found": res["expected_not_found"],
                  "pages": {k: {kk: vv for kk, vv in v.items() if kk in ("blocks", "figures", "try_its", "problems", "tables")} for k, v in res["pages"].items()}}, ensure_ascii=False))
sys.exit(1 if bad or res["errors"] else 0)
