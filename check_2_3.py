"""Check page 2.3 (Dynamics: Newton's second law): recompute every number printed on the page independently from the page's own
rounded inputs and require it, formatted as printed, in the rendered page text (figure-only numbers: in the text of that figure's svg).
Also: 31 figures (ids, captions "Figure 1." .. "Figure 31." in page order, no figure error, no svg text outside its viewBox, no two text
boxes overlapping, one PNG per figure), structure (TRY IT 1..17, PROBLEM 1..5, THINK 1..3, ladder levels, "Why this step:", every
Try it / Figure reference points to an item), tables (a "How to read this table." paragraph before each table, a figure after it that the
paragraph names, a marks/notes tool each), the six code quotes against the reference data, GitHub links, the References block, the bans
(9.81, "refut"), and a negative control (one mutated number must be reported missing).
Run on the cloud machine only (never on the owner's PC):
python check_2_3.py page/samples/2.3_Dynamics_Newtons_second_law.html"""
import json
import math
import re
import sys
import urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright

page_path = Path(sys.argv[1]).resolve()
Path("out").mkdir(exist_ok=True)
Path("out/fig_2_3").mkdir(exist_ok=True)
sq = math.sqrt
M = "−"                                             # the minus sign printed on the page


def fm(x, d=0):                                     # fixed decimals, true minus sign
    s = f"{x:.{d}f}"
    return (M + s[1:]) if s.startswith("-") and float(s) != 0 else s.lstrip("-")


def th(x, d=1):                                     # thousands with a space: 1 198.8
    return f"{x:,.{d}f}".replace(",", " ")


pre_errors = []

# ---- gravity: the WGS84 values of 2.1.2 (the same formulas as check_2_1.py) ----
A, INV_F, GM, OMEGA = 6378137.0, 298.257223563, 3.986004418e14, 7.292115e-5
B, E2, GE, GP = 6356752.3142, 0.00669437999014, 9.7803253359, 9.8321849378
K, M_SPIN = 0.00193185265241, 0.00344978650684
FL_ = 1 / INV_F


def g_full(phi, h):
    s = math.sin(phi) ** 2
    gp = GE * (1 + K * s) / math.sqrt(1 - E2 * s)
    return gp * (1 - (2 / A) * (1 + FL_ + M_SPIN - 2 * FL_ * s) * h + (3 / A ** 2) * h ** 2)


g, gD = 9.8062, 9.7965
if round(g_full(math.radians(45), 0), 4) != g or round(g_full(math.radians(39.74), 1600), 4) != gD:
    pre_errors.append("WGS84 values of g do not give 9.8062 and 9.7965")
rho, rhoD = 1.225, 1.049

E = []                                              # (name, printed string)
FL = {}                                             # svg id -> labels that the figure code must draw (exact text of a text element)


def ex(name, s):
    E.append((name, s))


def lab(fid, *items):
    FL.setdefault(fid, []).extend(items)


Wr = round(1.2 * g, 3)                              # 11.767, the weight of 1.2 kg at the home field
Wd = round(1.2 * gD, 3)                             # 11.756

# ================= 2.3.1 =================
ex("case a", f"a = 0 / 1.2 = {fm(0 / 1.2, 2)} m/s²")
ex("case b", f"ΣF = +3.6 N, a = 3.6 / 1.2 = +{fm(3.6 / 1.2, 2)} m/s²")
ex("case c", f"|a| = 3.6 / 1.2 = {fm(3.6 / 1.2, 2)} m/s² at a right angle to v")
ex("t1 a", f"a = ΣF / m = 3.6 / 0.8 = {fm(3.6 / 0.8, 2)} m/s²")
ex("t1 b", f"a = ΣF / m = 3.6 / 1.6 = {fm(3.6 / 1.6, 2)} m/s²")
# Try it 2
F2 = sq(3.0 ** 2 + 4.0 ** 2)
a2 = F2 / 2.0
th2 = math.degrees(math.atan(4.0 / 3.0))
ex("t2 F", f"ΣF = √(3.0² + 4.0²) = √{3.0 ** 2 + 4.0 ** 2:.2f} = {F2:.2f} N")
ex("t2 a", f"a = ΣF / m = {F2:.2f} / 2.0 = {a2:.2f} m/s²")
ex("t2 th", f"θ = atan(Fy / Fx) = atan(4.0 / 3.0) = {th2:.1f}°")
ex("t2 ax", f"ax = 3.0 / 2.0 = {3.0 / 2.0:.2f} m/s²")
ex("t2 ay", f"ay = 4.0 / 2.0 = {4.0 / 2.0:.2f} m/s²")
ex("t2 chk", f"√(1.50² + 2.00²) = √{1.5 ** 2 + 2.0 ** 2:.2f} = {sq(1.5 ** 2 + 2.0 ** 2):.2f} m/s²")
ex("t2 sum", f"shorter than their sum {3.0 + 4.0:.1f}")
lab("g3", "3.0 N, east", "4.0 N, north", f"ΣF = {F2:.2f} N", f"{th2:.1f}°", f"ax = {3.0 / 2.0:.2f} m/s²", f"ay = {4.0 / 2.0:.2f} m/s²", f"a = {a2:.2f} m/s²")
# Try it 3
a3 = 6.0 / 1.2
dv3 = a3 * 0.50
q1 = 10.0 + dv3
q2 = q1 + dv3
p1 = sq(10.0 ** 2 + dv3 ** 2)
p2 = sq(10.0 ** 2 + (2 * dv3) ** 2)
an1 = math.degrees(math.atan(dv3 / 10.0))
an2 = math.degrees(math.atan(2 * dv3 / 10.0))
ex("t3 a", f"a = ΣF / m = 6.0 / 1.2 = {a3:.2f} m/s²")
ex("t3 dv", f"Δv = a·Δt = {a3:.2f} × 0.50 = {dv3:.2f} m/s")
ex("t3 q1", f"10.0 + {dv3:.2f} = {q1:.1f} m/s")
ex("t3 q2", f"{q1:.1f} + {dv3:.2f} = {q2:.1f} m/s")
ex("t3 p1", f"√(10.0² + {dv3:.2f}²) = √{10.0 ** 2 + dv3 ** 2:.2f} = {p1:.2f} m/s")
ex("t3 an1", f"atan({dv3:.2f} / 10.0) = {an1:.1f}°")
ex("t3 p2", f"√(10.0² + {2 * dv3:.2f}²) = √{10.0 ** 2 + (2 * dv3) ** 2:.2f} = {p2:.2f} m/s")
ex("t3 an2", f"atan({2 * dv3:.2f} / 10.0) = {an2:.1f}°")
ex("t3 gain", f"case Q gained {dv3:.2f} m/s and case P only {p1 - 10.0:.2f} m/s")
ex("t3 chk1", f"shorter than 10.0 + {dv3:.2f} = {10.0 + dv3:.1f}: {p1:.2f} is")
ex("t3 chk2", f"{p2:.2f} is shorter than 10.0 + {2 * dv3:.2f} = {10.0 + 2 * dv3:.1f}")
lab("g4", "10.0 m/s", f"10.0 + {dv3:.2f} = {q1:.1f} m/s", f"{q1:.1f} + {dv3:.2f} = {q2:.1f} m/s", f"still 1: 10.0 m/s at {0:.0f}°",
    f"still 2: {p1:.2f} m/s at {an1:.1f}°", f"still 3: {p2:.2f} m/s at {an2:.1f}°", f"Δv = {dv3:.2f} m/s, the same in every step")

# ================= 2.3.2 =================
ex("fbd W", f"W = m·g = 1.2 × 9.8062 = {1.2 * g:.3f} N")
Tup, Tdn = Wr + 2.0, Wr - 2.0
ex("t4 a0", "a = ΣF / m = 0 / 1.2 = 0.00 m/s²")
ex("t4 up", f"T = W + D = {Wr:.3f} + 2.0 = {Tup:.3f} N")
ex("t4 dn", f"T = W − D = {Wr:.3f} − 2.0 = {Tdn:.3f} N")
ex("t4 diff", f"{Tup:.3f} − {Tdn:.3f} = {Tup - Tdn:.3f} N")
ex("t4 chk1", f"{Tup:.3f} = {Wr:.3f} + 2.0")
ex("t4 chk2", f"{Tdn:.3f} + 2.0 = {Wr:.3f}")
ex("t4 wi up", f"T = {Wr:.3f} + 3.0 = {Wr + 3.0:.3f} N")
ex("t4 wi dn", f"T = {Wr:.3f} − 3.0 = {Wr - 3.0:.3f} N")
ex("t4 wi diff", f"differ by {(Wr + 3.0) - (Wr - 3.0):.1f} N")
lab("g5", f"T = {Tup:.3f} N", f"W = {Wr:.3f} N", "D = 2.0 N", f"ΣF = T − W − D = {Tup:.3f} − {Wr:.3f} − 2.0 = 0, so a = 0 and the speed stays steady")
lab("g6", f"T = {Tup:.3f} N", f"W = {Wr:.3f} N", "D = 2.0 N", f"T = {Tdn:.3f} N", "v = 3 m/s up", "v = 3 m/s down",
    f"ΣF = {Tup:.3f} − {Wr:.3f} − 2.0 = 0", f"ΣF = {Tdn:.3f} + 2.0 − {Wr:.3f} = 0")
# Try it 5
D5 = 0.05 * 4.0 ** 2
sF5 = 10.4 - Wr - D5
a5 = round(sF5 / 1.2, 2)
sF50 = 10.4 - Wr
a50 = round(sF50 / 1.2, 2)
ex("t5 D", f"D = k·v² = 0.05 × 4.0² = {D5:.2f} N")
ex("t5 sF", f"ΣF = T − W − D = 10.4 − {Wr:.3f} − {D5:.2f} = {fm(sF5, 3)} N")
ex("t5 a", f"a = ΣF / m = {fm(sF5, 3)} / 1.2 = {fm(a5, 2)} m/s²")
ex("t5 sF0", f"ΣF = 10.4 − {Wr:.3f} = {fm(sF50, 3)} N")
ex("t5 a0", f"a = {fm(sF50, 3)} / 1.2 = {fm(a50, 2)} m/s²")
t51, t52 = 4.0 / abs(a5), 4.0 / abs(a50)
ex("t5 t1", f"t = v / |a| = 4.0 / {abs(a5):.2f} = {t51:.2f} s")
ex("t5 t2", f"t = 4.0 / {abs(a50):.2f} = {t52:.2f} s")
ex("t5 between", f"between {t51:.2f} s and {t52:.2f} s")
ex("t5 chk", f"{abs(a5):.2f} is more than {abs(a50):.2f}")
ex("t5 wi sF", f"ΣF = {Wr:.3f} − {Wr:.3f} − {D5:.2f} = {fm(-D5, 2)} N")
ex("t5 wi a", f"a = {fm(-D5, 2)} / 1.2 = {fm(-D5 / 1.2, 2)} m/s²")
lab("g7", "T = 10.4 N", f"W = {Wr:.3f} N", f"D = {D5:.2f} N", f"a = {fm(a5, 2)} m/s²", f"ΣF = 10.4 − {Wr:.3f} − {D5:.2f} = {fm(sF5, 3)} N",
    f"a = {fm(a50, 2)} m/s²", f"ΣF = 10.4 − {Wr:.3f} = {fm(sF50, 3)} N", "v = 4.0 m/s", "v = 0")

# ================= 2.3.3 =================
Wrough = round(1.2 * 9.8, 3)
ex("g row1", f"W = m·g = 1.2 × 9.8062 = {Wr:.3f} N")
ex("g row2", f"W = m·g = 1.2 × 9.7965 = {Wd:.3f} N")
ex("g row3", f"W = m·g = 1.2 × 9.8 = {Wrough:.3f} N")
ex("g d1", f"{Wr:.3f} − {Wrough:.3f} = {Wr - Wrough:.3f} N")
ex("g d2", f"{Wr:.3f} − {Wd:.3f} = {Wr - Wd:.3f} N")
ex("g span", f"only {9.810 - 9.790:.2f} m/s², about {(0.02 / 9.8) * 100:.1f} % of g")
lab("g8", "9.790", "9.795", "9.800", "9.805", "9.810", "Denver", "g = 9.7965 m/s²", f"W = {Wd:.3f} N", "rough value", "g = 9.8 m/s²", f"W = {Wrough:.3f} N",
    "home field", "g = 9.8062 m/s²", f"W = {Wr:.3f} N")
ex("t6 W", f"W = m·g = 0.8 × 9.8062 = {0.8 * g:.3f} N")
# Try it 7
gr = Wd / g
gram = round(gr * 1000, 1)
ex("t7 F", f"F = m·g = 1.2 × 9.7965 = {Wd:.3f} N")
ex("t7 read", f"reading = F / g(cal) = {Wd:.3f} / 9.8062 = {gr:.4f} kg = {th(gram)} g")
ex("t7 diff", f"1 200 − {th(gram)} = {1200 - gram:.1f} g")
ex("t7 chk1", f"{gD:.4f} / {g:.4f} = {gD / g:.4f}")
ex("t7 chk2", f"1 200 × {gD / g:.4f} = {th(round(1200 * round(gD / g, 4), 1))}")
lab("g9", "1 198", "1 199", "1 200", "1 201", "Denver reading", f"{th(gram)} g", "Home-field reading (set)", "1 200 g", "1.2 g lower in Denver, same mass")
# Try it 8
N1, N2 = round(1.2 * (g + 1.5), 3), round(1.2 * (g - 1.5), 3)
ex("t8 N1", f"N = m·(g + a) = 1.2 × (9.8062 + 1.5) = 1.2 × {g + 1.5:.4f} = {N1:.3f} N")
ex("t8 N2", f"N = m·(g + a) = 1.2 × (9.8062 − 1.5) = 1.2 × {g - 1.5:.4f} = {N2:.3f} N")
ex("t8 W", f"W = m·g = 1.2 × 9.8062 = {1.2 * g:.3f} N")
ex("t8 chk1", f"{N1:.3f} − {Wr:.3f} = {N1 - Wr:.3f} N")
ex("t8 chk2", f"{N1 - Wr:.3f} / 1.2 = {(N1 - Wr) / 1.2:.2f} m/s²")
ex("t8 chk3", f"{N2:.3f} − {Wr:.3f} = {fm(N2 - Wr, 3)} N")
ex("t8 wi", f"N = m·g = {Wr:.3f} N")
lab("g10", f"N = {N1:.3f} N", f"W = {Wr:.3f} N", f"N = {N2:.3f} N", "a = +1.5 m/s²", f"a = {M}1.5 m/s²", "a = −g", f"ΣF = {N1:.3f} − {Wr:.3f} = +{N1 - Wr:.3f} N",
    f"ΣF = {N2:.3f} − {Wr:.3f} = {fm(N2 - Wr, 3)} N", f"ΣF = −W = {M}{Wr:.3f} N", "no scale force: N = 0", f"{M}{g:.4f} m/s²")

# ================= 2.3.4 =================
for i, tw in enumerate((1.0, 2.0, 4.0)):
    ex(f"twr T {tw}", f"{tw:.1f} × {Wr:.3f} = {tw * Wr:.3f} N")
    ex(f"twr u {tw}", f"1 / {tw:.1f} = {1 / tw:.2f}")
    ex(f"twr a {tw}", f"({tw:.1f} − 1) × 9.8062 = {(tw - 1) * g:.3f} m/s²")
ex("t9", f"TWR = Tmax / W = 30.0 / {Wr:.3f} = {30.0 / Wr:.2f}")
# Try it 10
W17 = round(1.7 * g, 3)
mlim = 30.0 / (2.0 * g)
ex("t10 m", f"m = 1.2 + 0.50 = {1.2 + 0.5:.1f} kg")
ex("t10 W", f"W = m·g = 1.7 × 9.8062 = {1.7 * g:.3f} N")
ex("t10 TWR", f"TWR = Tmax / W = 30.0 / {W17:.3f} = {30.0 / W17:.2f}")
ex("t10 mlim", f"m = Tmax / (2.0 × g) = 30.0 / (2.0 × 9.8062) = 30.0 / {2 * g:.3f} = {mlim:.2f} kg")
ex("t10 pay", f"{round(mlim, 2):.2f} − 1.2 = {round(mlim, 2) - 1.2:.2f} kg")
ex("t10 den", f"m = 30.0 / (2.0 × 9.7965) = 30.0 / {2 * gD:.3f} = {30.0 / (2 * gD):.2f} kg")
ex("t10 chk1", f"W = {round(mlim, 2):.2f} × 9.8062 = {round(mlim, 2) * g:.3f} N")
ex("t10 chk2", f"30.0 / {round(round(mlim, 2) * g, 3):.3f} = {30.0 / round(round(mlim, 2) * g, 3):.2f}")
ex("t10 wi", f"m = 30.0 / (3.0 × 9.8062) = 30.0 / {3 * g:.3f} = {30.0 / (3 * g):.2f} kg")
lab("g13", "TWR 2", f"{round(mlim, 2):.2f}", f"1.2 kg, empty drone: TWR {30.0 / Wr:.2f}", f"1.7 kg, payload 0.50 kg: TWR {30.0 / W17:.2f}",
    f"TWR 2 at {round(mlim, 2):.2f} kg: payload limit {round(mlim, 2) - 1.2:.2f} kg")
# Try it 11
r = sq(0.5)
Tq = round(1.10 ** 2 * Wr, 3)
Tl = round(1.10 * Wr, 3)
aq = round((Tq - Wr) / 1.2, 2)
al = round((Tl - Wr) / 1.2, 2)
Tb = round(0.9 ** 2 * Wr, 3)
ab = (Tb - Wr) / 1.2
ex("t11 r", f"ω / ωmax = √(W / Tmax) = √({Wr:.3f} / {2 * Wr:.3f}) = √0.50 = {r:.4f}")
ex("t11 hov", f"{r:.4f} × 1 200 = {round(r, 4) * 1200:.1f} rad/s")
ex("t11 up r", f"ω / ωmax = 1.10 × {r:.4f} = {1.10 * round(r, 4):.4f}")
ex("t11 up TW", f"T / W = 1.10² = {1.10 ** 2:.2f}")
ex("t11 up T", f"T = 1.21 × {Wr:.3f} = {Tq:.3f} N")
ex("t11 up a", f"a = (T − W) / m = ({Tq:.3f} − {Wr:.3f}) / 1.2 = {Tq - Wr:.3f} / 1.2 = {aq:.2f} m/s²")
ex("t11 lin T", f"T = 1.10 × {Wr:.3f} = {Tl:.3f} N")
ex("t11 lin a", f"a = ({Tl:.3f} − {Wr:.3f}) / 1.2 = {Tl - Wr:.3f} / 1.2 = {al:.2f} m/s²")
ex("t11 ratio", f"{aq:.2f} / {al:.2f} = {aq / al:.1f}")
ex("t11 wi TW", f"T / W = 0.90² = {0.9 ** 2:.2f}")
ex("t11 wi T", f"T = 0.81 × {Wr:.3f} = {Tb:.3f} N")
ex("t11 wi a", f"a = ({Tb:.3f} − {Wr:.3f}) / 1.2 = {fm(ab, 2)} m/s²")
ex("t11 wi cmp", f"({fm(ab, 2)} against +{aq:.2f})")
lab("g12", f"{r:.4f}", f"thrust 1/2 needs speed {r:.4f}", "0.00", "0.25", "0.50", "1.00")
lab("g11", "weight: T / W = 1", "TWR 1.0", "TWR 2.0", "TWR 4.0", "1.00", "0.50", "0.25")
lab("g14", f"quadratic law: {1.10 ** 2:.2f}", f"linear law: {1.10:.2f}", "hover: T / W = 1", "1.10", "quadratic law, T / W = x2 (the curve)", "linear law, T / W = x (the dashed line)")

# ================= 2.3.5 =================
for nm, a_ in (("hover", 0.0), ("climb", 3.0), ("descent", -2.0)):
    T = round(1.2 * (g + a_), 3)
    net = T - Wr
    if nm == "hover":
        ex("ax net hover", f"{Wr:.3f} − {Wr:.3f} = 0.000 N")
        ex("ax a hover", "0.000 / 1.2 = 0.00 m/s²")
        ex("ax T hover", f"1.2 × (9.8062 + 0.00) = {1.2 * g:.3f} N")
    elif nm == "climb":
        ex("ax net climb", f"{T:.3f} − {Wr:.3f} = +{net:.3f} N")
        ex("ax a climb", f"+{net:.3f} / 1.2 = +{net / 1.2:.2f} m/s²")
        ex("ax T climb", f"1.2 × (9.8062 + 3.00) = 1.2 × {g + 3:.4f} = {T:.3f} N")
    else:
        ex("ax net descent", f"{T:.3f} − {Wr:.3f} = {fm(net, 3)} N")
        ex("ax a descent", f"{fm(net, 3)} / 1.2 = {fm(net / 1.2, 2)} m/s²")
        ex("ax T descent", f"1.2 × (9.8062 − 2.00) = 1.2 × {g - 2:.4f} = {T:.3f} N")
    ex(f"ax n {nm}", f"{T:.3f} / {Wr:.3f} = {T / Wr:.2f}")
ex("ax n up", f"n = 1 + a / g = 1 + 3.00 / 9.8062 = {1 + 3 / g:.2f}")
ex("ax n dn", f"n = 1 + (−2.00) / 9.8062 = {1 - 2 / g:.2f}")
Tc, Td = round(1.2 * (g + 3.0), 3), round(1.2 * (g - 2.0), 3)
lab("g15", f"T = {Wr:.3f} N", f"W = {Wr:.3f} N", f"T = {Tc:.3f} N", f"T = {Td:.3f} N", "a = 3.00 m/s²", "a = 2.00 m/s², down", "a = 0",
    f"ΣF = {Wr:.3f} − {Wr:.3f} = 0", f"ΣF = {Tc:.3f} − {Wr:.3f} = +{Tc - Wr:.3f} N", f"ΣF = {Td:.3f} − {Wr:.3f} = {fm(Td - Wr, 3)} N")
# Try it 12
sF12 = W17 - Wr
ex("t12 W1", f"W = m·g = 1.7 × 9.8062 = {1.7 * g:.3f} N")
ex("t12 W2", f"W = m·g = 1.2 × 9.8062 = {1.2 * g:.3f} N")
ex("t12 sF", f"ΣF = T − W = {W17:.3f} − {Wr:.3f} = {sF12:.3f} N")
ex("t12 a", f"a = ΣF / m = {sF12:.3f} / 1.2 = {sF12 / 1.2:.2f} m/s²")
ex("t12 v", f"v = a·t = 4.09 × 0.40 = {4.09 * 0.40:.2f} m/s")
ex("t12 h", f"Δh = ½·a·t² = 0.5 × 4.09 × 0.40² = {0.5 * 4.09 * 0.40 ** 2:.2f} m")
ex("t12 chk", f"4.09 / 9.8062 = {4.09 / g:.2f}")
W15 = round(1.5 * g, 3)
ex("t12 wi W", f"W = 1.5 × 9.8062 = {1.5 * g:.3f} N")
ex("t12 wi sF", f"ΣF = {W17:.3f} − {W15:.3f} = {W17 - W15:.3f} N")
ex("t12 wi a", f"a = {W17 - W15:.3f} / 1.5 = {(W17 - W15) / 1.5:.2f} m/s²")
lab("g16", f"T = {W17:.3f} N", f"W = {W17:.3f} N", f"W = {Wr:.3f} N", f"a = {sF12 / 1.2:.2f} m/s²", f"ΣF = {W17:.3f} − {W17:.3f} = 0", f"ΣF = {W17:.3f} − {Wr:.3f} = +{sF12:.3f} N",
    "Before the drop: m = 1.7 kg", "After the drop: m = 1.2 kg")
# Try it 13
T13a, T13b, T13c = round(1.2 * (g + 4.0), 3), round(1.7 * (g + 4.0), 3), round(1.7 * (g + 6.25), 3)
ab13 = (23.534 - W17) / 1.7
ex("t13 a", f"a = v² / (2·d) = 4.0² / (2 × 2.0) = {4.0 ** 2:.1f} / {2 * 2.0:.1f} = {4.0 ** 2 / (2 * 2.0):.2f} m/s²")
ex("t13 T1", f"T = m·(g + a) = 1.2 × (9.8062 + 4.00) = 1.2 × {g + 4:.4f} = {T13a:.3f} N")
ex("t13 T payload", f"T = 1.7 × (9.8062 + 4.00) = 1.7 × {g + 4:.4f} = {T13b:.3f} N")
ex("t13 margin", f"{23.534:.3f} − {T13b:.3f} = {23.534 - T13b:.3f} N")
ex("t13 pct", f"{23.534 - T13b:.3f} / 23.534 = {(23.534 - T13b) / 23.534 * 100:.1f} %")
ex("t13 a5", f"a = 5.0² / (2 × 2.0) = {5.0 ** 2:.1f} / {2 * 2.0:.1f} = {5.0 ** 2 / (2 * 2.0):.2f} m/s²")
ex("t13 T3", f"T = 1.7 × (9.8062 + 6.25) = 1.7 × {g + 6.25:.4f} = {T13c:.3f} N")
ex("t13 over", f"{T13c:.3f} − 23.534 = {T13c - 23.534:.3f} N")
ex("t13 best", f"a = (Tmax − W) / m = (23.534 − {W17:.3f}) / 1.7 = {23.534 - W17:.3f} / 1.7 = {ab13:.2f} m/s²")
ex("t13 d", f"d = v² / (2·a) = 5.0² / (2 × 4.04) = {5.0 ** 2:.1f} / {2 * 4.04:.2f} = {25.0 / (2 * 4.04):.2f} m")
ex("t13 chk", f"v² = 2 × 4.04 × 3.09 = {2 * 4.04 * 3.09:.1f}")
ex("t13 wi a", f"a = 4.0² / (2 × 1.0) = {4.0 ** 2 / (2 * 1.0):.2f} m/s²")
ex("t13 wi T1", f"T = 1.2 × (9.8062 + 8.00) = {1.2 * (g + 8.0):.3f} N")
ex("t13 wi T2", f"T = 1.7 × (9.8062 + 8.00) = 1.7 × {g + 8.0:.4f} = {1.7 * (g + 8.0):.3f} N")
lab("g17", f"{T13a:.3f} N", f"{T13b:.3f} N", f"{T13c:.3f} N", "largest thrust 23.534 N (dashed limit)")

# ================= 2.3.6 =================
ex("drag 5a", f"k1·v = 0.30 × 5.0 = {0.30 * 5.0:.2f} N")
ex("drag 5b", f"k2·v² = 0.06 × 5.0² = {0.06 * 5.0 ** 2:.2f} N")
ex("drag 10a", f"0.30 × 10.0 = {0.30 * 10.0:.2f} N")
ex("drag 10b", f"0.06 × 10.0² = {0.06 * 10.0 ** 2:.2f} N")
ex("drag 20a", f"0.30 × 20.0 = {0.30 * 20.0:.2f} N")
ex("drag 20b", f"0.06 × 20.0² = {0.06 * 20.0 ** 2:.1f} N")
lab("g18", f"{0.06 * 20.0 ** 2:.1f} N (quadratic)", f"{0.30 * 20.0:.2f} N (linear)", "at 5.0 m/s both", f"give {0.30 * 5.0:.2f} N",
    "linear: D = k1·v, k1 = 0.30 N·s/m (straight line)", "quadratic: D = k2·v2, k2 = 0.06 N·s²/m² (curve)")
# Try it 14
D14 = 0.06 * 10.0 ** 2
ex("t14 D", f"D = k2·v² = 0.06 × 10.0² = {D14:.2f} N")
ex("t14 sF", f"ΣF = T − W + D = {Wr:.3f} − {Wr:.3f} + {D14:.2f} = +{D14:.2f} N")
ex("t14 a", f"a = ΣF / m = {D14:.2f} / 1.2 = +{D14 / 1.2:.2f} m/s²")
ex("t14 chk", f"{Wr:.3f} + {D14:.2f} = {Wr + D14:.3f} N")
ex("t14 wi D", f"D = k1·v = 0.30 × 10.0 = {0.30 * 10.0:.2f} N")
ex("t14 wi a", f"a = {0.30 * 10.0:.2f} / 1.2 = +{0.30 * 10.0 / 1.2:.2f} m/s²")
lab("g19", f"T = {Wr:.3f} N", f"D = {D14:.2f} N", f"W = {Wr:.3f} N", f"ΣF = {D14:.2f} N", f"a = {D14 / 1.2:.2f} m/s²", "v = 10.0 m/s",
    f"ΣF = T + D − W = {Wr:.3f} + {D14:.2f} − {Wr:.3f} = +{D14:.2f} N", f"a = ΣF / m = {D14:.2f} / 1.2 = +{D14 / 1.2:.2f} m/s²")
# Try it 15
v15 = sq(6.0 / 0.06)
ex("t15 v", f"v = √(H / k2) = √(6.0 / 0.06) = √{6.0 / 0.06:.0f} = {v15:.1f} m/s")
ex("t15 head", f"{v15:.1f} − 3.0 = {v15 - 3.0:.1f} m/s")
ex("t15 tail", f"{v15:.1f} + 3.0 = {v15 + 3.0:.1f} m/s")
ex("t15 as", f"5.0 + 3.0 = {5.0 + 3.0:.1f} m/s")
ex("t15 D", f"D = k2·v² = 0.06 × 8.0² = {0.06 * 8.0 ** 2:.2f} N")
ex("t15 sF", f"ΣF = H − D = 6.0 − {0.06 * 8.0 ** 2:.2f} = {6.0 - 0.06 * 8.0 ** 2:.2f} N")
ex("t15 a", f"a = ΣF / m = {6.0 - 0.06 * 8.0 ** 2:.2f} / 1.2 = {(6.0 - 0.06 * 8.0 ** 2) / 1.2:.2f} m/s²")
ex("t15 chk", f"0.06 × 10.0² = {0.06 * 10.0 ** 2:.2f} N = H")
ex("t15 wi", f"v = H / k1 = 6.0 / 0.30 = {6.0 / 0.30:.1f} m/s")
lab("g20", f"{v15 - 3.0:.1f} m/s", f"{v15:.1f} m/s", f"{v15 + 3.0:.1f} m/s", f"air speed {v15:.1f} m/s in all three cases (dashed)")

# ================= 2.3.7 =================
Q1, Q2 = round(Wr / 8.0, 3), round(Wr / 4.0, 3)
ex("t16 W", f"W = m·g = 1.2 × 9.8062 = {1.2 * g:.3f} N")
ex("t16 Q1", f"Q = T / Δv = {Wr:.3f} / 8.0 = {Q1:.3f} kg/s")
ex("t16 V", f"{Q1:.3f} / 1.225 = {Q1 / rho:.2f} m³/s")
ex("t16 Q2", f"Q = {Wr:.3f} / 4.0 = {Q2:.3f} kg/s")
ex("t16 ratio", f"{Q2:.3f} / {Q1:.3f} = {Q2 / Q1:.1f}")
ex("t16 wi", f"Q = {Tc:.3f} / 8.0 = {Tc / 8.0:.3f} kg/s")
ex("t16 wi pct", f"{round((Tc / Wr - 1) * 100):d} % more air")
lab("g22", f"T = {Wr:.3f} N", "Δv = 8.0 m/s", f"Q = {Q1:.3f} kg/s of air", f"T = Q·Δv = {Wr:.3f} N", "Δv = 4.0 m/s", f"Q = {Q2:.3f} kg/s of air")
# Try it 17
Wt = round(3.2 * g, 3)
ex("t17 W", f"(M + m)·g = 3.2 × 9.8062 = {3.2 * g:.3f} N")
ex("t17 climb", f"N = (M + m)·g + m·a = {Wt:.3f} + 1.2 × 3.00 = {Wt:.3f} + {1.2 * 3.0:.3f} = {Wt + 1.2 * 3.0:.3f} N")
ex("t17 desc", f"N = {Wt:.3f} + 1.2 × (−2.00) = {Wt:.3f} − {1.2 * 2.0:.3f} = {Wt - 1.2 * 2.0:.3f} N")
ex("t17 chk1", f"{Wt + 1.2 * 3.0:.3f} − {Wt:.3f} = {1.2 * 3.0:.3f} N")
ex("t17 chk2", f"m·a = 1.2 × 3.00 = {1.2 * 3.0:.3f} N")
ex("t17 wi", f"N = (M + m)·g + m·(−g) = M·g = 2.0 × 9.8062 = {2.0 * g:.3f} N")
lab("g23", f"N = {Wt:.3f} N", f"N = {Wt + 3.6:.3f} N", f"N = {Wt - 2.4:.3f} N", f"W = {Wt:.3f} N", f"N = (2.0 + 1.2) × 9.8062 = {Wt:.3f} N",
    f"N = {Wt:.3f} + 1.2 × 3.00 = {Wt + 3.6:.3f} N", f"N = {Wt:.3f} + 1.2 × (−2.00) = {Wt - 2.4:.3f} N")

# ================= 2.3.8 problems =================
# Problem 1
ex("p1 m", f"m = 1.2 + 0.30 = {1.2 + 0.30:.1f} kg")
ex("p1 W", f"W = m·g = 1.5 × 9.8062 = {1.5 * g:.3f} N")
ex("p1 TWR", f"TWR = Tmax / W = 30.0 / {W15:.3f} = {30.0 / W15:.2f}")
ex("p1 pay", f"{round(mlim, 2):.2f} − 1.2 = {round(mlim, 2) - 1.2:.2f} kg")
ex("p1 amax", f"amax = (Tmax − W) / m = (30.0 − {W15:.3f}) / 1.5 = {30.0 - W15:.3f} / 1.5 = {(30.0 - W15) / 1.5:.2f} m/s²")
T1p = round(1.5 * (g + 6.0), 3)
ex("p1 T", f"T = 1.5 × (9.8062 + 6.00) = 1.5 × {g + 6.0:.4f} = {T1p:.3f} N")
ex("p1 share", f"{T1p:.3f} / 30.0 = {T1p / 30.0:.2f}")
lab("g24", f"{W15:.3f} N", f"{30.0 / 2:.3f} N", f"{T1p:.3f} N", "30.0 N")
# Problem 2
Vp, Hp = Wr, 2.4
T2p = sq(round(Vp ** 2, 3) + round(Hp ** 2, 3))
T2r = round(T2p, 3)
ex("p2 V", f"V = W = m·g = 1.2 × 9.8062 = {Wr:.3f} N")
ex("p2 H", "H = Hw = 2.4 N")
ex("p2 T", f"T = √(V² + H²) = √({Wr:.3f}² + 2.4²) = √({Vp ** 2:.3f} + {Hp ** 2:.3f}) = √{round(Vp ** 2, 3) + round(Hp ** 2, 3):.3f} = {T2r:.3f} N")
th2p = math.degrees(math.atan(Hp / Vp))
ex("p2 th", f"θ = atan(H / V) = atan(2.4 / {Wr:.3f}) = {th2p:.1f}°")
ex("p2 ratio", f"T / W = {T2r:.3f} / {Wr:.3f} = {T2r / Wr:.3f}")
ex("p2 pct", f"the tilt costs {(T2r / Wr - 1) * 100:.1f} % more thrust")
T2q = sq(round(Vp ** 2, 3) + round(4.8 ** 2, 3))
ex("p2 T4", f"T = √({Wr:.3f}² + 4.8²) = √({Vp ** 2:.3f} + {4.8 ** 2:.3f}) = √{round(Vp ** 2, 3) + round(4.8 ** 2, 3):.3f} = {T2q:.3f} N")
ex("p2 th4", f"θ = atan(4.8 / {Wr:.3f}) = {math.degrees(math.atan(4.8 / Vp)):.1f}°")
ex("p2 ratio4", f"T / W = {round(T2q, 3):.3f} / {Wr:.3f} = {round(T2q, 3) / Wr:.3f}")
lab("g25", f"V = {Wr:.3f} N", f"T = {T2r:.3f} N", "H = 2.4 N, points west", f"tilt {th2p:.1f}° from the vertical")
# Problem 3
t3a = sq(2 * 20.0 / gD)
ex("p3 W", f"W = m·g = 1.2 × 9.7965 = {Wd:.3f} N")
ex("p3 t", f"t = √(2·h / g) = √(2 × 20.0 / 9.7965) = √{2 * 20.0 / gD:.2f} = {t3a:.2f} s")
ex("p3 v", f"v = g·t = 9.7965 × {round(t3a, 2):.2f} = {gD * round(t3a, 2):.2f} m/s")
ex("p3 D", f"D = k2·v² = 0.06 × 10.0² = {0.06 * 10.0 ** 2:.2f} N")
ex("p3 sF", f"ΣF = D − W = 6.00 − {Wd:.3f} = {fm(6.0 - Wd, 3)} N")
ex("p3 a", f"a = ΣF / m = {fm(6.0 - Wd, 3)} / 1.2 = {fm((6.0 - Wd) / 1.2, 2)} m/s²")
vt = sq(Wd / 0.06)
ex("p3 vt", f"v = √(W / k2) = √({Wd:.3f} / 0.06) = √{Wd / 0.06:.2f} = {vt:.2f} m/s")
ex("p3 chk", f"0.06 × 14.00² = {0.06 * 14.00 ** 2:.2f} N")
lab("g26", f"{M}{gD:.4f} m/s² at rest", f"{fm(-gD + 0.06 * 10.0 ** 2 / 1.2, 2)} m/s² at 10.0 m/s", f"0 at {vt:.2f} m/s: drag = weight")
# Problem 4
W08 = round(0.8 * g, 3)
Tm4 = round(4.0 * W08, 3)
a4 = (Tm4 - W08) / 0.8
t4 = sq(2 * 10.0 / round(a4, 2))
ex("p4 W", f"W = m·g = 0.8 × 9.8062 = {0.8 * g:.3f} N")
ex("p4 Tmax", f"Tmax = TWR × W = 4.0 × {W08:.3f} = {Tm4:.3f} N")
ex("p4 a", f"a = (Tmax − W) / m = ({Tm4:.3f} − {W08:.3f}) / 0.8 = {Tm4 - W08:.3f} / 0.8 = {a4:.2f} m/s²")
ex("p4 t", f"t = √(2·h / a) = √(2 × 10.0 / {round(a4, 2):.2f}) = √{2 * 10.0 / round(a4, 2):.2f} = {t4:.2f} s")
ex("p4 v", f"v = a·t = {round(a4, 2):.2f} × {round(t4, 2):.2f} = {round(a4, 2) * round(t4, 2):.2f} m/s")
ex("p4 chk", f"½ × {round(t4, 2):.2f} × {round(a4, 2) * round(t4, 2):.2f} = {0.5 * round(t4, 2) * round(round(a4, 2) * round(t4, 2), 2):.2f} m")
lab("g27", f"{round(round(a4, 2) * round(t4, 2), 2):.2f} m/s at {round(t4, 2):.2f} s", f"slope = a = {a4:.2f} m/s²", "area under the line = height gained = 10.0 m", f"{round(t4, 2):.2f}")
# Problem 5
u5 = 0.50 * 1.7 / 1.2
ex("p5 u", f"u = 0.50 × 1.7 / 1.2 = {u5:.4f}")
ex("p5 TWR", f"TWR = 1 / u = 1 / {u5:.4f} = {1 / round(u5, 4):.2f}")
tw5 = round(1 / round(u5, 4), 2)
ex("p5 a", f"amax = (TWR − 1)·g = ({tw5:.2f} − 1) × 9.8062 = {(tw5 - 1) * g:.2f} m/s²")
ex("p5 chk1", f"1 − {u5:.4f} = {1 - round(u5, 4):.2f}")
ex("p5 chk2", f"{(tw5 - 1) * g:.2f} / 9.806 = {round((tw5 - 1) * g, 2) / 9.806:.2f}")
lab("g28", "0.50", f"{u5:.4f}", "limit: full thrust, 1.0", "spare 0.50", f"spare {1 - round(u5, 4):.2f}")

# ================= 2.3.9 Think =================
# Think 1
rr = round(rhoD / rho, 3)
TmD = round(rr * 30.0, 2)
Wp = round(1.6 * gD, 3)
mD = TmD / (2 * gD)
ex("k1 ratio", f"ρ(Denver) / ρ(sea level) = 1.049 / 1.225 = {rr:.3f}")
ex("k1 Tmax", f"Tmax(Denver) = {rr:.3f} × 30.0 = {TmD:.2f} N")
ex("k1 m", f"m = 1.2 + 0.40 = {1.2 + 0.40:.1f} kg")
ex("k1 W", f"W = m·g = 1.6 × 9.7965 = {1.6 * gD:.3f} N")
ex("k1 TWR", f"TWR = {TmD:.2f} / {Wp:.3f} = {TmD / Wp:.2f}")
ex("k1 amax", f"amax = ({TmD:.2f} − {Wp:.3f}) / 1.6 = {TmD - Wp:.3f} / 1.6 = {(TmD - Wp) / 1.6:.2f} m/s²")
ex("k1 mlim", f"m = {TmD:.2f} / (2.0 × 9.7965) = {TmD:.2f} / {2 * gD:.3f} = {mD:.2f} kg")
ex("k1 pay", f"{round(mD, 2):.2f} − 1.2 = {round(mD, 2) - 1.2:.2f} kg")
ex("k1 lost", f"0.33 − {round(round(mD, 2) - 1.2, 2):.2f} = {0.33 - round(round(mD, 2) - 1.2, 2):.2f} kg")
ex("k1 gonly", f"m = 30.0 / (2.0 × 9.7965) = 30.0 / {2 * gD:.3f} = {30.0 / (2 * gD):.2f} kg")
W16h = round(1.6 * g, 3)
ex("k1 chk1", f"W = 1.6 × 9.8062 = {1.6 * g:.3f} N")
ex("k1 chk2", f"TWR = 30.0 / {W16h:.3f} = {30.0 / W16h:.2f}")
Tneed = round(2.0 * Wp, 3)
ex("k1 wi T", f"Tmax = 2.0 × {Wp:.3f} = {Tneed:.3f} N")
ex("k1 wi ratio", f"{Tneed:.3f} / {TmD:.2f} = {Tneed / TmD:.2f}")
lab("t1", "30.0 N", f"{TmD:.2f} N", f"needed for TWR 2 with the parcel: {Tneed:.3f} N")
# Think 2
ex("k2 W", f"W = m·g = 1.2 × 9.8062 = {1.2 * g:.3f} N")
ex("k2 TWR4", f"TWR = 24.0 / {Wr:.3f} = {24.0 / Wr:.2f}")
ex("k2 18", f"3 × 6.0 = {3 * 6.0:.1f} N")
ex("k2 TWR3", f"TWR = 18.0 / {Wr:.3f} = {18.0 / Wr:.2f}")
s4, s3 = round(Wr / 4, 3), round(Wr / 3, 3)
ex("k2 s4", f"{Wr:.3f} / 4 = {s4:.3f} N")
ex("k2 sh4", f"{s4:.3f} / 6.0 = {s4 / 6.0:.2f}")
ex("k2 s3", f"{Wr:.3f} / 3 = {s3:.3f} N")
ex("k2 sh3", f"{s3:.3f} / 6.0 = {s3 / 6.0:.2f}")
ex("k2 a3", f"amax = (18.0 − {Wr:.3f}) / 1.2 = {18.0 - Wr:.3f} / 1.2 = {(18.0 - Wr) / 1.2:.2f} m/s²")
ex("k2 a4", f"(24.0 − {Wr:.3f}) / 1.2 = {(24.0 - Wr) / 1.2:.2f} m/s²")
m18 = 18.0 / g
ex("k2 m", f"m = 18.0 / 9.8062 = {m18:.2f} kg")
ex("k2 pay", f"{round(m18, 2):.2f} − 1.2 = {round(m18, 2) - 1.2:.2f} kg")
ex("k2 chk", f"3 × {s3:.3f} = {3 * s3:.3f} N")
W20 = round(2.0 * g, 3)
ex("k2 wi W", f"W = 2.0 × 9.8062 = {2.0 * g:.3f} N")
ex("k2 wi sF", f"ΣF = 18.0 − {W20:.3f} = {fm(18.0 - W20, 3)} N")
ex("k2 wi a", f"a = {fm(18.0 - W20, 3)} / 2.0 = {fm((18.0 - W20) / 2.0, 2)} m/s²")
lab("t2", f"{s4:.3f} N", f"{s3:.3f} N", "largest thrust of one rotor: 6.0 N", f"four rotors, {s4:.3f} N each", f"three rotors, {s3:.3f} N each")
# Think 3
T0 = round(2.2 * g, 3)
W1h = round(1.95 * g, 3)
W2h = round(1.70 * g, 3)
ex("k3 m0", f"m0 = 1.2 + 1.0 = {1.2 + 1.0:.1f} kg")
ex("k3 T", f"T = W0 = m·g = 2.2 × 9.8062 = {2.2 * g:.3f} N")
ex("k3 m1", f"m = 2.2 − 0.25 = {2.2 - 0.25:.2f} kg")
ex("k3 W1", f"W = 1.95 × 9.8062 = {1.95 * g:.3f} N")
ex("k3 sF1", f"ΣF = {T0:.3f} − {W1h:.3f} = {T0 - W1h:.3f} N")
ex("k3 a1", f"a = ΣF / m = {T0 - W1h:.3f} / 1.95 = {(T0 - W1h) / 1.95:.2f} m/s²")
ex("k3 m2", f"m = 2.2 − 0.50 = {2.2 - 0.50:.2f} kg")
ex("k3 W2", f"W = 1.70 × 9.8062 = {1.70 * g:.3f} N")
ex("k3 sF2", f"ΣF = {T0:.3f} − {W2h:.3f} = {T0 - W2h:.3f} N")
a32 = round((T0 - W2h) / 1.70, 2)
ex("k3 a2", f"a = {T0 - W2h:.3f} / 1.70 = {a32:.2f} m/s²")
ex("k3 share", f"{a32:.2f} / 9.8062 = {a32 / g:.2f}")
ex("k3 empty", f"{Wr:.3f} / {T0:.3f} = {Wr / T0:.2f}")
ex("k3 chk", f"0.25 × 9.8062 = {0.25 * g:.3f} N")
ex("k3 wi", f"1.7 × 9.8062 = {1.7 * g:.3f} N")
lab("t3", f"thrust T = {T0:.3f} N, kept constant", f"weight W = m·g falls to {Wr:.3f} N at 4 min", "net force ΣF = T − W (shaded)")

# ===== the page's figure ids and the expected structure =====
FIG_IDS = [f"g{i}" for i in range(1, 11)] + ["g12", "g11"] + [f"g{i}" for i in range(13, 29)] + ["t1", "t2", "t3"]
REFS_EXPECT = {1, 21, 24, 29, 30, 38, 39, 42, 45, 69, 70}

JS_FIG = """() => { const out = {ids: [], texts: {}, empty: [], clipped: [], overlaps: [], caps: []};
  for (const s of document.querySelectorAll('figure svg')) {
    out.ids.push(s.id);
    const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
    out.texts[s.id] = ts.map(o => o.t.textContent.trim());
    if (s.children.length < 5) out.empty.push(s.id);
    const vb = s.viewBox.baseVal, root = s.getBoundingClientRect(), sc = root.width / vb.width;
    for (const o of ts) { const r = o.r; const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
      if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.clipped.push(s.id + ': ' + o.t.textContent.slice(0, 40)); }
    for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
      const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
      if (w > 1 && h > 1) { const fr = w * h / Math.min(a.width * a.height, c.width * c.height);
        if (fr > 0.05) out.overlaps.push(s.id + ': [' + ts[i].t.textContent.slice(0, 25) + '] / [' + ts[j].t.textContent.slice(0, 25) + '] ' + fr.toFixed(2)); } } }
  for (const c of document.querySelectorAll('figcaption')) out.caps.push(c.textContent.trim().slice(0, 14));
  return out; }"""

JS_TAB = """() => { const out = {tables: 0, bad: [], order: [], figmatch: []};
  for (const b of document.querySelectorAll('.blk')) {
    const c = b.querySelector('.c'); const n = c.querySelectorAll('table').length; if (!n) continue; out.tables += n;
    const how = (b.innerText.match(/How to read this table[.]/g) || []).length, figs = b.querySelectorAll('figure').length;
    if (how < n || figs < n) out.bad.push(b.dataset.b + ': tables ' + n + ', how-to ' + how + ', figures ' + figs);
    const kids = [...c.querySelectorAll('p, .tblk')]; let seenHow = 0, seenTab = 0;
    for (const k of kids) { if (k.classList.contains('tblk')) { seenTab++; if (seenHow < seenTab) out.order.push(b.dataset.b + ' table ' + seenTab + ' has no how-to paragraph before it'); }
      else if (k.innerText.trim().startsWith('How to read this table.')) seenHow++; }
    for (const t of c.querySelectorAll('.tblk')) {
      let p = t.previousElementSibling; while (p && !(p.tagName === 'P' && p.innerText.trim().startsWith('How to read this table.'))) p = p.previousElementSibling;
      const f = t.nextElementSibling;
      if (!p) { out.figmatch.push(b.dataset.b + ': no how-to paragraph among the previous siblings'); continue; }
      if (!f || f.tagName !== 'FIGURE') { out.figmatch.push(b.dataset.b + ': the element after the table is not a figure'); continue; }
      const m = f.querySelector('figcaption').textContent.trim().match(/^Figure ([0-9]+)/);
      if (!m || !new RegExp('Figure ' + m[1] + '[^0-9]').test(p.innerText)) out.figmatch.push(b.dataset.b + ': the how-to paragraph does not name ' + (m ? 'Figure ' + m[1] : 'the figure after the table')); } }
  return out; }"""

JS_PROBS = """() => { const out = {tags: [], items: []};
  out.tags = [...document.querySelectorAll('.prob .tag')].map(t => t.innerText.trim());
  for (const p of document.querySelectorAll('.prob')) { const tag = p.querySelector('.tag').innerText.trim();
    const lis = [...p.querySelectorAll(':scope > details > ol > li')];
    out.items.push({tag: tag.slice(0, 30), figures: p.querySelectorAll('figure svg').length, steps: lis.length,
      without_why: lis.filter(l => !l.innerText.trim().startsWith('Why this step:')).length}); }
  return out; }"""

JS_CITES = """() => { const blocks = [...document.querySelectorAll('[data-b="refs"]')];
  const inRefs = a => blocks.some(b => b.contains(a));
  const all = [...document.querySelectorAll('a.cite')]; const pair = a => [a.getAttribute('href'), a.textContent.trim()];
  return {outside: all.filter(a => !inRefs(a)).map(pair), listed: [...document.querySelectorAll('[data-b="refs"] ul.reflist > li')].map(li => li.querySelector('a.cite').textContent.trim()), n_blocks: blocks.length}; }"""

res = {"errors": list(pre_errors), "checked_strings": len(E)}
errors = res["errors"]
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: errors.append("page error: " + str(e)))
    pg.goto(page_path.as_uri())
    pg.wait_for_selector(".blk")
    pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
    text = pg.inner_text("main").replace(" ", " ").replace(" ", " ").replace(" ", " ")
    src = page_path.read_text(encoding="utf-8")
    res["blocks"] = pg.locator(".blk").count()
    # ---- figures ----
    fg = pg.evaluate(JS_FIG)
    res["figure_ids"] = fg["ids"]
    if fg["ids"] != FIG_IDS:
        errors.append(f"figure ids {fg['ids']} differ from the expected page order {FIG_IDS}")
    fe = pg.evaluate("window.figErrors")
    if fe:
        errors.append(f"figure errors {fe}")
    if fg["empty"]:
        errors.append(f"empty figures {fg['empty']}")
    if fg["clipped"]:
        errors.append(f"labels outside the figure {fg['clipped']}")
    res["clipped_labels"] = fg["clipped"]
    res["overlapping_labels"] = fg["overlaps"]
    if fg["overlaps"]:
        errors.append(f"overlapping labels {fg['overlaps']}")
    labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", src)
    res["caption_labels"] = labs
    if labs != [str(i + 1) for i in range(31)] or len(fg["ids"]) != 31:
        errors.append(f"figure labels {labs} are not 1..31 for {len(fg['ids'])} figures")
    for i, fid in enumerate(fg["ids"]):
        try:
            pg.locator("figure").nth(i).screenshot(path=f"out/fig_2_3/{i + 1:02d}_{fid}.png")
        except Exception as e:
            errors.append(f"screenshot of {fid} failed: {str(e)[:80]}")
    pg.screenshot(path="out/2_3_full.png", full_page=True)
    svgall = "\n".join("\n".join(v) for v in fg["texts"].values())
    # ---- numbers (page text or the svg text) ----
    missing = {}
    for name, s in E:
        if s not in text and s not in svgall:
            missing[name] = s
    # ---- figure labels: each must be the exact text of a text element of that figure ----
    for fid, items in FL.items():
        have = set(fg["texts"].get(fid, []))
        for s in items:
            if s not in have:
                missing[f"figure {fid}: {s}"] = s
    res["missing_numbers"] = missing
    # ---- negative control ----
    base = next(s for n, s in E if n == "t13 T payload")
    mut = base.replace("23.471", "23.472")
    nc_ok = mut != base and mut not in text and mut not in svgall
    res["negative_control"] = {"mutated": mut, "reported_missing": nc_ok}
    if not nc_ok:
        errors.append("negative control failed: the mutated number was not reported missing")
    # ---- structure ----
    pr = pg.evaluate(JS_PROBS)
    tags = pr["tags"]
    tries = [int(m.group(1)) for t in tags for m in [re.match(r"TRY IT (\d+) ·", t)] if m]
    probs = [int(m.group(1)) for t in tags for m in [re.match(r"PROBLEM (\d+) ·", t)] if m]
    thinks = [int(m.group(1)) for t in tags for m in [re.match(r"THINK (\d+) ·", t)] if m]
    res["try_its"], res["problems"], res["thinks"] = tries, probs, thinks
    if tries != list(range(1, 18)):
        errors.append(f"Try it tags {tries}, expected 1..17 in order")
    if probs != [1, 2, 3, 4, 5]:
        errors.append(f"Problem tags {probs}, expected 1..5")
    if thinks != [1, 2, 3]:
        errors.append(f"Think tags {thinks}, expected 1..3")
    levels = {int(m.group(1)): m.group(2) for t in tags for m in [re.match(r"TRY IT (\d+) · (WARM-UP|MEDIUM|HARD) ·", t)] if m}
    res["levels"] = levels
    if len(levels) != 17 or {n for n, v in levels.items() if v == "WARM-UP"} != {1, 6, 9}:
        errors.append(f"ladder levels {levels}: the warm-ups must be exactly Try its 1, 6 and 9 and every tag needs a level")
    for it in pr["items"]:
        lv = re.match(r"TRY IT (\d+) · (WARM-UP|MEDIUM|HARD)", it["tag"])
        if lv and lv.group(2) != "WARM-UP":
            if it["figures"] < 1 or it["steps"] < 3 or it["without_why"]:
                errors.append(f"ladder item {it}")
    res["items"] = pr["items"]
    for pat, hi, nm in ((r"Try its? (\d+)(?:\s*(?:and|,)\s*(\d+))?", 17, "Try it"), (r"Figures? (\d+)", 31, "Figure"),
                        (r"Problems? (\d+)(?:\s*(?:and|to)\s*(\d+))?", 5, "Problem"), (r"Think (\d+)(?:\s*(?:and|to)\s*(\d+))?", 3, "Think")):
        for m in re.finditer(pat, text):
            for x in m.groups():
                if x is not None and not 1 <= int(x) <= hi:
                    errors.append(f"reference '{m.group(0)}' points to a {nm} that does not exist (1..{hi})")
    # ---- tables ----
    tb = pg.evaluate(JS_TAB)
    res["tables"] = tb["tables"]
    res["table_tools"] = pg.locator(".tblk .ttools").count()
    if tb["tables"] != res["table_tools"] or tb["tables"] == 0:
        errors.append(f"{tb['tables']} tables but {res['table_tools']} table tools")
    for key_ in ("bad", "order", "figmatch"):
        if tb[key_]:
            errors.append(f"tables {key_}: {tb[key_]}")
    # ---- references block ----
    ck = pg.evaluate(JS_CITES)
    cited = set()
    for href, t in ck["outside"]:
        if not href.endswith("#ref-" + t):
            errors.append(f"cite {t} has href {href}")
        cited.add(int(t))
    listed = [int(x) for x in ck["listed"]]
    res["cited"], res["listed"] = sorted(cited), listed
    if ck["n_blocks"] != 1 or sorted(listed) != sorted(cited) or len(set(listed)) != len(listed):
        errors.append(f"References block {listed} differs from the numbers cited in the text {sorted(cited)}")
    if cited != REFS_EXPECT:
        errors.append(f"cited numbers {sorted(cited)} differ from the expected {sorted(REFS_EXPECT)}")
    # ---- links ----
    hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
    mapsrc = (page_path.parent.parent / "reference" / "autopilot_map.html").read_text(encoding="utf-8")
    bad_links = []
    for h in hrefs:
        if h.startswith("http") or h.startswith("#"):
            continue
        base_, _, frag = h.partition("#")
        if not (page_path.parent / base_).exists():
            bad_links.append(h)
        elif "autopilot_map" in base_ and frag and f'id="{frag}"' not in mapsrc:
            bad_links.append(h + " (anchor not in the map)")
    res["broken_links"] = bad_links
    if bad_links:
        errors.append(f"broken links {bad_links}")
    ext = sorted({h for h in hrefs if h.startswith("http")})
    res["external_links"] = ext
    # ---- code quotes ----
    pres = [" ".join(x.split()) for x in pg.locator("pre.code").all_inner_texts()]
    links = pg.evaluate("[...document.querySelectorAll('a[href^=\"https://github.com\"]')].map(a => a.href)")
    # ---- bans ----
    if re.search(r"(?<![\d.])9\.81(?!\d)", text) or re.search(r"(?<![\d.])9\.81(?!\d)", src):
        errors.append("the number 9.81 appears; use 9.8062 or 9.7965")
    if re.search(r"refut", text, re.I) or re.search(r"refut", src, re.I):
        errors.append("banned word 'refut'")
    if any("<sub>" in s_ for s_ in re.findall(r"<svg.*?</svg>", src, flags=re.S)):
        errors.append("HTML <sub> inside an svg")
    # ---- marks and notes ----
    blk = pg.locator('.blk[data-b="goal"]')
    blk.hover()
    blk.locator('button.dot[title="question"]').click()
    blk.locator("text=+ Note").click()
    pg.wait_for_timeout(200)
    pg.keyboard.type("sample note on 2.3")
    pg.wait_for_timeout(600)
    pg.reload()
    pg.wait_for_selector(".blk")
    res["note_after_reload"] = pg.locator('.blk[data-b="goal"] textarea').input_value()
    page_id = pg.evaluate("PAGE")
    res["ls_keys"] = pg.evaluate("Object.keys(JSON.parse(localStorage.getItem('ardp_v2_sec_" + page_id + "')||'{}'))")
    if res["note_after_reload"] != "sample note on 2.3" or res["ls_keys"] != [page_id + ":goal"]:
        errors.append(f"note did not survive reload under key {page_id}:goal ({res['ls_keys']})")
    tbl = pg.locator(".tblk").nth(1)
    tkey = page_id + ":" + tbl.get_attribute("data-t")
    tbl.hover()
    tbl.locator('.ttools button.dot[title="change this"]').click()
    tbl.locator(".ttools >> text=+ Note").click()
    pg.wait_for_timeout(200)
    pg.keyboard.type("table note on 2.3")
    pg.wait_for_timeout(600)
    pg.reload()
    pg.wait_for_selector(".blk")
    tbl = pg.locator('.tblk[data-t="' + tkey.split(":", 1)[1] + '"]')
    res["table_note_after_reload"] = tbl.locator(".tnotes textarea").input_value()
    if res["table_note_after_reload"] != "table note on 2.3" or "mk-red" not in (tbl.get_attribute("class") or ""):
        errors.append(f"table note/mark did not survive reload under {tkey}")
    tbl.screenshot(path="out/2_3_table_tools.png")
    b.close()

# ---- code quotes against the reference data ----
ref = {}
for name in ("gazebo", "airsim", "px4sim", "px4ap"):
    f = page_path.parent.parent / "reference" / f"sim_formulas_{name}.js"
    txt = f.read_text(encoding="utf-8").strip()
    for row in json.loads(re.match(r"^window\.SIM_FORMULAS_[A-Z0-9]+\s*=\s*(\[.*\])\s*;?\s*$", txt, re.S).group(1)):
        ref[row["id"]] = row
ws = lambda s: " ".join(s.split())
need = ["g08-sih-translation", "PA-8-01", "PA-2-01", "g02-sih-multirotor-linear", "g05-sih-mr-drag", "PA-1-01"]
res["code_quotes"] = {}
for i in need:
    ok = i in ref and ws(ref[i]["code"]) in pres
    res["code_quotes"][i] = ok
    if not ok:
        errors.append(f"code of reference row {i} is not quoted word for word on the page")
want = {ws(ref[i]["code"]) for i in need if i in ref}
res["code_blocks"] = len(pres)
if len(pres) != 6:
    errors.append(f"{len(pres)} code blocks, expected 6")
for q in pres:
    if q not in want:
        errors.append(f"a code block is not one of the six reference rows: {q[:60]}")


def ghlink(r):
    m = re.search(r"(\d+)(?:\s*[-–]\s*(\d+))?", str(r["lines"]))
    a = ("#L" + m.group(1) + ("-L" + m.group(2) if m.group(2) else "")) if m else ""
    return f"https://github.com/{r['repo']}/blob/{r['commit']}/{r['file']}{a}"


exp_links = {ghlink(ref[i]) for i in need if i in ref}
res["github_links_missing"] = sorted(exp_links - set(links))
if res["github_links_missing"]:
    errors.append(f"github links not on the page: {res['github_links_missing']}")
res["links"] = {}
for u in res["external_links"]:
    try:
        req = urllib.request.Request(u, headers={"User-Agent": "Mozilla/5.0"})
        res["links"][u] = urllib.request.urlopen(req, timeout=30).status
    except Exception as e:
        res["links"][u] = str(e)[:80]
for u, s in res["links"].items():
    if "github.com" in u and s != 200:
        errors.append(f"GitHub link does not answer 200: {u} -> {s}")

Path("out/check_2_3.json").write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding="utf-8")
print(json.dumps({"missing": list(res["missing_numbers"].keys()), "errors": res["errors"], "expected_not_found": res["missing_numbers"],
                  "checked_strings": res["checked_strings"], "figures": len(res["figure_ids"]), "tables": res["tables"],
                  "negative_control": res["negative_control"]}, ensure_ascii=False))
sys.exit(1 if res["missing_numbers"] or res["errors"] else 0)
