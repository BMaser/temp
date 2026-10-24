"""Check the Think problems of the four pages 2.2a, 2.2b, 2.2c and 2.2d. Run on the cloud machine only (never on the owner's PC):
    python check_2_2d.py page/samples/2.2a_*.html page/samples/2.2b_*.html page/samples/2.2c_*.html page/samples/2.2d_*.html
What it checks:
 1. Every number of every Think problem (and of the Coriolis subsection of 2.2b, and of the check-yourself questions of 2.2d) is recomputed
    here from the rounded inputs that the page prints, and must appear, formatted as printed, in the text of its page. It also compares the
    printed results with the exact values of think_calc.py (a second, independent computation) within a stated tolerance.
 2. Think tags run 1..25 over the four pages in page order. Every Think block has a figure, "Think first", a hint, a solution with at least two
    "Why this step" lines, "Check your answer." and "What if".
 3. Every pre.cx quotation of page 2.2b (data-src="repo|commit|path|lines") is found, word for word (spaces ignored), in the named file at the commit.
 4. Page 2.2d as a whole: figures (labels 1..N after the renumbering step, no figure error, no empty figure, no clipped or overlapping label, one PNG
    each), tables ("How to read this table." before each table, a figure in the same block, a marks and notes tool each), local links, gravity
    (never 9.81), the banned word, no HTML sub inside an svg, notes and marks that survive a reload."""
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
M = "−"                                              # the minus sign printed on the pages


def fm(x, d=1):                                      # fixed decimals, true minus sign
    s = f"{x:.{d}f}"
    return (M + s[1:]) if s.startswith("-") and float(s) != 0 else s.lstrip("-")


def r2(x):
    return round(x, 2)


g, gD = 9.8062, 9.7965
E = []                                               # (page, name, printed string)


def ex(page, name, s):
    E.append((page, name, s))


def deg(x):
    return math.degrees(x)


# ================================================== 2.2a
A = lambda n, s: ex("A", n, s)
A("t1 out", f"t = d / v = 120 / 8.0 = {fm(120 / 8.0)} s")
A("t1 back", f"t = d / v = 120 / 24.0 = {fm(120 / 24.0)} s")
A("t1 T", f"T = 15.0 + 5.0 = {fm(15.0 + 5.0)} s")
A("t1 avg", f"d / T = 240 / 20.0 = {fm(240 / 20.0)} m/s")
A("t1 vel", "Δx / T = 0 / 20.0 = 0 m/s")
A("t1 T16", f"240 / 16.0 = {fm(240 / 16.0)} s")
A("t1 left", f"15.0 − 15.0 = {fm(15.0 - 15.0)} s")
A("t1 share", f"15.0 / 20.0 = {round(15.0 / 20.0 * 100)} %")
A("t1 w1", f"120 / 12.0 = {fm(120 / 12.0)} s")
A("t1 w2", f"T = 15.0 s")
A("t1 w3", f"240 / 15.0 = {fm(240 / 15.0)} m/s")
A("t2 diff", "Δx = ½·a·3² − ½·a·2² = ½·a·(9 − 4) = 2.5·a")
A("t2 a", f"a = 10.0 / 2.5 = {fm(10.0 / 2.5)} m/s²")
A("t2 x3", f"x = ½ × 4.0 × 3² = {fm(0.5 * 4.0 * 9)} m")
A("t2 v3", f"v = a·t = 4.0 × 3.0 = {fm(4.0 * 3.0)} m/s")
A("t2 d1", f"½ × 4.0 × 1² = {fm(0.5 * 4.0 * 1)} m")
A("t2 d2", f"½ × 4.0 × (2² − 1²) = {fm(0.5 * 4.0 * (4 - 1))} m")
A("t2 d3", f"½ × 4.0 × (3² − 2²) = {fm(0.5 * 4.0 * (9 - 4))} m")
A("t2 mid", f"4.0 × 2.5 × 1.0 = {fm(4.0 * 2.5 * 1.0)} m")
A("t2 sum", f"2.0 + 6.0 + 10.0 = {fm(2.0 + 6.0 + 10.0)} m")
A("t2 w1", "Δx = v₀·1 + 2.5·a = 10.0")
A("t2 w2", f"a = (10.0 − 5.0) / 2.5 = {fm((10.0 - 5.0) / 2.5)} m/s²")
dd = 24.0 * 0.25
A("t3 delay", f"Δx = v·t = 24.0 × 0.25 = {fm(dd)} m")
A("t3 left", f"35.0 − 6.0 = {fm(35.0 - dd)} m")
v2 = 24.0 ** 2 + 2 * (-8.0) * 29.0
A("t3 v2", f"v² = v₀² + 2·a·Δx = 24.0² + 2 × ({M}8.0) × 29.0 = {24 ** 2} − {int(2 * 8 * 29)} = {int(v2)}")
A("t3 v", f"v = √112 = {fm(sq(112), 2)} m/s")
A("t3 latest", f"Δx = (8.0² − 24.0²) / (2 × ({M}8.0)) = (64 − 576) / ({M}16) = {fm((64 - 576) / (2 * -8.0))} m")
A("t3 total", f"32.0 + 6.0 = {fm(32.0 + dd)} m")
A("t3 a", f"a = (v² − v₀²) / (2·Δx) = (8.0² − 24.0²) / (2 × 29.0) = (64 − 576) / 58.0 = {fm((64 - 576) / 58.0, 2)} m/s²")
A("t3 late", f"38.0 − 35.0 = {fm(38.0 - 35.0)} m")
A("t3 time", f"3.0 / 24.0 = {fm(3.0 / 24.0, 3)} s")
A("t3 w1", f"24.0 × 0.10 = {fm(24.0 * 0.10)} m")
A("t3 w2", f"32.0 + 2.4 = {fm(32.0 + 2.4)} m")
A("t3 w3", f"v² = 24.0² + 2 × ({M}8.0) × 32.6 = 576 − {fm(2 * 8 * 32.6)} = {fm(576 - 2 * 8 * 32.6)}")
A("t3 w4", f"v = √54.4 = {fm(sq(54.4), 2)} m/s")
A("t4 up", f"24.0² / (2 × 6.0) = 576 / 12 = {fm(576 / 12)} m")
A("t4 dn", f"24.0² / (2 × 9.0) = 576 / 18 = {fm(576 / 18)} m")
A("t4 eq", "v² / 12 + v² / 18 = v² × 5 / 36 = 60")
A("t4 v2", f"v² = 60 × 36 / 5 = {int(60 * 36 / 5)}")
A("t4 v", f"v = √432 = {fm(sq(432), 3)} m/s")
A("t4 T", f"T = 20.785 / 6.0 + 20.785 / 9.0 = {fm(20.785 / 6.0, 3)} + {fm(20.785 / 9.0, 3)} = {fm(round(20.785 / 6.0, 3) + round(20.785 / 9.0, 3), 3)} s")
A("t4 chk1", f"432 / 12 + 432 / 18 = {fm(432 / 12)} + {fm(432 / 18)} = {fm(432 / 12 + 432 / 18)} m")
A("t4 chk2", f"60 / 5.773 = {fm(60 / 5.773, 2)} m/s")
A("t4 chk3", f"20.785 / 2 = {fm(20.785 / 2, 2)} m/s")
A("t4 w", f"48.0 + 32.0 = {fm(48.0 + 32.0)} m")

# ================================================== 2.2b
B = lambda n, s: ex("B", n, s)
dh = sq(90 ** 2 + 60 ** 2)
B("t5 dh", f"√(90² + 60²) = √11700 = {fm(dh, 2)} m")
B("t5 th", f"t = 108.17 / 10.0 = {fm(108.17 / 10.0, 2)} s")
B("t5 tv", f"t = 20 / 3.0 = {fm(20 / 3.0, 2)} s")
B("t5 T", f"T = max(10.82, 6.67) = 10.82 s")
B("t5 sum", f"10.82 + 6.67 = {fm(10.82 + 6.67, 2)} s")
B("t5 vE", f"90 / 10.82 = {fm(90 / 10.82, 2)} m/s")
B("t5 vN", f"60 / 10.82 = {fm(60 / 10.82, 2)} m/s")
B("t5 vz", f"20 / 10.82 = {fm(20 / 10.82, 2)} m/s")
q = [r2(8.32 ** 2), r2(5.55 ** 2), r2(1.85 ** 2)]
B("t5 speed", f"√(8.32² + 5.55² + 1.85²) = √({fm(q[0], 2)} + {fm(q[1], 2)} + {fm(q[2], 2)}) = √{fm(sum(q), 2)} = {fm(sq(sum(q)), 2)} m/s")
B("t5 chk", f"√(8.32² + 5.55²) = √({fm(q[0], 2)} + {fm(q[1], 2)}) = √{fm(q[0] + q[1], 2)} = {fm(sq(q[0] + q[1]), 2)} m/s")
B("t5 w1", f"t = 40 / 3.0 = {fm(40 / 3.0, 2)} s")
B("t5 w2", f"108.17 / 13.33 = {fm(108.17 / 13.33, 2)} m/s")
q2 = r2(8.11 ** 2)
B("t5 w3", f"√(8.11² + 3.0²) = √({fm(q2, 2)} + 9.00) = √{fm(q2 + 9.0, 2)} = {fm(sq(q2 + 9.0), 2)} m/s")
B("t6 t", f"t = (8 − 20) / ({M}6.0) = {fm((8 - 20) / -6.0)} s")
B("t6 ds", f"Δs = ½ × (20 + 8) × 2.0 = {fm(0.5 * 28 * 2.0)} m")
B("t6 freed", f"42.0 − 28.0 = {fm(42.0 - 28.0)} m")
B("t6 extra", f"14.0 / 20 = {fm(14.0 / 20)} s")
B("t6 cruise", f"3.0 + 0.7 = {fm(3.0 + 0.7)} s")
B("t6 lap", f"4.0 + 3.7 + 2.0 + 5.0 + 2.0 = {fm(4.0 + 3.7 + 2.0 + 5.0 + 2.0)} s")
B("t6 gain", f"17.0 − 16.7 = {fm(17.0 - 16.7)} s")
B("t6 chk", f"40.0 + 20 × 3.7 + 28.0 + 40.0 + 28.0 = 40.0 + {fm(20 * 3.7)} + 28.0 + 40.0 + 28.0 = {fm(40.0 + 20 * 3.7 + 28.0 + 40.0 + 28.0)} m")
B("t6 naive", f"3.0 − 2.0 = {fm(3.0 - 2.0)} s")
B("t6 net", f"1.0 − 0.7 = {fm(1.0 - 0.7)} s")
B("t6 w1", f"t = (20 − 8) / 8.0 = {fm((20 - 8) / 8.0)} s")
B("t6 w2", f"Δs = ½ × (8 + 20) × 1.5 = {fm(0.5 * 28 * 1.5)} m")
B("t6 w3", f"28.0 − 21.0 = {fm(28.0 - 21.0)} m")
B("t6 w4", f"7.0 / 20 = {fm(7.0 / 20, 2)} s")
B("t6 w5", f"(2.0 − 1.5) − 0.35 = {fm((2.0 - 1.5) - 0.35, 2)} s")
B("t6 w6", f"17.0 − 0.15 = {fm(17.0 - 0.15, 2)} s")
v7 = round(g * 2.0, 3)
B("t7 v", f"v = g·t = 9.8062 × 2.0 = {fm(v7, 3)} m/s")
B("t7 dz", f"Δz = ½·g·t² = ½ × 9.8062 × 2.0² = {fm(0.5 * g * 4.0, 3)} m")
B("t7 slow", f"Δ = (v₀² − v²) / (2·a) = (19.612² − 5.0²) / (2 × 15) = ({fm(r2(19.612 ** 2), 2)} − 25.0) / 30 = {fm((r2(19.612 ** 2) - 25.0) / 30, 2)} m")
B("t7 h", f"19.612 + 11.99 = {fm(19.612 + 11.99, 2)} m")
B("t7 ts", f"(19.612 − 5.0) / 15 = {fm((19.612 - 5.0) / 15, 3)} s")
B("t7 T", f"2.0 + 0.974 = {fm(2.0 + 0.974, 3)} s")
t7b = sq(2 * 31.60 / g)
B("t7 free", f"√(2 × 31.60 / 9.8062) = {fm(t7b, 2)} s and end at {fm(g * round(t7b, 2), 1)} m/s")
B("t7 w1", f"25 − 19.612 = {fm(25 - 19.612, 2)} m")
B("t7 w2", f"v² = 19.612² − 2 × 15 × 5.39 = {fm(r2(19.612 ** 2), 2)} − {fm(2 * 15 * 5.39, 2)} = {fm(r2(19.612 ** 2) - 2 * 15 * 5.39, 2)}")
B("t7 w3", f"v = √222.93 = {fm(sq(222.93), 2)} m/s")
B("t8 V2", "V² = 25.0² × 1.2 = 750.0")
B("t8 V", f"V = √750.0 = {fm(sq(750.0), 2)} m/s")
B("t8 a", f"a = 2.5 / 1.2 = {fm(2.5 / 1.2, 3)} m/s²")
B("t8 s", f"s = V² / (2·a) = 750.0 / (2 × 2.5 / 1.2) = 750.0 / {fm(2 * 2.5 / 1.2, 4)} = {fm(750.0 / round(2 * 2.5 / 1.2, 4))} m")
B("t8 ratio", f"180.0 / 125.0 = {fm(180.0 / 125.0, 2)} = 1.2²")
B("t8 k", f"k = √(150 / 125.0) = √1.2 = {fm(sq(1.2), 4)}")
B("t8 pct", f"(1.0954 − 1) × 100 = {fm((1.0954 - 1) * 100)} %")
B("t8 chk", f"125.0 × 1.0954² = 125.0 × {fm(1.0954 ** 2, 4)} = {fm(125.0 * round(1.0954 ** 2, 4))} m")
B("t8 w1", f"a = 2.5 × 1.2 / 1.2 = {fm(2.5 * 1.2 / 1.2)} m/s²")
B("t8 w2", f"s = 750.0 / (2 × 2.5) = {fm(750.0 / 5.0)} m")
B("t9 coef", f"½ × 8.0 × 3.0 = {fm(0.5 * 8.0 * 3.0)}")
B("t9 e1", f"12.0 × 0.020 = {fm(12.0 * 0.020, 2)} m")
B("t9 e2", f"12.0 × 0.003 = {fm(12.0 * 0.003, 3)} m")
B("t9 dt", f"Δt ≤ 0.05 / 12.0 = {fm(0.05 / 12.0, 6)} s")
B("t9 rate", f"1 / 0.004167 = {fm(1 / 0.004167)} steps per second")
B("t9 chk1", f"(24.0 + 16.0 + 8.0) × 1.0 = {fm((24.0 + 16.0 + 8.0) * 1.0)} m")
B("t9 chk2", f"48.0 − 36.0 = {fm(48.0 - 36.0)} m")
B("t9 w", f"at {fm(36.0 - 0.24, 2)} m instead of 36.0 m")
Om = 7.292115e-5


def sci(x, d=3, p=3):                                 # 2.917 × 10⁻³
    e = int(math.floor(math.log10(abs(x))))
    mant = x / 10 ** e
    sup = {"0": "⁰", "1": "¹", "2": "²", "3": "³", "4": "⁴", "5": "⁵", "6": "⁶", "7": "⁷", "8": "⁸", "9": "⁹", "-": "⁻"}
    return f"{mant:.{d}f} × 10" + "".join(sup[c] for c in str(e))


aC = 2 * Om * 20
B("t10 a", f"2 × Ω × v = 2 × 7.292115 × 10⁻⁵ × 20 = {sci(aC)} m/s²")
B("c a", f"2 × Ω × v = 2 × 7.292115 × 10⁻⁵ × 20 = {sci(aC)} m/s²")
B("c e17", f"½ × 0.002917 × 17² = ½ × 0.002917 × 289 = {fm(0.5 * 0.002917 * 289, 2)} m")
B("c e300", f"½ × 0.002917 × 300² = ½ × 0.002917 × 90000 = {fm(0.5 * 0.002917 * 90000, 0)} m")
B("c tr", f"v² / R = 20² / 6 378 137 = {sci(400 / 6378137)} m/s²")
B("c tr17", f"½ × 0.00006271 × 289 = {fm(0.5 * 0.00006271 * 289, 4)} m")
B("c tr300", f"½ × 0.00006271 × 90000 = {fm(0.5 * 0.00006271 * 90000, 1)} m")
B("c ratio", f"It is about {round(0.002917 / 0.00006271)} times smaller than the Coriolis term")
B("c g", "about 0.0003 times g")
B("t10 t", f"t = √(2·E / a) = √(2 × 1.5 / 0.002917) = √{fm(3.0 / 0.002917, 1)} = {fm(sq(round(3.0 / 0.002917, 1)), 2)} s")
B("t10 x", f"20 × 32.07 = {fm(20 * 32.07, 0)} m")
B("t10 d", f"δ = b / g = 0.002917 / 9.8062 = {sci(0.002917 / g)} rad")
B("t10 deg", f"2.975 × 10⁻⁴ × 57.30 = {fm(2.975e-4 * 57.30, 3)}°")
B("t10 c10", f"½ × 0.002917 × 10² = {fm(0.5 * 0.002917 * 100, 3)} m")
B("t10 b10", f"½ × 0.04 × 10² = {fm(0.5 * 0.04 * 100, 2)} m")
B("t10 ratio", f"2.00 / 0.146 = {fm(2.00 / 0.146, 1)}")
B("t10 chk1", f"2 × 32.07 = {fm(2 * 32.07, 2)} s")
B("t10 chk2", f"4 × 1.5 = {fm(4 * 1.5)} m")
B("t10 chk3", f"½ × 0.002917 × 64.14² = {fm(0.5 * 0.002917 * 64.14 ** 2)} m")
B("t10 w1", f"2 × 7.292115 × 10⁻⁵ × 40 = {sci(2 * Om * 40)} m/s²")
B("t10 w2", f"32.07 / √2 = {fm(32.07 / sq(2), 2)} s")
B("t10 w3", f"40 × 22.68 = {fm(40 * 22.68, 0)} m")
B("fn", "It is common practice to neglect the Earth’s rotation rate")
B("fn2", "might become directly measurable")
# ---- Think 10a (Vial et al.: the Earth rate on the gyroscope). Every step is recomputed from the rounded inputs that the page prints.
B("t10a Om", f"7.292115 × 10⁻⁵ × 57.30 = {sci(Om * 57.30)} °/s")
B("t10a Oh", f"4.178 × 10⁻³ × 3600 = {fm(4.178e-3 * 3600, 2)} °/h")
B("t10a Od", f"4.178 × 10⁻³ × 0.6393 = {sci(4.178e-3 * 0.6393)} °/s")
B("t10a Odh", f"15.04 × 0.6393 = {fm(15.04 * 0.6393, 2)} °/h")
B("t10a h17", f"2.671 × 10⁻³ × 17 = {fm(2.671e-3 * 17, 3)}°")
B("t10a h300", f"2.671 × 10⁻³ × 300 = {fm(2.671e-3 * 300, 2)}°")
B("t10a h3600", f"2.671 × 10⁻³ × 3600 = {fm(2.671e-3 * 3600, 2)}°")
B("t10a h3600 chk", f"2.671 × 10⁻³ × 3600 = {fm(2.671e-3 * 3600, 2)} °/h")
B("t10a vial", f"2.78 × 10⁻³ × 3600 = {fm(2.78e-3 * 3600, 1)}°")
B("t10a eps", f"2.671 × 10⁻³ / 57.30 = {sci(2.671e-3 / 57.30)} rad/s")
B("t10a aeq", f"20 × 4.661 × 10⁻⁵ = {sci(20 * 4.661e-5)} m/s²")
B("t10a pos", f"½ × 9.322 × 10⁻⁴ × 300² = ½ × 9.322 × 10⁻⁴ × 90000 = {fm(0.5 * 9.322e-4 * 90000, 1)} m")
B("t10a epsB", f"0.020 / 57.30 = {sci(0.020 / 57.30)} rad/s")
B("t10a aeqB", f"20 × 3.490 × 10⁻⁴ = {sci(20 * 3.490e-4)} m/s²")
B("t10a posB", f"½ × 6.980 × 10⁻³ × 90000 = {fm(0.5 * 6.980e-3 * 90000, 0)} m")
B("t10a ratio", f"0.020 / 0.002671 = {fm(0.020 / 0.002671, 2)}")
B("t10a w1", f"4.178 × 10⁻³ × 0.8660 = {sci(4.178e-3 * 0.8660)} °/s")
B("t10a w2", f"3.618 × 10⁻³ × 300 = {fm(3.618e-3 * 300, 2)}°")
B("t10a pole", "at the pole (φ = 90°) it is the whole 4.178 × 10⁻³ °/s")
# ---- Think 10b (the error state: zero mean, growing covariance, one GPS fix)
Kb = 0.025 / (0.025 + 0.010)
B("t10b Vi", f"0.5² × 0.01² = {sci(0.5 ** 2 * 0.01 ** 2, 1)} (m/s)²")
B("t10b P1", f"100 × 2.5 × 10⁻⁵ = {sci(100 * 2.5e-5, 1)} (m/s)²")
B("t10b s1", f"√(2.5 × 10⁻³) = {fm(sq(2.5e-3), 4)} m/s")
B("t10b P10", f"1000 × 2.5 × 10⁻⁵ = {sci(1000 * 2.5e-5, 1)} (m/s)²")
B("t10b s10", f"√(2.5 × 10⁻²) = {fm(sq(2.5e-2), 4)} m/s")
B("t10b K", f"K = P / (P + V) = 0.025 / (0.025 + 0.010) = {fm(Kb, 4)}")
B("t10b sur", f"12.10 − 12.30 = {fm(12.10 - 12.30, 2)} m/s")
B("t10b dv", f"δv̂ = K × (12.10 − 12.30) = 0.7143 × ({M}0.20) = {fm(0.7143 * -0.20, 4)} m/s")
B("t10b v", f"v ← 12.30 + ({M}0.1429) = {fm(12.30 - 0.1429, 3)} m/s")
B("t10b P", f"P ← (1 − K)·P = (1 − 0.7143) × 0.025 = {sci((1 - 0.7143) * 0.025, 2)} (m/s)²")
B("t10b sP", f"√(7.14 × 10⁻³) = {fm(sq(7.14e-3), 4)} m/s")
B("t10b chk1", f"√(1000 / 100) = {fm(sq(10), 3)}")
B("t10b chk2", f"1 / (1/0.025 + 1/0.010) = 1 / (40 + 100) = {sci(1 / 140, 2)}")
B("t10b w1", f"K = 0.025 / (0.025 + 0.25) = {fm(0.025 / (0.025 + 0.25), 4)}")
B("t10b w2", f"0.0909 × ({M}0.20) = {fm(0.0909 * -0.20, 4)} m/s")
B("t10b w3", f"12.30 + ({M}0.0182) = {fm(12.30 - 0.0182, 3)} m/s")
B("t10b w4", f"(1 − 0.0909) × 0.025 = {sci((1 - 0.0909) * 0.025, 2)} (m/s)²")
# ---- Think 10c (one SGal(3) step as a matrix product; the numbers follow the block rules R·ΔR, v + R·Δv, p + v·Δt + R·Δp, t + Δt)
Rz90 = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
Rdv = [sum(Rz90[i][k] * (1.0, 0.0, 0.0)[k] for k in range(3)) for i in range(3)]
Rdp = [sum(Rz90[i][k] * (0.25, 0.0, 0.0)[k] for k in range(3)) for i in range(3)]
B("t10c dv", f"Δv = a·Δt = 2 × 0.5 = {fm(2 * 0.5, 1)} m/s")
B("t10c dp", f"Δp = ½·a·Δt² = ½ × 2 × 0.5² = {fm(0.5 * 2 * 0.5 ** 2, 2)} m")
B("t10c v1", f"v₁ = 0 + 1.0 = {fm(0 + 1.0, 1)} m/s")
B("t10c p1", f"p₁ = 0 + 0 × 0.5 + 0.25 = {fm(0 + 0 * 0.5 + 0.25, 2)} m")
B("t10c t1", f"t₁ = 0 + 0.5 = {fm(0 + 0.5, 1)} s")
B("t10c v2", f"v₂ = 1.0 + 1.0 = {fm(1.0 + 1.0, 1)} m/s")
B("t10c p2", f"p₂ = 0.25 + 1.0 × 0.5 + 0.25 = {fm(0.25 + 1.0 * 0.5 + 0.25, 2)} m")
B("t10c t2", f"t₂ = 0.5 + 0.5 = {fm(0.5 + 0.5, 1)} s")
B("t10c ex x", f"x = ½·a·t² = ½ × 2 × 1.0² = {fm(0.5 * 2 * 1.0 ** 2, 2)} m")
B("t10c ex v", f"v = a·t = 2 × 1.0 = {fm(2 * 1.0, 1)} m/s")
B("t10c v3", f"v₃ = 2.0 + 1.0 = {fm(2.0 + 1.0, 1)} m/s")
B("t10c p3", f"p₃ = 1.00 + 2.0 × 0.5 + 0.25 = {fm(1.00 + 2.0 * 0.5 + 0.25, 2)} m")
B("t10c p4", f"p₄ = 2.25 + 3.0 × 0.5 + 0.25 = {fm(2.25 + 3.0 * 0.5 + 0.25, 2)} m")
B("t10c chk1", f"a·Δt = 2 × 1.0 = {fm(2 * 1.0, 1)} m/s")
B("t10c chk2", f"½·a·Δt² = ½ × 2 × 1.0² = {fm(0.5 * 2 * 1.0 ** 2, 2)} m")
B("t10c Rdv", f"v = R·Δv = ({fm(Rdv[0], 0)}, {fm(Rdv[1], 1)}, {fm(Rdv[2], 0)}) m/s")
B("t10c Rdp", f"p = R·Δp = ({fm(Rdp[0], 0)}, {fm(Rdp[1], 2)}, {fm(Rdp[2], 0)}) m")

# ================================================== 2.2c
C = lambda n, s: ex("C", n, s)
C("t11 border", f"A² / J = 3.0² / 4.0 = {fm(9.0 / 4.0, 2)} m/s")
C("t11 T", f"T = V / A + A / J = 2.25 / 3.0 + 0.75 = {fm(2.25 / 3.0 + 0.75)} s")
C("t11 T2", f"2·A / J = 2 × 3.0 / 4.0 = {fm(2 * 3.0 / 4.0)} s")
C("t11 V1", f"V = (T − A / J) × A = (2.75 − 0.75) × 3.0 = {fm((2.75 - 0.75) * 3.0)} m/s")
C("t11 V2", f"V = J × (T / 2)² = 4.0 × 0.5² = 4.0 × 0.25 = {fm(4.0 * 0.25)} m/s")
C("t11 amax", f"J × T / 2 = 4.0 × 1.0 / 2 = {fm(4.0 * 1.0 / 2)} m/s²")
C("t11 chk1", f"6.0 / 3.0 + 0.75 = {fm(6.0 / 3.0 + 0.75, 2)} s")
C("t11 chk2", f"2 × √(1.0 / 4.0) = 2 × 0.5 = {fm(2 * sq(1.0 / 4.0))} s")
C("t11 w1", f"(1.5 − 0.75) × 3.0 = {fm((1.5 - 0.75) * 3.0, 2)} m/s")
C("t11 w2", f"4.0 × 0.75² = {fm(4.0 * 0.75 ** 2, 2)} m/s")
C("t12 one", f"x = V·T / 2 = 6.0 × 2.75 / 2 = {fm(6.0 * 2.75 / 2, 2)} m")
C("t12 two", f"2 × 8.25 = {fm(2 * 8.25)} m")
C("t12 quad", f"V² + 2.25 × V − 36.0 = 0")
C("t12 V", f"V = ½ × (−2.25 + √(2.25² + 4 × 36.0)) = ½ × (−2.25 + √{fm(2.25 ** 2 + 4 * 36.0, 4)}) = ½ × (−2.25 + {fm(sq(149.0625), 3)}) = {fm(0.5 * (-2.25 + round(sq(149.0625), 3)), 2)} m/s")
C("t12 cond", "4.98 > 2.25")
C("t12 half", f"Thalf = V / A + A / J = 4.98 / 3.0 + 0.75 = {fm(4.98 / 3.0, 2)} + 0.75 = {fm(4.98 / 3.0 + 0.75, 2)} s")
C("t12 total", f"2 × 2.41 = {fm(2 * 2.41, 2)} s")
C("t12 chk", f"4.98 × 2.41 / 2 = {fm(4.98 * 2.41 / 2, 2)} m")
C("t12 w", f"2 × 8.25 = {fm(2 * 8.25)} m")
C("t13 d3", f"d = 6.0² / (2 × 3.0) + (2 × 3.0 / 4.0) × 6.0 = {fm(36 / 6.0)} + {fm(1.5 * 6.0)} = {fm(36 / 6.0 + 1.5 * 6.0)} m")
C("t13 d2", f"d = 6.0² / (2 × 2.0) + (2 × 2.0 / 4.0) × 6.0 = {fm(36 / 4.0)} + {fm(1.0 * 6.0)} = {fm(36 / 4.0 + 1.0 * 6.0)} m")
C("t13 d4", f"d = 6.0² / (2 × 4.0) + (2 × 4.0 / 4.0) × 6.0 = {fm(36 / 8.0)} + {fm(2.0 * 6.0)} = {fm(36 / 8.0 + 2.0 * 6.0)} m")
C("t13 A", f"A* = √(V × J / 4) = √(6.0 × 4.0 / 4) = √6.0 = {fm(sq(6.0), 3)} m/s²")
C("t13 dmin", f"d = V² / A* = 36.0 / 2.449 = {fm(36.0 / 2.449, 2)} m")
C("t13 prod", f"2.0 × 3.0 = {fm(2.0 * 3.0)} = A*²")
C("t13 w1", f"6.0 + (2 × 3.0 / 8.0) × 6.0 = 6.0 + {fm(0.75 * 6.0)} = {fm(6.0 + 0.75 * 6.0)} m")
C("t13 w2", f"9.0 + (2 × 2.0 / 8.0) × 6.0 = 9.0 + {fm(0.5 * 6.0)} = {fm(9.0 + 0.5 * 6.0)} m")
C("t14 N", f"N = 15 − 35 = {M}20")
C("t14 E", "E = 40 − 10 = 30")
C("t14 D", f"D = −(8 − 3) = {M}5")
C("t14 dist", f"√(({M}20)² + 30² + ({M}5)²) = √(400 + 900 + 25) = √1325 = {fm(sq(1325), 2)} m")
k14 = 5.0 / 36.40
C("t14 v", f"5.0 × ({M}20, 30, {M}5) / 36.40 = ({fm(-20 * k14, 2)}, {fm(30 * k14, 2)}, {fm(-5 * k14, 2)}) m/s")
C("t14 t", f"36.40 / 5.0 = {fm(36.40 / 5.0, 2)} s")
C("t14 clip", f"clip(5.0 × 0.5, 1, 3) = {fm(min(max(5.0 * 0.5, 1), 3))} m")
C("t14 T", f"(36.40 − 2.5) / 5.0 + 2.5 / 1.0 = {fm((36.40 - 2.5) / 5.0, 2)} + 2.5 = {fm((36.40 - 2.5) / 5.0 + 2.5, 2)} s")
C("t14 chk1", f"2.5 / 1.0 − 2.5 / 5.0 = 2.5 − 0.5 = {fm(2.5 / 1.0 - 2.5 / 5.0)} s")
C("t14 chk2", f"7.28 + 2.0 = {fm(7.28 + 2.0, 2)} s")
C("t14 w", f"36.40 / 2.5 = {fm(36.40 / 2.5, 2)} s")

# ================================================== 2.2d
D = lambda n, s: ex("D", n, s)
D("t15 w", f"w = (6 × sin 60°, 6 × cos 60°) = ({fm(6 * math.sin(math.radians(60)), 3)}, {fm(6 * math.cos(math.radians(60)))}) m/s")
D("t15 v", f"v = (0 + 5.196, 12 + 3.0) = (5.196, {fm(12 + 3.0)}) m/s")
D("t15 speed", f"|v| = √(5.196² + 15.0²) = √({fm(5.196 ** 2)} + {fm(15.0 ** 2)}) = √252 = {fm(sq(252), 2)} m/s")
D("t15 ang", f"arctan(5.196 / 15.0) = {fm(deg(math.atan(5.196 / 15.0)))}°")
D("t15 t", f"t = √(2·h / g) = √(2 × 40 / 9.8062) = √{fm(80 / g, 4)} = {fm(sq(round(80 / g, 4)), 3)} s")
D("t15 N", f"15.0 × 2.856 = {fm(15.0 * 2.856, 2)} m")
D("t15 E", f"5.196 × 2.856 = {fm(5.196 * 2.856, 2)} m")
D("t15 tot", f"√(42.84² + 14.84²) = √({fm(42.84 ** 2)} + {fm(14.84 ** 2)}) = √{fm(r2(42.84 ** 2) * 0 + round(42.84 ** 2, 1) + round(14.84 ** 2, 1))} = {fm(sq(round(42.84 ** 2, 1) + round(14.84 ** 2, 1)), 2)} m")
D("t15 vz", f"g·t = 9.8062 × 2.856 = {fm(g * 2.856, 2)} m/s")
D("t15 vt", f"√(252 + 28.01²) = √(252 + {fm(r2(28.01 ** 2), 2)}) = √{fm(252 + r2(28.01 ** 2), 2)} = {fm(sq(252 + r2(28.01 ** 2)), 2)} m/s")
D("t15 chk", f"√252 × 2.856 = {fm(sq(252) * 2.856, 2)} m")
D("t16 h", f"10 × cos 30° = {fm(10 * math.cos(math.radians(30)), 3)} m/s")
D("t16 vz", f"10 × sin 30° = {fm(10 * math.sin(math.radians(30)))} m/s")
D("t16 vn", f"8.660 + 5.0 = {fm(8.660 + 5.0, 3)} m/s")
D("t16 root", f"√(5.0² + 2 × 9.8062 × 25) = √{fm(25 + 2 * g * 25, 2)} = {fm(sq(round(25 + 2 * g * 25, 2)), 2)}")
D("t16 t", f"t = (5.0 + 22.70) / 9.8062 = {fm((5.0 + 22.70) / g, 3)} s")
D("t16 x", f"13.660 × 2.825 = {fm(13.660 * 2.825, 2)} m")
D("t16 x0", f"8.660 × 2.825 = {fm(8.660 * 2.825, 2)} m")
D("t16 dx", f"38.59 − 24.46 = {fm(38.59 - 24.46, 2)} m")
D("t16 top", f"5.0² / (2 × 9.8062) = 25.0 / {fm(2 * g, 4)} = {fm(25.0 / (2 * g), 2)} m")
D("t16 ztop", f"25 + 1.27 = {fm(25 + 1.27, 2)} m")
D("t16 ttop", f"5.0 / 9.8062 = {fm(5.0 / g, 3)} s")
D("t16 imp", f"√(13.660² + 22.70²) = √({fm(r2(13.660 ** 2), 2)} + {fm(r2(22.70 ** 2), 2)}) = √{fm(r2(13.660 ** 2) + r2(22.70 ** 2), 2)} = {fm(sq(r2(13.660 ** 2) + r2(22.70 ** 2)), 2)} m/s")
D("t16 chk", f"5.0 − 9.8062 × 2.825 = 5.0 − {fm(g * 2.825, 2)} = {fm(5.0 - round(g * 2.825, 2), 2)} m/s")
D("t16 w1", f"8.660 − 5.0 = {fm(8.660 - 5.0, 3)} m/s")
D("t16 w2", f"3.660 × 2.825 = {fm(3.660 * 2.825, 2)} m")
# Think 17: the lap
D("t17 w", f"w = (6 × sin 120°, 6 × cos 120°) = ({fm(6 * math.sin(math.radians(120)), 2)}, {fm(6 * math.cos(math.radians(120)), 2)}) m/s")
c1, c2 = round(sq(15 ** 2 - 5.20 ** 2), 2), round(sq(15 ** 2 - 3.00 ** 2), 2)
D("t17 r1", f"√(15² − 5.20²) − 3.00 = {fm(c1, 2)} − 3.00 = {fm(c1 - 3.00, 2)}")
D("t17 r1t", f"300 / {fm(c1 - 3.00, 2)} = {fm(300 / (c1 - 3.00), 2)}")
D("t17 r2", f"√(15² − 3.00²) + 5.20 = {fm(c2, 2)} + 5.20 = {fm(c2 + 5.20, 2)}")
D("t17 r2t", f"300 / {fm(c2 + 5.20, 2)} = {fm(300 / (c2 + 5.20), 2)}")
D("t17 r3", f"{fm(c1, 2)} + 3.00 = {fm(c1 + 3.00, 2)}")
D("t17 r3t", f"300 / {fm(c1 + 3.00, 2)} = {fm(300 / (c1 + 3.00), 2)}")
D("t17 r4", f"{fm(c2, 2)} − 5.20 = {fm(c2 - 5.20, 2)}")
D("t17 r4t", f"300 / {fm(c2 - 5.20, 2)} = {fm(300 / (c2 - 5.20), 2)}")
ts = [round(300 / (c1 - 3.00), 2), round(300 / (c2 + 5.20), 2), round(300 / (c1 + 3.00), 2), round(300 / (c2 - 5.20), 2)]
D("t17 lap", f"{fm(ts[0], 2)} + {fm(ts[1], 2)} + {fm(ts[2], 2)} + {fm(ts[3], 2)} = {fm(sum(ts), 2)} s")
D("t17 still", f"4 × 300 / 15 = {fm(4 * 300 / 15)} s")
D("t17 loss", f"{fm(sum(ts), 2)} − 80.0 = {fm(sum(ts) - 80.0, 2)} s")
D("t17 pct", f"{fm(sum(ts) - 80.0, 2)} / 80.0 = {round((sum(ts) - 80.0) / 80.0 * 100)} %")
a1, a2 = deg(math.asin(5.20 / 15)), deg(math.asin(3.00 / 15))
D("t17 n1", f"0° − {fm(a1)}° = {fm((0 - round(a1, 1)) % 360)}°")
D("t17 n2", f"90° − {fm(a2)}° = {fm(90 - round(a2, 1))}°")
D("t17 n3", f"180° + {fm(a1)}° = {fm(180 + round(a1, 1))}°")
D("t17 n4", f"270° + {fm(a2)}° = {fm(270 + round(a2, 1))}°")
D("t17 as1", f"arcsin(5.20 / 15) = {fm(a1)}°")
D("t17 as2", f"arcsin(3.00 / 15) = {fm(a2)}°")
D("t17 p1", f"{fm(ts[0], 2)} + {fm(ts[2], 2)} = {fm(ts[0] + ts[2], 2)} s")
D("t17 p2", f"{fm(ts[1], 2)} + {fm(ts[3], 2)} = {fm(ts[1] + ts[3], 2)} s")
D("t17 p3", f"2 × 300 / 15 = {fm(2 * 300 / 15)} s")
D("t17 w1", f"(15 × sin 120°, 15 × cos 120°) = ({fm(15 * math.sin(math.radians(120)), 2)}, {fm(15 * math.cos(math.radians(120)), 2)})")
D("t17 w2", f"√(15² − 7.50²) = √{fm(225 - 56.25, 2)} = {fm(sq(168.75), 2)}")
D("t17 w3", f"{fm(12.99 - 12.99, 2)} m/s")
D("t17 chk", f"14.07 − 3.00 = {fm(14.07 - 3.00, 2)} m/s")
# Think 18: the wind from two ground speeds
q1, q2 = r2(20.22 ** 2), r2(22.29 ** 2)
D("t18 A", f"wE² + (15 + wN)² = 20.22² = {fm(q1, 2)}")
D("t18 B", f"(15 + wE)² + wN² = 22.29² = {fm(q2, 2)}")
D("t18 diff", f"30 × wN − 30 × wE = {fm(q1, 2)} − {fm(q2, 2)} = {fm(q1 - q2, 2)}")
dN = round((q1 - q2) / 30, 3)
D("t18 d", f"wN − wE = {fm(q1 - q2, 2)} / 30 = {fm(dN, 3)}")
qq = 15 + dN
D("t18 sub", f"wE² + ({fm(qq, 3)} + wE)² = 2 × wE² + {fm(2 * qq, 3)} × wE + {fm(r2(qq ** 2), 2)}")
Cc = r2(qq ** 2) - q1
D("t18 quad", f"2 × wE² + {fm(2 * qq, 3)} × wE − {fm(-Cc, 2)} = 0")
disc = (2 * qq) ** 2 - 8 * Cc
D("t18 roots", f"wE = (−{fm(2 * qq, 3)} ± √({fm(2 * qq, 3)}² + 8 × {fm(-Cc, 2)})) / 4 = (−{fm(2 * qq, 3)} ± √{fm(disc, 2)}) / 4 = (−{fm(2 * qq, 3)} ± {fm(sq(round(disc, 2)), 3)}) / 4")
rt = sq(round(disc, 2))
wE1, wE2 = round((-2 * qq + round(rt, 3)) / 4, 3), round((-2 * qq - round(rt, 3)) / 4, 3)
D("t18 wE1", f"wE = {fm(wE1, 3)}")
D("t18 wE2", f"wE = {fm(wE2, 3)}")
wN1, wN2 = round(wE1 + dN, 3), round(wE2 + dN, 3)
D("t18 wN1", f"wN = {fm(wE1, 3)} − 2.933 = {fm(wN1, 3)}")
D("t18 wN2", f"wN = {fm(wE2, 3)} − 2.933 = {fm(wN2, 3)}")
s1 = r2(wE1 ** 2) + r2(wN1 ** 2)
D("t18 s1", f"√(6.929² + 3.996²) = √({fm(r2(wE1 ** 2), 2)} + {fm(r2(wN1 ** 2), 2)}) = √{fm(s1, 2)} = {fm(sq(s1), 2)} m/s")
D("t18 b1", f"arctan(6.929 / 3.996) = {fm(deg(math.atan(wE1 / wN1)))}°")
s2 = r2(wE2 ** 2) + r2(wN2 ** 2)
D("t18 s2", f"√(18.996² + 21.929²) = √({fm(r2(wE2 ** 2), 2)} + {fm(r2(wN2 ** 2), 2)}) = √{fm(s2, 2)} = {fm(sq(s2), 2)} m/s")
D("t18 b2", f"180° + arctan(18.996 / 21.929) = 180° + {fm(deg(math.atan(18.996 / 21.929)))}°")
c3a = r2(wE1 ** 2) + r2((wN1 - 15) ** 2)
D("t18 c1", f"√(6.929² + (3.996 − 15)²) = √({fm(r2(wE1 ** 2), 2)} + {fm(r2((wN1 - 15) ** 2), 2)}) = √{fm(c3a, 2)} = {fm(sq(c3a), 2)} m/s")
c3b = r2(wE2 ** 2) + r2((wN2 - 15) ** 2)
D("t18 c2", f"√(18.996² + ({M}21.929 − 15)²) = √({fm(r2(wE2 ** 2), 2)} + {fm(r2((wN2 - 15) ** 2), 2)}) = √{fm(c3b, 2)} = {fm(sq(c3b), 2)} m/s")
ca = r2(wE1 ** 2) + r2((15 + wN1) ** 2)
D("t18 k1", f"√(6.929² + (15 + 3.996)²) = √({fm(r2(wE1 ** 2), 2)} + {fm(r2((15 + wN1) ** 2), 2)}) = √{fm(ca, 2)} = {fm(sq(ca), 2)} m/s")
cb = r2((15 + wE1) ** 2) + r2(wN1 ** 2)
D("t18 k2", f"√((15 + 6.929)² + 3.996²) = √({fm(r2((15 + wE1) ** 2), 2)} + {fm(r2(wN1 ** 2), 2)}) = √{fm(cb, 2)} = {fm(sq(cb), 2)} m/s")
cw = r2((wE1 - 15) ** 2) + r2(wN1 ** 2)
D("t18 w", f"√((6.929 − 15)² + 3.996²) = √({fm(r2((wE1 - 15) ** 2), 2)} + {fm(r2(wN1 ** 2), 2)}) = √{fm(cw, 2)} = {fm(sq(cw), 2)} m/s")
# Think 19: runways
D("t19 f", f"√1.05 = {fm(sq(1.05), 4)}")
D("t19 V", f"V = 27.0 × 1.0247 = {fm(27.0 * 1.0247, 2)} m/s")
D("t19 a", f"a = 2.141 / 1.05 = {fm(2.141 / 1.05, 3)} m/s²")
D("t19 2a", f"2·a = 2 × 2.039 = {fm(2 * 2.039, 3)}")
D("t19 hw", f"8.0 × cos 30° = {fm(8.0 * math.cos(math.radians(30)), 2)} m/s")
D("t19 cw", f"8.0 × sin 30° = {fm(8.0 * math.sin(math.radians(30)), 2)} m/s")
D("t19 hw2", f"8.0 × cos 150° = {fm(8.0 * math.cos(math.radians(150)), 2)} m/s")
D("t19 cw2", f"8.0 × sin 150° = {fm(8.0 * math.sin(math.radians(150)), 2)} m/s")
for nm, gs, den, lbl in (("1", 27.67 - 6.93, 4.078, "27.67 − 6.93"), ("2", 27.67 + 6.93, 4.078, "27.67 + 6.93"), ("4", 25.0 - 6.93, 4.0, "25.0 − 6.93"), ("5", 25.0 + 6.93, 4.0, "25.0 + 6.93")):
    gs = round(gs, 2)
    D(f"t19 row{nm}", f"{lbl} = {fm(gs, 2)}")
    D(f"t19 roll{nm}", f"{fm(gs, 2)}² / {fm(den, 3) if den != 4.0 else '4.0'} = {fm(r2(gs ** 2), 2)} / {fm(den, 3) if den != 4.0 else '4.0'} = {fm(r2(gs ** 2) / den, 2)}")
D("t19 row3", "27.67")
D("t19 roll3", f"27.67² / 4.078 = {fm(r2(27.67 ** 2), 2)} / 4.078 = {fm(r2(27.67 ** 2) / 4.078, 2)}")
D("t19 need", f"√(2·a·150) = √(4.078 × 150) = √{fm(4.078 * 150, 2)} = {fm(sq(611.70), 2)} m/s")
D("t19 hneed", f"27.67 − 24.73 = {fm(27.67 - 24.73, 2)} m/s")
D("t19 wneed", f"2.94 / cos 30° = 2.94 / 0.866 = {fm(2.94 / 0.866, 2)} m/s")
D("t19 w1", f"27.67 − 3.00 = {fm(27.67 - 3.00, 2)} m/s")
D("t19 w2", f"24.67² / 4.078 = {fm(r2(24.67 ** 2), 2)} / 4.078 = {fm(r2(24.67 ** 2) / 4.078, 2)} m")
# Think 20: stop over the pad
D("t20 W", f"W = 3.5 × 9.8062 = {fm(3.5 * g, 3)} N")
D("t20 v", f"v = 12 + 6 = {12 + 6} m/s")
D("t20 ah", f"18² / (2 × 20.0) = 324 / 40.0 = {fm(324 / 40.0, 2)} m/s²")
D("t20 t", f"t = 2·d / v = 2 × 20.0 / 18 = {fm(2 * 20.0 / 18, 3)} s")
D("t20 aup", f"3.0 / 2.222 = {fm(3.0 / 2.222, 2)} m/s²")
D("t20 depth", f"½ × 3.0 × 2.222 = {fm(0.5 * 3.0 * 2.222, 2)} m")
inner = r2(8.10 ** 2) + r2((g + 1.35) ** 2)
D("t20 T", f"3.5 × √(8.10² + (9.8062 + 1.35)²) = 3.5 × √({fm(r2(8.10 ** 2), 2)} + {fm(r2((g + 1.35) ** 2), 2)}) = 3.5 × √{fm(inner, 2)} = {fm(3.5 * sq(inner), 2)} N")
D("t20 rotor", f"48.25 / 4 = {fm(48.25 / 4, 2)} N")
D("t20 share", f"12.06 / 15 = {fm(12.06 / 15 * 100)} %")
D("t20 hover", f"34.322 / 4 / 15 = {fm(34.322 / 4 / 15 * 100)} %")
D("t20 A3", f"18² / (2 × 3.0) = 324 / 6.0 = {fm(324 / 6.0)} m")
D("t20 vmax", f"√(2 × 3.0 × 20.0) = √{fm(2 * 3.0 * 20.0)} = {fm(sq(120.0), 2)} m/s")
D("t20 air", f"10.95 − 6.0 = {fm(10.95 - 6.0, 2)} m/s")
D("t20 chk1", f"48.25 / 34.322 = {fm(48.25 / 34.322, 3)}")
D("t20 chk2", f"12² / (2 × 3.0) = 144 / 6.0 = {fm(144 / 6.0)} m")
D("t20 w1", f"24² / (2 × 20.0) = {fm(24 ** 2 / 40.0, 2)} m/s²")
D("t20 w2", f"t = 2 × 20.0 / 24 = {fm(2 * 20.0 / 24, 3)} s")
D("t20 w3", f"3.0 / 1.667 = {fm(3.0 / 1.667, 2)} m/s²")
inner2 = r2(14.40 ** 2) + r2((g + 1.80) ** 2)
D("t20 w4", f"3.5 × √(14.40² + (9.8062 + 1.80)²) = 3.5 × √({fm(r2(14.40 ** 2), 2)} + {fm(r2((g + 1.80) ** 2), 2)}) = 3.5 × √{fm(inner2, 2)} = {fm(3.5 * sq(inner2), 2)} N")
# Think 21: the climb to the gate
D("t21 gv", f"(10 − 4.0, 3.0 + 1.5) = ({fm(10 - 4.0)}, {fm(3.0 + 1.5)}) m/s")
D("t21 t", f"t = 60 / 6.0 = {fm(60 / 6.0)} s")
D("t21 z", f"2 + 4.5 × 10.0 = {fm(2 + 4.5 * 10.0)} m")
D("t21 vz", f"18 / 10.0 = {fm(18 / 10.0)} m/s")
D("t21 air", f"1.8 − 1.5 = {fm(1.8 - 1.5)} m/s")
D("t21 chk", f"1.8 × 10.0 = {fm(1.8 * 10.0)} m")
D("t21 w1", f"1.8 − ({M}1.5) = {fm(1.8 + 1.5)} m/s")
D("t21 w2", f"3.0 + ({M}1.5) = {fm(3.0 - 1.5)} m/s")
D("t21 w3", f"18 / 1.5 = {fm(18 / 1.5)} s")
D("t21 w4", f"60 / 12.0 = {fm(60 / 12.0)} m/s")
D("t21 w5", f"5.0 + 4.0 = {fm(5.0 + 4.0)} m/s")
# Think 22: the gust
D("t22 p1", f"8.0 × 6.0 = {fm(8.0 * 6.0)}")
D("t22 p2", f"½ × (8.0 + 2.0) × 2.0 = {fm(0.5 * 10.0 * 2.0)}")
D("t22 p3", f"2.0 × 4.0 = {fm(2.0 * 4.0)}")
D("t22 p4", f"½ × (2.0 + 8.0) × 2.0 = {fm(0.5 * 10.0 * 2.0)}")
D("t22 p5", f"120 − 76.0 = {fm(120 - 76.0)} in 44.0 / 8.0 = {fm(44.0 / 8.0)} s")
D("t22 T", f"T = 14.0 + 5.5 = {fm(14.0 + 5.5)} s")
D("t22 plan", f"120 / 8.0 = {fm(120 / 8.0)} s")
D("t22 late", f"19.5 − 15.0 = {fm(19.5 - 15.0)} s")
D("t22 pos", f"76.0 + 8.0 × 1.0 = {fm(76.0 + 8.0)} m")
D("t22 short", f"120 − 84.0 = {fm(120 - 84.0)} m")
D("t22 avg", f"84.0 / 15.0 = {fm(84.0 / 15.0)} m/s")
D("t22 wind", f"12.0 − 5.6 = {fm(12.0 - 5.6)} m/s")
D("t22 w1", f"2.0 × 2.0 = {fm(2.0 * 2.0)} m")
D("t22 w2", f"48.0 + 10.0 + 4.0 + 10.0 = {fm(48.0 + 10.0 + 4.0 + 10.0)} m")
D("t22 w3", f"(120 − 72.0) / 8.0 = {fm((120 - 72.0) / 8.0)} s")
D("t22 w4", f"12.0 + 6.0 = {fm(12.0 + 6.0)} s")
D("t22 w5", f"18.0 − 15.0 = {fm(18.0 - 15.0)} s")
# Think 23: the bias
D("t23 t", f"t = √(2·E / b) = √(2 × 1.5 / 0.04) = √{fm(2 * 1.5 / 0.04)} = {fm(sq(75.0), 2)} s")
D("t23 x", f"10 × 8.66 = {fm(10 * 8.66)} m")
for tt in (4.0, 8.0, 12.0):
    D(f"t23 g{int(tt)}", f"½ × 0.04 × {fm(tt)}² = {fm(0.5 * 0.04 * tt ** 2, 2)} m")
D("t23 v", f"0.04 × 12.0 = {fm(0.04 * 12.0, 2)} m/s")
D("t23 tilt", f"δ = asin(b / g) = asin(0.04 / 9.8062) = asin({fm(0.04 / g, 6)}) = {fm(deg(math.asin(0.04 / g)), 3)}°".replace("0.004079", "0.004079"))
D("t23 b2", f"t = √(2 × 1.5 / 0.01) = √{fm(2 * 1.5 / 0.01)} = {fm(sq(300.0), 2)} s")
D("t23 ratio", f"17.32 / 8.66 = {fm(17.32 / 8.66)}")
D("t23 w1", f"½ × 0.01 × 16.0² = {fm(0.5 * 0.01 * 16.0 ** 2, 2)} m")
D("t23 w2", f"½ × 0.01 × 20.0² = {fm(0.5 * 0.01 * 20.0 ** 2, 2)} m")
# Think 24: the S-curve with a parcel
D("t24 T", f"T = 6.0 / 3.0 + 3.0 / 4.0 = {fm(6.0 / 3.0 + 3.0 / 4.0, 2)} s")
D("t24 x", f"x = 6.0 × 2.75 / 2 = {fm(6.0 * 2.75 / 2, 2)} m")
D("t24 two", f"2 × 8.25 = {fm(2 * 8.25)} m")
D("t24 cr", f"20.0 − 16.5 = {fm(20.0 - 16.5)} m")
D("t24 tcr", f"3.5 / 6.0 = {fm(3.5 / 6.0, 3)} s")
D("t24 tot", f"2 × 2.75 + 0.583 = {fm(2 * 2.75 + 0.583, 3)} s")
D("t24 thr", f"A² / J = 2.0² / 4.0 = {fm(2.0 ** 2 / 4.0)}")
D("t24 T2", f"T = V / A + A / J = 6.0 / 2.0 + 2.0 / 4.0 = {fm(6.0 / 2.0)} + {fm(2.0 / 4.0)} = {fm(6.0 / 2.0 + 2.0 / 4.0)} s")
D("t24 x2", f"V·T / 2 = 6.0 × 3.5 / 2 = {fm(6.0 * 3.5 / 2)} m")
D("t24 two2", f"2 × 10.5 = {fm(2 * 10.5)} m")
D("t24 quad", "V² + (A² / J) × V − 2 × A × 10.0 = V² + 1.0 × V − 40.0 = 0")
D("t24 V", f"V = ½ × (−1.0 + √(1.0² + 4 × 40.0)) = ½ × (−1.0 + √{fm(1.0 + 4 * 40.0)}) = ½ × (−1.0 + {fm(sq(161.0), 3)}) = {fm(0.5 * (-1.0 + round(sq(161.0), 3)), 2)} m/s")
D("t24 half", f"Thalf = 5.84 / 2.0 + 0.5 = {fm(5.84 / 2.0, 2)} + 0.5 = {fm(5.84 / 2.0 + 0.5, 2)} s")
D("t24 tot2", f"2 × 3.42 = {fm(2 * 3.42, 2)} s")
D("t24 lost", f"6.84 − 6.083 = {fm(6.84 - 6.083, 2)} s")
D("t24 chk", f"5.84 × 3.42 / 2 = {fm(5.84 * 3.42 / 2, 2)} m")
D("t24 b", f"b = 4 × 2.0² / 4.0 = {fm(4 * 2.0 ** 2 / 4.0)}")
D("t24 px4", f"½ × (−4.0 + √(4.0² + 8 × 2.0 × 10.0)) = ½ × (−4.0 + √{fm(16 + 160.0)}) = ½ × (−4.0 + {fm(sq(176.0), 3)}) = {fm(0.5 * (-4.0 + round(sq(176.0), 3)), 2)} m/s")
D("t24 w", f"2 × 10.5 = {fm(2 * 10.5)} m")
# Think 25: the best plan
for v in (16, 14, 12, 10):
    tu = v / 4.0
    xu = v ** 2 / (2 * 4.0)
    cr = 200 - 2 * xu
    tcr = cr / v
    dec = 3 if v in (14, 12) else 1
    tcr_s = fm(tcr, 3) if v in (14, 12) else fm(tcr, 1)
    D(f"t25 {v} tu", f"{v} / 4.0 = {fm(tu)}")
    D(f"t25 {v} xu", f"{v}² / (2 × 4.0) = {v ** 2} / 8.0 = {fm(xu)}")
    D(f"t25 {v} cr", f"200 − 2 × {fm(xu)} = {fm(cr)}")
    D(f"t25 {v} tcr", f"{fm(cr)} / {v} = {tcr_s}")
    tot_s = fm(2 * tu + round(tcr, 3), 3) if v in (14, 12) else fm(2 * tu + tcr, 1)
    D(f"t25 {v} T", f"2 × {fm(tu)} + {tcr_s} = {tot_s}")
D("t25 f1", f"16 / 4.0 + 200 / 16 = {fm(16 / 4.0)} + {fm(200 / 16)} = {fm(16 / 4.0 + 200 / 16)} s")
D("t25 f2", f"12 / 4.0 + 200 / 12 = {fm(12 / 4.0)} + {fm(200 / 12, 3)} = {fm(12 / 4.0 + 200 / 12, 3)} s")
D("t25 add", f"1 / a = 1 / 4.0 = {fm(1 / 4.0, 2)} s")
D("t25 save", f"L / v² = 200 / 16² = {fm(200 / 16 ** 2, 2)} s")
D("t25 vopt", f"v = √(a·L) = √(4.0 × 200) = √{fm(4.0 * 200, 0)} = {fm(sq(800), 2)} m/s")
D("t25 chk", f"32.0 + 136.0 + 32.0 = {fm(32.0 + 136.0 + 32.0)} m")
D("t25 w1", f"16 / 4.0 + 4.0 / 4.0 = {fm(16 / 4.0 + 4.0 / 4.0)} s")
D("t25 w2", f"16 × 5.0 / 2 = {fm(16 * 5.0 / 2)} m")
D("t25 w3", f"200 − 2 × 40.0 = {fm(200 - 2 * 40.0)} m")
D("t25 w4", f"120.0 / 16 = {fm(120.0 / 16)} s")
D("t25 w5", f"2 × 5.0 + 7.5 = {fm(2 * 5.0 + 7.5)} s")
D("t25 w6", f"17.5 − 16.5 = {fm(17.5 - 16.5)} s")
D("q2", f"90 / (15 − 5) + 90 / (15 + 5) = {fm(90 / 10)} + {fm(90 / 20)} = {fm(90 / 10 + 90 / 20)} s")
D("q2b", f"2 × 90 / 15 = {fm(2 * 90 / 15)} s")
D("q5", f"16² / 4.0 = {fm(16 ** 2 / 4.0)} m")

# ----- the printed results against the exact values of think_calc.py (a second computation), within a tolerance
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from think_calc import compute
    K = compute()
except Exception as e:                                # a failure here is reported, not hidden
    K = None
    print("think_calc could not be imported:", e)
tol_errors = []


def near(name, printed, exact, tol):
    if abs(printed - exact) > tol:
        tol_errors.append(f"{name}: printed {printed}, exact {exact}")


if K:
    near("k1 speed", 12.0, K["k1_speed"], 1e-9); near("k3 v", 10.58, K["k3_v"], 0.006); near("k5 T", 10.82, K["k5_T"], 0.006)
    near("k5 speed", 10.17, K["k5_speed"], 0.01); near("k6 T", 16.7, K["k6_T"], 1e-9); near("k7 h", 31.60, K["k7_h"], 0.006)
    near("k8 s", 180.0, K["k8_s"], 0.01); near("k11 V", 4.98, K["k11_V"], 0.006); near("k13 dist", 36.40, K["k13_dist"], 0.006)
    near("k13 t", 9.28, K["k13_t"], 0.006); near("k14 t", 2.856, K["k14_t"], 0.0006); near("k14 d", 45.34, K["k14_d"], 0.006)
    near("k15 t", 2.825, K["k15_t"], 0.0006); near("k15 x", 38.59, K["k15_x"], 0.006); near("k16 T", 91.33, K["k16_T"], 0.01)
    near("k17 s27", 105.48, K["k17_s27"], 0.01); near("k17 s09", 293.57, K["k17_s09"], 0.01); near("k18 T", 48.25, K["k18_T"], 0.01)
    near("k20 T", 19.5, K["k20_T"], 1e-9); near("k21 t", 8.66, K["k21_t_miss"], 0.006); near("k22 Vp", 5.84, K["k22_Vpeak"], 0.006)
    # Think 10a, 10b, 10c: the printed values against exact values (no rounded inputs; sums and matrix products instead of closed forms)
    near("k10a om", 4.178e-3, K["k10a_om_deg"], 1e-6); near("k10a down", 2.671e-3, K["k10a_down"], 1e-6)
    near("k10a h17", 0.045, K["k10a_h17"], 0.001); near("k10a h300", 0.80, K["k10a_h300"], 0.006); near("k10a h3600", 9.62, K["k10a_h3600"], 0.006)
    near("k10a eps", 4.661e-5, K["k10a_eps"], 3e-8); near("k10a pos", 41.9, K["k10a_pos"], 0.1); near("k10a pos sin", 41.9, K["k10a_pos_sin"], 0.2)
    near("k10a posB", 314.0, K["k10a_posB"], 0.5); near("k10a ratio", 7.49, K["k10a_ratio"], 0.01); near("k10a 60", 1.09, K["k10a_h300_60"], 0.006)
    near("k10b P1", 2.5e-3, K["k10b_P1"], 1e-9); near("k10b P10", 2.5e-2, K["k10b_P10"], 1e-9); near("k10b K", 0.7143, K["k10b_K"], 1e-4)
    near("k10b dv", -0.1429, K["k10b_dv"], 1e-4); near("k10b v", 12.157, K["k10b_v"], 0.001); near("k10b Pafter", 7.14e-3, K["k10b_Pafter"], 1e-5)
    near("k10b Pafter sum", 7.14e-3, K["k10b_Pafter_sum"], 1e-5); near("k10b Kw", 0.0909, K["k10b_Kw"], 1e-4); near("k10b vw", 12.282, K["k10b_vw"], 0.001)
    near("k10b Pw", 2.27e-2, K["k10b_Pw"], 1e-4)
    near("k10c dv", 1.0, K["k10c_S_dv"], 1e-9); near("k10c dp", 0.25, K["k10c_S_dp"], 1e-9); near("k10c dt", 0.5, K["k10c_S_dt"], 1e-9)
    for i_, (pv, pp, pt) in enumerate([(1.0, 0.25, 0.5), (2.0, 1.00, 1.0), (3.0, 2.25, 1.5), (4.0, 4.00, 2.0)]):
        near(f"k10c v{i_ + 1}", pv, K["k10c_traj"][i_]["v"], 1e-9); near(f"k10c p{i_ + 1}", pp, K["k10c_traj"][i_]["p"], 1e-9); near(f"k10c t{i_ + 1}", pt, K["k10c_traj"][i_]["t"], 1e-9)
    # the printed inputs themselves
    for nm_, got_, want_ in (("sin 39.74", round(math.sin(math.radians(39.74)), 4), 0.6393), ("sin 60", round(math.sin(math.radians(60)), 4), 0.8660), ("57.30", round(180 / math.pi, 2), 57.30)):
        near("input " + nm_, got_, want_, 1e-9)
    near("k23 speed", 8.0, K["k23_sols"][0]["speed"], 0.01); near("k23 from", 240.0, K["k23_sols"][0]["frm"], 0.1); near("k24 T", 17.5, K["k24_T_S"], 1e-9)

# ===== run in the browser =====
res = {"errors": [], "pages": {}, "numbers": {}}
text_by_page = {}
banned = re.compile(r"refut", re.I)
think_numbers = []
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    for page_path in page_paths:
        tag = page_path.name.split("_")[0].replace(".", "_")                # 2_2a ... 2_2d
        key = tag[-1].upper()
        r = res["pages"][page_path.name] = {}
        pg.goto(page_path.as_uri())
        pg.wait_for_selector(".blk")
        pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
        text = pg.inner_text("main")
        text_by_page[key] = text
        src = page_path.read_text(encoding="utf-8")
        tags = pg.locator(".prob.think .tag").all_inner_texts()
        nums = [m.group(1) + m.group(2).lower() for t in tags for m in [re.match(r"THINK (\d+)([A-Za-z]?)\b", t)] if m]       # "10", "10a" (a suffixed tag follows its base number)
        r["think"] = nums
        think_numbers += nums
        fe = pg.evaluate("window.figErrors")
        if fe:
            res["errors"].append(f"{page_path.name}: figure errors {fe}")
        # structure of every Think block
        bad = pg.evaluate("""() => { const out = []; for (const t of document.querySelectorAll('.prob.think')) { const tx = t.innerText; const id = (t.querySelector('.tag') || {}).innerText || '?';
            const why = (tx.match(/Why this step:/g) || []).length;
            const miss = [];
            if (!t.querySelector('figure svg')) miss.push('figure');
            if (!/Think first/.test(tx)) miss.push('Think first');
            if (![...t.querySelectorAll('summary')].some(s => s.innerText.trim() === 'Hint')) miss.push('Hint');
            if (!/Show the solution/.test(tx)) miss.push('Show the solution');
            if (why < 2) miss.push('Why this step x' + why);
            if (!/Check your answer\\./.test(tx)) miss.push('Check your answer');
            if (!/What if/.test(tx)) miss.push('What if');
            if (miss.length) out.push(id + ': ' + miss.join(', ')); } return out; }""")
        r["think_structure_problems"] = bad
        if bad:
            res["errors"].append(f"{page_path.name}: {bad}")
        # photograph every Think figure
        n_fig = pg.locator(".prob.think figure").count()
        for i in range(n_fig):
            pg.locator(".prob.think figure").nth(i).screenshot(path=f"out/think_{tag}_fig{i}.png")
        empty = pg.evaluate("[...document.querySelectorAll('.prob.think figure svg')].filter(s => s.children.length < 5).map(s => s.id)")
        if empty:
            res["errors"].append(f"{page_path.name}: empty figures {empty}")
        clipped = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const vb = s.viewBox.baseVal; const root = s.getBoundingClientRect();
            const sc = root.width / vb.width;
            for (const t of s.querySelectorAll('text')) { const r = t.getBoundingClientRect(); const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
              if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.push(s.id + ':' + t.textContent.slice(0, 30)); } } return out; }""")
        if clipped:
            res["errors"].append(f"{page_path.name}: labels outside the figure {clipped}")
        overlaps = pg.evaluate("""() => { const out = []; for (const s of document.querySelectorAll('.prob.think figure svg, #f1')) { const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
            for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
              const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
              if (w > 2 && h > 2 && w * h > 0.2 * Math.min(a.width * a.height, c.width * c.height)) out.push(s.id + ': "' + ts[i].t.textContent.slice(0, 20) + '" / "' + ts[j].t.textContent.slice(0, 20) + '"'); } } return out; }""")
        r["overlapping_labels"] = overlaps
        if overlaps:
            res["errors"].append(f"{page_path.name}: overlapping labels {overlaps}")
        if re.search(r"(?<![\d.])9\.81(?!\d)", text):
            res["errors"].append(f"{page_path.name}: the number 9.81 appears; use 9.8062 or 9.7965")
        if banned.search(text) or banned.search(src):
            res["errors"].append(f"{page_path.name}: banned word")
        # the quotations of page 2.2b
        if key == "B":
            quotes = pg.evaluate("[...document.querySelectorAll('pre.cx')].map(p => [p.getAttribute('data-src'), p.innerText])")
            r["quotes"] = len(quotes)
            ws = lambda s: " ".join(s.split())
            cache = {}
            for ds, body in quotes:
                repo, commit, path, lines = ds.split("|")
                u = f"https://raw.githubusercontent.com/{repo}/{commit}/{path}"
                if u not in cache:
                    req = urllib.request.Request(u, headers={"User-Agent": "Mozilla/5.0"})
                    cache[u] = urllib.request.urlopen(req, timeout=60).read().decode("utf-8").split("\n")
                a_, _, b_ = lines.partition("-")
                lo, hi = int(a_), int(b_ or a_)
                seg = ws(" ".join(cache[u][lo - 1:hi]))
                if seg != ws(body):
                    res["errors"].append(f"quotation {ds} differs from the file: file says {seg[:80]!r}, page says {ws(body)[:80]!r}")
        if key == "D":
            r["figures"] = pg.locator("figure").count()
            r["tables"] = pg.locator("main table").count()
            r["table_tools"] = pg.locator(".tblk .ttools").count()
            if r["tables"] != r["table_tools"]:
                res["errors"].append(f"{page_path.name}: {r['tables']} tables but {r['table_tools']} table tools")
            for i in range(r["figures"]):
                pg.locator("figure").nth(i).screenshot(path=f"out/{tag}_fig{i}.png")
            pg.screenshot(path=f"out/{tag}_full.png", full_page=True)
            labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", src)
            r["caption_labels"] = labs
            if labs != [str(i + 1) for i in range(len(labs))] or len(labs) != r["figures"]:
                res["errors"].append(f"{page_path.name}: figure labels {labs} are not 1..{r['figures']}")
            if re.search(r"Figures? \d+ and \d+", text):
                res["errors"].append(f"{page_path.name}: a 'Figures N and M' list must be checked by hand")
            bad_blocks = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const n = b.querySelectorAll('table').length; if (!n) continue;
                const how = (b.innerText.match(/How to read this table\\./g) || []).length, figs = b.querySelectorAll('figure').length;
                if (how < n || figs < n) out.push(b.dataset.b + ': tables ' + n + ', how-to ' + how + ', figures ' + figs); } return out; }""")
            if bad_blocks:
                res["errors"].append(f"{page_path.name}: {bad_blocks}")
            order_bad = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const c = b.querySelector('.c'); const kids = [...c.querySelectorAll('p, .tblk')];
                let seenHow = 0, seenTab = 0; for (const k of kids) { if (k.classList.contains('tblk')) { seenTab++; if (seenHow < seenTab) out.push(b.dataset.b + ' table ' + seenTab + ' has no how-to paragraph before it'); } else if (/How to read this table\\./.test(k.innerText)) seenHow++; } } return out; }""")
            if order_bad:
                res["errors"].append(f"{page_path.name}: {order_bad}")
            hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
            r["broken_links"] = [h for h in hrefs if not h.startswith(("http", "#")) and "autopilot_map" not in h and "references.html" not in h and "sim_physics" not in h and not (page_path.parent / h.split("#")[0]).exists()]
            if r["broken_links"]:
                res["errors"].append(f"{page_path.name}: broken links {r['broken_links']}")
            anchors = [h for h in hrefs if "#s2." in h and not h.startswith("http")]
            for h in anchors:                                              # the section anchors that the problems link to must exist on the named page
                fn, _, an = h.partition("#")
                tgt = (page_path.parent / fn)
                if not tgt.exists() or f'id="{an}"' not in tgt.read_text(encoding="utf-8"):
                    res["errors"].append(f"{page_path.name}: link {h} has no target")
            svg_html = re.findall(r"<svg.*?</svg>", src, flags=re.S)
            if any("<sub>" in s for s in svg_html):
                res["errors"].append(f"{page_path.name}: HTML <sub> inside an svg")
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
            pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")     # the tables lie inside the solutions, which are closed after a reload
            tb = pg.locator(".tblk").nth(1)
            tkey = page_id + ":" + tb.get_attribute("data-t")
            tb.hover(); tb.locator('.ttools button.dot[title="change this"]').click()
            tb.locator(".ttools >> text=+ Note").click(); pg.wait_for_timeout(200)
            pg.keyboard.type("table note on " + tag); pg.wait_for_timeout(600)
            pg.reload(); pg.wait_for_selector(".blk")
            pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
            tb = pg.locator(f'.tblk[data-t="{tkey.split(":", 1)[1]}"]')
            r["table_note_after_reload"] = tb.locator(".tnotes textarea").input_value()
            if r["table_note_after_reload"] != "table note on " + tag or "mk-red" not in (tb.get_attribute("class") or ""):
                res["errors"].append(f"{page_path.name}: table note/mark did not survive reload under {tkey}")
    b.close()

def think_key(s):
    m_ = re.fullmatch(r"(\d+)([a-z]?)", s)
    return (int(m_.group(1)), m_.group(2))


plain_ = [int(s) for s in think_numbers if s.isdigit()]
if plain_ != list(range(1, 26)) or think_numbers != sorted(think_numbers, key=think_key):
    res["errors"].append(f"Think numbers are {think_numbers}, expected the plain numbers 1..25 in page order")
for n_ in sorted({think_key(s)[0] for s in think_numbers}):            # a suffix run is a, b, c ... without a gap, right after its base number
    sufs_ = [think_key(s)[1] for s in think_numbers if think_key(s)[0] == n_]
    if sufs_ != [""] + list("abcdefgh")[:len(sufs_) - 1]:
        res["errors"].append(f"Think {n_}: tags {sufs_} are not '', 'a', 'b' ... in order")
res["think_tags"] = think_numbers

norm = lambda s: " ".join(s.split())
for page, name, s in E:
    t = text_by_page[page]
    found = s in t
    res["numbers"][page + " " + name] = {"printed": s, "found": found}
bad = [k for k, v in res["numbers"].items() if not v["found"]]
res["missing_numbers"] = bad
res["expected_not_found"] = {k: res["numbers"][k]["printed"] for k in bad}
res["tolerance_errors"] = tol_errors
res["errors"] += tol_errors

Path("out/check_2_2d.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"missing": bad, "errors": res["errors"], "expected_not_found": res["expected_not_found"],
                  "pages": {k: {kk: vv for kk, vv in v.items() if kk in ("think", "figures", "tables", "quotes", "think_structure_problems", "overlapping_labels")} for k, v in res["pages"].items()},
                  "n_numbers": len(E)}, ensure_ascii=False))
sys.exit(1 if bad or res["errors"] else 0)
