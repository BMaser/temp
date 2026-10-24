"""Check section 2.1: recompute every number independently and require it, formatted as printed,
in the page text. Also exercise the page's marks/notes in headless Chromium.
Section 2.1 is split over two pages (2.1a, 2.1b, owner's order 2026-10-20); pass both.
Run on the cloud machine (never on the owner's PC):
python check_2_1.py page/samples/2.1a_The_four_forces_one_at_a_time.html page/samples/2.1b_Forces_together_tilt_wind_simulators.html"""
import json
import math
import sys
from pathlib import Path
from playwright.sync_api import sync_playwright

page_paths = [Path(a).resolve() for a in sys.argv[1:]]
rho = 1.225
q = lambda v: 0.5 * rho * v ** 2
const_errors = []

# ---- gravity, version 4 of 2.1.2: WGS84 normal gravity (Somigliana on the ellipsoid + second-order height terms) ----
A, INV_F, GM, OMEGA = 6378137.0, 298.257223563, 3.986004418e14, 7.292115e-5     # WGS84 defining constants
B, E2, GE, GP = 6356752.3142, 0.00669437999014, 9.7803253359, 9.8321849378       # WGS84 derived constants (as printed)
K, M_SPIN = 0.00193185265241, 0.00344978650684
F = 1 / INV_F
for name, printed, derived in (("b = a(1-f)", B, A * (1 - F)), ("e2 = f(2-f)", E2, F * (2 - F)),
                               ("k = b gp/(a ge) - 1", K, B * GP / (A * GE) - 1), ("m = w2 a2 b/GM", M_SPIN, OMEGA ** 2 * A ** 2 * B / GM)):
    if abs(printed / derived - 1) > 1e-8:                     # the printed constants must agree with their definitions
        const_errors.append(f"WGS84 constant {name}: printed {printed}, derived {derived}")
def g_phi(phi):
    s = math.sin(phi) ** 2
    return GE * (1 + K * s) / math.sqrt(1 - E2 * s)
def h_factor(phi, h):
    s = math.sin(phi) ** 2
    return 1 - (2 / A) * (1 + F + M_SPIN - 2 * F * s) * h + (3 / A ** 2) * h ** 2
def g_full(phi, h):
    return g_phi(phi) * h_factor(phi, h)
# home field of the exercises: 45 deg N, h = 0 m; the page carries g rounded to 4 decimals
g = round(g_full(math.radians(45), 0), 4)                     # 9.8062
gD = round(g_full(math.radians(39.74), 1600), 4)              # 9.7965, the high field of the exercises: Denver, 39.74 deg N, h = 1600 m

expect = {}
# worked example 1: m 2.0 kg, S 0.50 m2, V 15 m/s
W1 = 2.0 * g; q1 = q(15); k1 = q1 * 0.50; CL1 = W1 / k1
expect["WE1 weight"] = (f"{W1:.2f} N", W1)
expect["WE1 q"] = (f"{q1:.1f} Pa", q1)
expect["WE1 q*S"] = (f"{k1:.1f}", k1)
expect["WE1 C_L"] = (f"{CL1:.3f}", CL1)
# worked example 2: m 0.80 kg, V 10 m/s, C_D 1.0, S 0.02 m2
W2 = 0.80 * g; D2 = q(10) * 0.02 * 1.0; T2 = math.hypot(W2, D2); th2 = math.degrees(math.atan2(D2, W2))
expect["WE2 hover thrust"] = (f"{W2:.3f} N", W2)
expect["WE2 per rotor"] = (f"{W2/4:.3f} N", W2 / 4)
expect["WE2 drag"] = (f"{D2:.3f} N", D2)
expect["WE2 T^2"] = (f"√{T2**2:.2f}", T2 ** 2)
expect["WE2 thrust"] = (f"{T2:.3f} N", T2)
expect["WE2 ratio"] = (f"arctan({D2/W2:.4f})", D2 / W2)
expect["WE2 tilt"] = (f"{th2:.2f}°", th2)
expect["WE2 extra thrust %"] = (f"{(T2/W2-1)*100:.1f} %", (T2 / W2 - 1) * 100)
d = math.radians
# 2.1.1 force: 100 g apple
expect["apple weight"] = (f"{0.1*g:.2f} N", 0.1 * g)
# 2.1.2 weight: 1.2 kg + 0.3 kg camera; Mars g = 3.71
expect["T2 before"] = (f"{1.2*g:.3f} N", 1.2 * g)
expect["T2 after"] = (f"{1.5*g:.3f} N", 1.5 * g)
expect["T3 Mars"] = (f"{1.2*3.71:.3f} N", 1.2 * 3.71)
# Try it 2a (2.1.2, what the accelerometer reads): reading = r_ddot_z - g_z (Barfoot 7.150), z up so g_z = -g; thrust = m x reading
MN = "−"
expect["T2a table"] = (f"0 − ({MN}{g:.4f}) = {g:.4f} m/s²", g)
expect["T2a climb"] = (f"3.0 − ({MN}{g:.4f}) = {3.0 + g:.4f} m/s²", 3.0 + g)
expect["T2a fall"] = (f"{MN}{g:.4f} − ({MN}{g:.4f}) = {(-g) - (-g):.0f} m/s²", 0.0)
expect["T2a thrust"] = (f"1.5 × {3.0 + g:.4f} = {1.5 * (3.0 + g):.3f} N", 1.5 * (3.0 + g))
expect["T2a check"] = (f"{1.5 * g:.3f} + 1.5 × 3.0 = {1.5 * g:.3f} + 4.5 = {round(1.5 * g, 3) + 4.5:.3f} N", 1.5 * g + 4.5)
# 2.1.3 thrust: 1.5 kg hover, 4 x 10 N motors
expect["T4 per rotor"] = (f"{1.5*g/4:.3f} N", 1.5 * g / 4)
expect["T5 T/W"] = (f"{40/(1.5*g):.2f}", 40 / (1.5 * g))
expect["T5 accel"] = (f"{(40-1.5*g)/1.5:.2f} m/s²", (40 - 1.5 * g) / 1.5)
# 2.1.4 air
expect["T6 q10"] = (f"{q(10):.2f} Pa", q(10))
expect["T7 speed"] = (f"{90/3.6:.0f} m/s", 90 / 3.6)
expect["T7 q25"] = (f"{q(25):.1f} Pa", q(25))
# 2.1.5 drag
expect["T8 drag"] = (f"{q(15)*0.02*1.0:.2f} N", q(15) * 0.02)
expect["T9 slower"] = (f"{15/math.sqrt(2):.1f} m/s", 15 / math.sqrt(2))
# 2.1.6 C_L from a wind-tunnel test, then at 10 m/s
CLt = 30 / (q(20) * 0.25)
expect["T10 C_L"] = (f"{CLt:.2f}", CLt)
expect["T10 lift at 10 m/s"] = (f"{q(10)*0.25*round(CLt,2):.1f} N", q(10) * 0.25 * round(CLt, 2))
# 2.1.7 net force: 3 kg plane
expect["T12 lift"] = (f"{3*g:.2f} N", 3 * g)
# Try it 19a: climbing plane by parts (page rounds cos/sin to 4 decimals and each part to 3)
c10, s10 = round(math.cos(d(10)), 4), round(math.sin(d(10)), 4)
expect["T19a cos"] = (f"{c10:.4f}", c10)
expect["T19a sin"] = (f"{s10:.4f}", s10)
px = [round(20 * c10, 3), round(-8 * c10, 3), round(-28 * s10, 3)]
pz = [round(20 * s10, 3), round(-8 * s10, 3), round(28 * c10, 3), round(-3 * g, 3)]
for i, v in enumerate(px):
    expect[f"T19a x{i}"] = (f"{v:.3f} N".replace("-", "−"), v)
for i, v in enumerate(pz):
    expect[f"T19a z{i}"] = (f"{v:.3f} N".replace("-", "−"), v)
Fx, Fz = sum(px), sum(pz); Fn = math.hypot(Fx, Fz)
expect["T19a Fx"] = (f"{Fx:.3f} N", Fx)
expect["T19a Fz"] = (f"{Fz:.3f} N", Fz)
expect["T19a Fnet"] = (f"{Fn:.3f} N", Fn)
expect["T19a a"] = (f"{Fn / 3:.2f} m/s²", Fn / 3)
expect["T19a angle"] = (f"{math.degrees(math.atan2(Fz, Fx)):.2f}°", math.degrees(math.atan2(Fz, Fx)))
# 2.1.8 tilt
expect["T13 up share"] = (f"{100*math.cos(d(30)):.1f} %", 100 * math.cos(d(30)))
expect["T13 fwd share"] = (f"{100*math.sin(d(30)):.0f} %", 100 * math.sin(d(30)))
expect["T14 up"] = (f"{10*math.cos(d(20)):.2f} N", 10 * math.cos(d(20)))
expect["T14 fwd"] = (f"{10*math.sin(d(20)):.2f} N", 10 * math.sin(d(20)))
expect["T15 decel"] = (f"{q(10)*0.02/0.80:.2f} m/s²", q(10) * 0.02 / 0.80)
# section problems 3 and 4
expect["S3 weight"] = (f"{2*g:.2f} N", 2 * g)
expect["S3 cos"] = (f"{2*g/50:.4f}", 2 * g / 50)
expect["S3 angle"] = (f"{math.degrees(math.acos(2*g/50)):.1f}°", math.degrees(math.acos(2 * g / 50)))
expect["S4 before"] = (f"{1*g:.2f} N", g)
# practice 2 (now Try it 11)
L2 = q(20) * 0.30 * 0.8
expect["P2 q"] = (f"{q(20):.0f} Pa", q(20))
expect["P2 lift"] = (f"{L2:.1f} N", L2)
expect["P2 weight 3 kg"] = (f"{3*g:.2f} N", 3 * g)
expect["P2 max mass"] = (f"{L2/g:.2f} kg", L2 / g)
# practice 3: 72 km/h
v3 = 72 / 3.6; D3 = q(v3) * 0.015 * 0.9; D3wrong = q(72) * 0.015 * 0.9
expect["P3 speed"] = (f"{v3:.0f} m/s", v3)
expect["P3 drag"] = (f"{D3:.2f} N", D3)
expect["P3 wrong drag"] = (f"{D3wrong:.1f} N", D3wrong)
expect["P3 factor ~13"] = ("13 times", D3wrong / D3)
# practice 4
expect["P4 fraction"] = (f"{(10/20)**2:.2f}", (10 / 20) ** 2)

# ---- 2026-10-20 additions: height, own C_D, C_T, wind, payload problems ----
rpm = lambda n: f"{round(n * 60, -2):,.0f}".replace(",", " ") + " rpm"
def nasa(h=None, Tc=None):                       # NASA Glenn Earth Atmosphere Model, troposphere, metric
    T = 15.04 - 0.00649 * h if Tc is None else Tc
    p = 101.29 * ((T + 273.1) / 288.08) ** 5.256
    return T, p, p / (0.2869 * (T + 273.1))
Td, pd, rd = nasa(1600)
rD = round(rd, 3)                                # the page carries 1.049 forward
expect["H T"] = (f"{Td:.3f} °C", Td)
expect["H T+273.1"] = (f"{Td + 273.1:.3f}", Td + 273.1)
expect["H p"] = (f"{pd:.2f} kPa", pd)
expect["H rho"] = (f"{rd:.3f} kg/m³", rd)
expect["H rho0"] = (f"{nasa(0)[2]:.3f} kg/m³", nasa(0)[2])
expect["H thinner %"] = (f"{(1 - rD / rho) * 100:.1f} %", (1 - rD / rho) * 100)
pD = math.radians(39.74)
expect["H g change"] = (f"{(1 - h_factor(pD, 1600)) * 100:.2f} %", (1 - h_factor(pD, 1600)) * 100)   # height alone, Denver
expect["H g Denver"] = (f"{gD:.4f} m/s²", gD)
expect["T9 W Denver"] = (f"{1.5 * gD:.3f} N", 1.5 * gD)
expect["T9 W home"] = (f"{1.5 * g:.3f} N", 1.5 * g)
expect["T9 less %"] = (f"{(1 - gD / g) * 100:.2f} %", (1 - gD / g) * 100)
expect["T9 q Denver"] = (f"{0.5 * rD * 400:.1f} Pa", 0.5 * rD * 400)
rhot = 101.3 / (0.2869 * (35 + 273.1))
expect["T10 hot rho"] = (f"{rhot:.3f} kg/m³", rhot)
expect["T10 hot %"] = (f"{(1 - rhot / rho) * 100:.1f} %", (1 - rhot / rho) * 100)
# 2.1.8 C_T
d4 = 0.127 ** 4; Ts = 0.5 * g; rn = rho * 400 ** 2 * d4; CTx = Ts / rn; CT = round(CTx, 4)
expect["T15 T"] = (f"{Ts:.3f} N", Ts)
expect["T15 d4"] = (f"{d4:.8f} m⁴", d4)
expect["T15 rho n2 d4"] = (f"{rn:.2f} N", rn)
expect["T15 C_T"] = (f"{CTx:.4f}", CTx)
den = 4 * CT * rho * d4; n16 = math.sqrt(0.8 * g / den)
expect["T16 denom"] = (f"{den:.8f}", den)
expect["T16 n"] = (f"{n16:.1f} rev/s", n16)
expect["T16 rpm"] = (rpm(n16), n16 * 60)
expect["T17 load"] = (f"{1.0 * g / 5:.3f} N", g / 5)
n17 = math.sqrt(1.0 * g / (5 * CT * rho * d4))
expect["T17 n same"] = (f"{n17:.1f} rev/s", n17)
n18 = n16 * math.sqrt((gD / g) * (rho / rD))          # Denver: lighter (g) and thinner air (rho)
expect["T18 g ratio"] = (f"{gD / g:.4f}", gD / g)
expect["T18 n"] = (f"{n18:.1f} rev/s", n18)
expect["T18 rpm"] = (rpm(n18), n18 * 60)
expect["T18 %"] = (f"{(n18 / n16 - 1) * 100:.1f} %", (n18 / n16 - 1) * 100)
# 2.1.11 own C_D
t12 = math.tan(d(12)); D23 = 0.8 * g * t12; CD23 = D23 / (q(12) * 0.02)
expect["T23 tan"] = (f"{t12:.4f}", t12)
expect["T23 D"] = (f"{D23:.3f} N", D23)
expect["T23 q"] = (f"{q(12):.1f} Pa", q(12))
expect["T23 C_D"] = (f"{CD23:.3f}", CD23)
CD24 = 0.8 * 2.0 / (q(12) * 0.02)
expect["T24 C_D"] = (f"{CD24:.3f}", CD24)
expect["T24 mean"] = (f"{(round(CD23, 3) + round(CD24, 3)) / 2:.2f}", (CD23 + CD24) / 2)
# Try it 22a: wind tunnel on a warm day (the page carries rho = 1.175 and q = 132.2 forward)
r22 = 100.5 / (0.2869 * (25 + 273.1)); r22r = round(r22, 3); q22 = 0.5 * r22r * 15 ** 2
expect["T22a rho"] = (f"{r22:.3f} kg/m³", r22)
expect["T22a q"] = (f"{q22:.1f} Pa", q22)
expect["T22a CD"] = (f"{2.10 / (round(q22, 1) * 0.02):.3f}", 2.10 / (round(q22, 1) * 0.02))
expect["T22a q sea"] = (f"{q(15):.1f} Pa", q(15))
expect["T22a CD sea"] = (f"{2.10 / (round(q(15), 1) * 0.02):.3f}", 2.10 / (round(q(15), 1) * 0.02))
# Try it 26a: CFD
cd26 = 1.52 / (round(q(12), 1) * 0.02)
expect["T26a CD"] = (f"{cd26:.3f}", cd26)
expect["T26a %"] = (f"{(1 - round(cd26, 3) / 0.945) * 100:.1f} %", (1 - round(cd26, 3) / 0.945) * 100)
expect["T25 W"] = (f"{0.3 * g:.3f} N", 0.3 * g)
expect["T25 q"] = (f"{q(9.5):.2f} Pa", q(9.5))
expect["T25 C_D"] = (f"{0.3 * g / (q(9.5) * 0.04):.2f}", 0.3 * g / (q(9.5) * 0.04))
parts = [1.28 * 0.010, 1.28 * 0.006, 0.5 * 0.002]
for i, v in enumerate(parts):
    expect[f"T26 part {i}"] = (f"{v:.5f}", v)
dA = sum(parts)
expect["T26 drag area"] = (f"{dA:.5f} m²", dA)
expect["T26 C_D"] = (f"{dA / 0.018:.2f}", dA / 0.018)
expect["T26 D"] = (f"{q(15) * dA:.2f} N", q(15) * dA)
# 2.1.12 wind
expect["T28 head"] = (f"{q(20) * 0.02:.2f} N", q(20) * 0.02)
V29 = math.hypot(10, 5); D29 = q(V29) * 0.02
expect["T29 V"] = (f"{V29:.2f} m/s", V29)
expect["T29 D"] = (f"{D29:.3f} N", D29)
expect["T29 west"] = (f"{D29 * 10 / V29:.3f} N", D29 * 10 / V29)
expect["T29 south"] = (f"{D29 * 5 / V29:.3f} N", D29 * 5 / V29)
expect["T29 lean"] = (f"{math.degrees(math.atan2(5, 10)):.1f}°", math.degrees(math.atan2(5, 10)))
expect["T29 tilt"] = (f"{math.degrees(math.atan2(D29, 0.8 * g)):.2f}°", math.degrees(math.atan2(D29, 0.8 * g)))
expect["T29 T"] = (f"{math.hypot(0.8 * g, D29):.3f} N", math.hypot(0.8 * g, D29))
# problem 5: 1.5 kg drone + 2 kg parcel, 4 x 15 N
W5 = 3.5 * g; W0 = 1.5 * g
expect["P5 W"] = (f"{W5:.3f} N", W5)
expect["P5 per rotor"] = (f"{W5 / 4:.3f} N", W5 / 4)
expect["P5 share"] = (f"{W5 / 4 / 15 * 100:.1f} %", W5 / 60 * 100)
expect["P5 share empty"] = (f"{W0 / 4 / 15 * 100:.1f} %", W0 / 60 * 100)
expect["P5 T/W"] = (f"{60 / W5:.2f}", 60 / W5)
expect["P5 T/W empty"] = (f"{60 / W0:.2f}", 60 / W0)
expect["P5 accel"] = (f"{(60 - W5) / 3.5:.2f} m/s²", (60 - W5) / 3.5)
expect["P5 cos"] = (f"{W5 / 60:.3f}", W5 / 60)
expect["P5 cos empty"] = (f"{W0 / 60:.3f}", W0 / 60)
a5 = math.degrees(math.acos(W5 / 60)); a0 = math.degrees(math.acos(W0 / 60))
expect["P5 max tilt"] = (f"{a5:.1f}°", a5)
expect["P5 max tilt empty"] = (f"{a0:.1f}°", a0)
expect["P5 tilt lost"] = (f"about {round(a0 - a5)}°", a0 - a5)
CdS = 1.0 * 0.03 + 1.28 * 0.02; D5 = q(12) * CdS
expect["P5 drag area"] = (f"{CdS:.4f} m²", CdS)
expect["P5 D"] = (f"{D5:.2f} N", D5)
expect["P5 tilt"] = (f"{math.degrees(math.atan2(D5, W5)):.2f}°", math.degrees(math.atan2(D5, W5)))
T5 = math.hypot(W5, D5)
expect["P5 T"] = (f"{T5:.2f} N", T5)
expect["P5 T per rotor"] = (f"{T5 / 4:.2f} N", T5 / 4)
expect["P5 T share"] = (f"{T5 / 60 * 100:.1f} %", T5 / 60 * 100)
expect["P5 Denver D"] = (f"{D5 * rD / rho:.2f} N", D5 * rD / rho)
expect["P5 Denver max T"] = (f"{60 * rD / rho:.2f} N", 60 * rD / rho)
W5d = 3.5 * gD
expect["P5 Denver W"] = (f"{W5d:.3f} N", W5d)
expect["P5 Denver T/W"] = (f"{60 * rD / rho / W5d:.2f}", 60 * rD / rho / W5d)
expect["P5 Denver n %"] = (f"{(math.sqrt((gD / g) * (rho / rD)) - 1) * 100:.1f} %", (math.sqrt((gD / g) * (rho / rD)) - 1) * 100)
# problem 6: crosswind 8 m/s from the west
D6 = q(8) * CdS; T6 = math.hypot(W5, D6)
expect["P6 D"] = (f"{D6:.2f} N", D6)
expect["P6 tilt"] = (f"{math.degrees(math.atan2(D6, W5)):.2f}°", math.degrees(math.atan2(D6, W5)))
expect["P6 T"] = (f"{T6:.2f} N", T6)
expect["P6 extra"] = (f"{(T6 / W5 - 1) * 100:.1f} %", (T6 / W5 - 1) * 100)
# problem 7: pentarotor 4.5 kg, d = 0.254 m, C_T 0.10
W7 = 4.5 * g; d47 = 0.254 ** 4; n7 = math.sqrt(W7 / 5 / (0.10 * rho * d47)); n7d = math.sqrt(4.5 * gD / 5 / (0.10 * rD * d47))
expect["P7 W Denver"] = (f"{4.5 * gD:.3f} N", 4.5 * gD)
expect["P7 W"] = (f"{W7:.3f} N", W7)
expect["P7 per rotor"] = (f"{W7 / 5:.3f} N", W7 / 5)
expect["P7 d4"] = (f"{d47:.6f} m⁴", d47)
expect["P7 n"] = (f"{n7:.1f} rev/s", n7)
expect["P7 rpm"] = (rpm(n7), n7 * 60)
expect["P7 n Denver"] = (f"{n7d:.1f} rev/s", n7d)
expect["P7 rpm Denver"] = (rpm(n7d), n7d * 60)

# ---- 2026-10-20 round 4: figure 2, wind at an angle, simulator section ----
expect["F2 Earth W"] = (f"{1.2 * g:.1f} N", 1.2 * g)
expect["F2 Mars W"] = (f"{1.2 * 3.71:.1f} N", 1.2 * 3.71)
W08 = 0.8 * g
for beta, tag in ((30, "30"), (45, "45")):
    b = d(beta); V2 = 10 ** 2 + 5 ** 2 + 2 * 10 * 5 * math.cos(b); Vb = math.sqrt(V2); Db = 0.5 * rho * V2 * 0.02
    Vx, Vy = 10 + 5 * math.cos(b), 5 * math.sin(b)
    expect[f"T30 V2 {tag}"] = (f"{V2:.2f}", V2)
    expect[f"T30 V {tag}"] = (f"{Vb:.2f} m/s", Vb)
    expect[f"T30 D {tag}"] = (f"{Db:.3f} N", Db)
    expect[f"T30 Vx {tag}"] = (f"{Vx:.3f}", Vx)
    expect[f"T30 Vy {tag}"] = (f"{Vy:.3f}", Vy)
    expect[f"T30 lean {tag}"] = (f"{math.degrees(math.atan2(Vy, Vx)):.2f}°", math.degrees(math.atan2(Vy, Vx)))
    expect[f"T30 tilt {tag}"] = (f"{math.degrees(math.atan2(Db, W08)):.2f}°", math.degrees(math.atan2(Db, W08)))
expect["T30 head V"] = (f"{15:.2f} m/s", 15)
expect["T30 head D"] = (f"{q(15) * 0.02:.3f} N", q(15) * 0.02)
front, top = 1.0 * 0.02, 1.33 * 0.04
expect["T31 top area"] = (f"{top:.4f} m²", top)
Vh, Vv = 10 * math.cos(d(30)), 10 * math.sin(d(30))
Dh, Dv = 0.5 * rho * Vh ** 2 * front, 0.5 * rho * Vv ** 2 * top
expect["T31a Vh"] = (f"{Vh:.3f} m/s", Vh)
expect["T31a Vv"] = (f"{Vv:.3f} m/s", Vv)
expect["T31a Dh"] = (f"{Dh:.3f} N", Dh)
expect["T31a Dv"] = (f"{Dv:.3f} N", Dv)
expect["T31a up"] = (f"{W08 - Dv:.3f} N", W08 - Dv)
expect["T31a tilt"] = (f"{math.degrees(math.atan2(Dh, W08 - Dv)):.2f}°", math.degrees(math.atan2(Dh, W08 - Dv)))
Ta = math.hypot(Dh, W08 - Dv)
expect["T31a T"] = (f"{Ta:.3f} N", Ta)
expect["T31a less"] = (f"{(1 - Ta / W08) * 100:.1f} %", (1 - Ta / W08) * 100)
Vh = 10 * math.cos(d(45)); Dh, Dv = 0.5 * rho * Vh ** 2 * front, 0.5 * rho * Vh ** 2 * top
expect["T31b Vh"] = (f"{Vh:.3f} m/s", Vh)
expect["T31b Dh"] = (f"{Dh:.4f} N", Dh)
expect["T31b Dv"] = (f"{Dv:.3f} N", Dv)
expect["T31b up"] = (f"{W08 + Dv:.3f} N", W08 + Dv)
expect["T31b tilt"] = (f"{math.degrees(math.atan2(Dh, W08 + Dv)):.2f}°", math.degrees(math.atan2(Dh, W08 + Dv)))
Tb = math.hypot(Dh, W08 + Dv)
expect["T31b T"] = (f"{Tb:.3f} N", Tb)
expect["T31b more"] = (f"{(Tb / W08 - 1) * 100:.1f} %", (Tb / W08 - 1) * 100)
# Try 32: Gazebo x500 (x500/model.sdf, x500_base/model.sdf)
kf, km, wmax, mx = 8.54858e-06, 0.016, 1000.0, 2.0
expect["T32 Tmax"] = (f"{kf * wmax ** 2:.3f} N", kf * wmax ** 2)
expect["T32 4Tmax"] = (f"{4 * kf * wmax ** 2:.2f}", 4 * kf * wmax ** 2)
expect["T32 TWR"] = (f"{4 * kf * wmax ** 2 / (mx * g):.2f}", 4 * kf * wmax ** 2 / (mx * g))
expect["T32 4kf"] = (f"{4 * kf * 1e5:.4f}", 4 * kf)
wh = math.sqrt(mx * g / (4 * kf))
expect["T32 omega"] = (f"{wh:.1f} rad/s", wh)
expect["T32 rpm"] = (rpm(wh / (2 * math.pi)), wh * 60 / (2 * math.pi))
expect["T32 Q"] = (f"{km * mx * g / 4:.4f} N·m", km * mx * g / 4)
# Try 33: Project AirSim rotor (docs/config_robot.md:613-617)
CTa, CPa, Da, rpma = 0.109919, 0.040164, 0.2286, 6396.667
na = rpma / 60; Tma = CTa * rho * na ** 2 * Da ** 4; Qma = CPa * rho * na ** 2 * Da ** 5 / (2 * math.pi)
expect["T33 n"] = (f"{na:.2f} rev/s", na)
expect["T33 Tmax"] = (f"{Tma:.3f} N", Tma)
expect["T33 Qmax"] = (f"{Qma:.5f} N·m", Qma)
expect["T33 Denver"] = (f"{Tma * rD / rho:.3f} N", Tma * rD / rho)
# Try 34: PX4 EKF2 drag model (drag_fusion.cpp:139-140)
BC, mu = 40.0, 0.15
a1, a2 = 0.5 * rho * 100 / BC, mu * 10
expect["T34 CdS"] = (f"{0.8 / BC:.3f} m²", 0.8 / BC)
expect["T34 a1"] = (f"{a1:.3f} m/s²", a1)
expect["T34 a2"] = (f"{a2:.2f} m/s²", a2)
expect["T34 sum"] = (f"{a1 + a2:.3f}", a1 + a2)
expect["T34 D"] = (f"{0.8 * (a1 + a2):.3f} N", 0.8 * (a1 + a2))
# Try 35: PX4 hover thrust estimator (zero_order_hover_thrust_ekf.cpp:104)
uh = g * 0.60 / (1.5 + g); u3 = uh * (3.0 + g) / g
expect["T35 a+g"] = (f"{1.5 + g:.2f}", 1.5 + g)
expect["T35 uh"] = (f"{uh:.3f}", uh)
expect["T35 uh4"] = (f"{uh:.4f}", uh)
expect["T35 u"] = (f"{u3:.3f}", u3)
# Try 36: THR_MDL_FAC (motor_params.yaml:15)
f_ = 0.3; a_, b_ = f_, 1 - f_
expect["T36 Trel"] = (f"{f_ * 0.25 + (1 - f_) * 0.5:.3f}", f_ * 0.25 + (1 - f_) * 0.5)
expect["T36 b/2a"] = (f"{b_ / (2 * a_):.4f}", b_ / (2 * a_))
expect["T36 b2/4a2"] = (f"{b_ ** 2 / (4 * a_ ** 2):.4f}", b_ ** 2 / (4 * a_ ** 2))
expect["T36 T/a"] = (f"{0.5 / a_:.4f}", 0.5 / a_)
u36 = -b_ / (2 * a_) + math.sqrt(b_ ** 2 / (4 * a_ ** 2) + 0.5 / a_)
expect["T36 u"] = (f"{u36:.3f}", u36)
# Try 37: PX4 SIH linear drag (sih_params.yaml:151-154)
expect["T37 linear"] = (f"{1.0 * g / 1.0:.2f} m/s", g)
expect["T37 rhoCdS"] = (f"{rho * 0.02:.4f}", rho * 0.02)
Vq = math.sqrt(2 * 1.0 * g / (rho * 0.02))
expect["T37 quad"] = (f"{Vq:.2f} m/s", Vq)

# ---- round 5: the most complete formulas from the code ----
som = g_phi                                                   # 2026-10-20: WGS84 k (PX4 stores 0.001931851353, same to 8 decimals)
expect["T3a e2"] = (f"{0.0818191908425 ** 2:.8f}", 0.0818191908425 ** 2)
expect["T3a k/2"] = (f"{K / 2:.9f}", K / 2)
expect["T3a e2/2"] = (f"{E2 / 2:.8f}", E2 / 2)
expect["T3a 1+k"] = (f"{1 + K:.9f}", 1 + K)
for deg in (0, 45, 90):
    expect[f"T3a g{deg}"] = (f"{som(d(deg)):.3f} m/s²", som(d(deg)))
expect["T3a dg"] = (f"{som(d(90)) - som(d(0)):.4f}", som(d(90)) - som(d(0)))
expect["T3a dW"] = (f"{1.5 * (som(d(90)) - som(d(0))):.3f} N", 1.5 * (som(d(90)) - som(d(0))))
expect["T3a %"] = (f"{(GP / GE - 1) * 100:.2f} %", (GP / GE - 1) * 100)
# gravity block of 2.1.2: why g changes, home field
expect["G spin push"] = (f"{OMEGA ** 2 * A:.4f} m/s²", OMEGA ** 2 * A)
expect["G equator speed"] = (f"{OMEGA * A:.0f} m/s", OMEGA * A)
expect["G a-b"] = (f"{(A - B) / 1000:.1f} km", (A - B) / 1000)
grad = g_phi(d(45)) * (2 / A) * (1 + F + M_SPIN - F)          # free-air gradient at 45 deg, per metre
expect["G gradient km"] = (f"{grad * 1000:.4f} m/s²", grad * 1000)
expect["G home"] = (f"{g:.4f} m/s²", g)
expect["G f"] = (f"{F:.11f}", F)
# Try 3b: full formula in Denver (39.74 deg N, h = 1600 m)
s3 = math.sin(pD) ** 2; c1 = (2 / A) * (1 + F + M_SPIN - 2 * F * s3); c2 = 3 * 1600 ** 2 / A ** 2; hf = 1 - c1 * 1600 + c2
expect["T3b sin2"] = (f"{s3:.5f}", s3)
expect["T3b 1+ks"] = (f"{1 + K * s3:.6f}", 1 + K * s3)
expect["T3b 1-e2s"] = (f"{1 - E2 * s3:.6f}", 1 - E2 * s3)
expect["T3b g_phi"] = (f"{g_phi(pD):.4f} m/s²", g_phi(pD))
expect["T3b bracket"] = (f"{1 + F + M_SPIN - 2 * F * s3:.6f}", 1 + F + M_SPIN - 2 * F * s3)
expect["T3b c1"] = (f"{c1 * 1e7:.4f} × 10⁻⁷", c1)
expect["T3b c1 h"] = (f"{c1 * 1600:.6f}", c1 * 1600)
expect["T3b c2"] = (f"{c2:.7f}", c2)
expect["T3b factor"] = (f"{hf:.6f}", hf)
expect["T3b g"] = (f"{g_full(pD, 1600):.4f} m/s²", g_full(pD, 1600))
# Try 3c: the four versions in Denver, against version 4
g4 = g_full(pD, 1600); R = 6371000; g2 = 9.80665 * (R / (R + 1600)) ** 2
for tag, val in (("v1 9.81", 9.81), ("v1 9.80665", 9.80665), ("v2", g2), ("v3", g_phi(pD))):
    expect[f"T3c {tag} %"] = (f"{abs(val / g4 - 1) * 100:.3f} %", abs(val / g4 - 1) * 100)
expect["T3c v2"] = (f"{g2:.4f} m/s²", g2)
# Try 3d: GPS heights, h = H + N
p3d = d(43.5); g3d = g_full(p3d, 330 - 36); g3dH = g_full(p3d, 330)
expect["T3d g"] = (f"{g3d:.4f} m/s²", g3d)
expect["T3d dg"] = (f"{g3d - g3dH:.6f} m/s²", g3d - g3dH)
expect["T3d %"] = (f"{(g3d / g3dH - 1) * 100:.4f} %", (g3d / g3dH - 1) * 100)
# Try 3e: an estimator that assumes 9.81 at the equator
ge0 = round(g_full(0, 0), 4); aerr = round(9.81 - ge0, 4)
expect["T3e g equator"] = (f"{ge0:.4f} m/s²", ge0)
expect["T3e a"] = (f"{aerr:.4f} m/s²", aerr)
expect["T3e v"] = (f"{aerr * 10:.3f} m/s", aerr * 10)
expect["T3e z"] = (f"{0.5 * aerr * 10 ** 2:.3f} m", 0.5 * aerr * 100)
Km = 0.15
for V in (10, 20):
    Db, Dr = 0.5 * rho * 0.02 * V ** 2, 0.8 * Km * V
    expect[f"T12a Dr{V}"] = (f"{Dr:.2f} N", Dr)
    expect[f"T12a total{V}"] = (f"{Db + Dr:.3f} N" if V == 10 else f"{Db + Dr:.2f} N", Db + Dr)
    expect[f"T12a share{V}"] = (f"{Dr / (Db + Dr) * 100:.1f} %", Dr / (Db + Dr) * 100)
Dbd, Drd = 0.5 * rD * 0.02 * 400, 0.8 * Km * 20 * math.sqrt(rD / rho)
expect["T12a Db Denver"] = (f"{Dbd:.3f} N", Dbd)
expect["T12a Dr Denver"] = (f"{Drd:.3f} N", Drd)
expect["T12a total Denver"] = (f"{Dbd + Drd:.3f} N", Dbd + Drd)
al = 2 * math.pi / 180; CLp = 0.15188 + 5.015 * al; CDp = 0.029 + CLp ** 2 / (math.pi * 6.5 * 0.97)
expect["T13a alpha"] = (f"{al:.4f} rad", al)
expect["T13a CL"] = (f"{CLp:.3f}", CLp)
expect["T13a CD"] = (f"{CDp:.4f}", CDp)
expect["T13a L/D"] = (f"{CLp / CDp:.1f}", CLp / CDp)
J = 15 / (252.9 * 0.127); CTJ = 0.0962 - 0.06 * J - 0.05 * J ** 2; TJ = CTJ * rho * 252.9 ** 2 * d4      # the page uses the rounded hover speed 252.9 rev/s
expect["T18a J"] = (f"{J:.3f}", J)
expect["T18a CT"] = (f"{CTJ:.4f}", CTJ)
expect["T18a T"] = (f"{TJ:.3f} N", TJ)
alag = math.exp(-0.004 / 0.0125); w1 = alag * 500 + (1 - alag) * 800
expect["T18b alpha"] = (f"{alag:.4f}", alag)
expect["T18b 1-alpha"] = (f"{1 - alag:.4f}", 1 - alag)
expect["T18b omega"] = (f"{w1:.1f} rad/s", w1)
ias = math.sqrt(2 * 150 / rho)
expect["T29a IAS"] = (f"{ias:.2f} m/s", ias)
expect["T29a TAS"] = (f"{ias * math.sqrt(rho / rD):.2f} m/s", ias * math.sqrt(rho / rD))

res = {"errors": list(const_errors), "numbers": {}, "pages": {}}
text = ""                                          # 2.1 is split over two pages (2.1a, 2.1b); numbers may sit on either
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    for page_path in page_paths:
        tag = page_path.name.split("_")[0].replace(".", "_")        # 2_1a, 2_1b
        r = res["pages"][page_path.name] = {}
        pg.goto(page_path.as_uri())
        pg.wait_for_selector(".blk")
        # open all solutions so their text is visible
        pg.evaluate("document.querySelectorAll('details').forEach(d => d.open = true)")
        text += pg.inner_text("main") + "\n"
        r["blocks"] = pg.locator(".blk").count()
        r["figures"] = pg.locator("figure").count()
        r["try_its"] = pg.locator(".prob .tag").filter(has_text="TRY IT").count()
        # every link to a local file must point to a file that exists
        hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
        r["broken_links"] = [h for h in hrefs if not h.startswith(("http", "#")) and not (page_path.parent / h.split("#")[0]).exists()]
        if r["broken_links"]:
            res["errors"].append(f"{page_path.name}: broken links {r['broken_links']}")
        pg.screenshot(path=f"out/{tag}_full.png", full_page=True)
        for i in range(pg.locator("figure").count()):
            pg.locator("figure").nth(i).screenshot(path=f"out/{tag}_fig{i+1}.png")
        # marks + notes: mark the goal box, add a note, reload, and the note must come back under this page's key
        blk = pg.locator('.blk[data-b="goal"]')
        blk.hover(); blk.locator('button.dot[title="question"]').click()
        blk.locator("text=+ Note").click(); pg.wait_for_timeout(200)
        pg.keyboard.type("sample note on " + tag); pg.wait_for_timeout(600)
        pg.reload(); pg.wait_for_selector(".blk")
        r["note_after_reload"] = pg.locator('.blk[data-b="goal"] textarea').input_value()
        r["mark_after_reload"] = pg.locator('.blk[data-b="goal"]').get_attribute("class")
        page_id = pg.evaluate("PAGE")
        r["ls_keys"] = pg.evaluate(f"Object.keys(JSON.parse(localStorage.getItem('ardp_v2_sec_{page_id}')||'{{}}'))")
        if r["note_after_reload"] != "sample note on " + tag or r["ls_keys"] != [page_id + ":goal"]:
            res["errors"].append(f"{page_path.name}: note did not survive reload under key {page_id}:goal")
        # every table has its own marks + notes (owner's order 2026-10-20)
        r["tables"] = pg.locator("main table").count()
        r["table_tools"] = pg.locator(".tblk .ttools").count()
        if r["tables"] != r["table_tools"]:
            res["errors"].append(f"{page_path.name}: {r['tables']} tables but {r['table_tools']} table tools")
        tb = pg.locator(".tblk").nth(1)
        tkey = page_id + ":" + tb.get_attribute("data-t")
        tb.hover(); tb.locator('.ttools button.dot[title="change this"]').click()
        tb.locator(".ttools >> text=+ Note").click(); pg.wait_for_timeout(200)
        pg.keyboard.type("table note on " + tag); pg.wait_for_timeout(600)
        pg.reload(); pg.wait_for_selector(".blk")
        tb = pg.locator(f'.tblk[data-t="{tkey.split(":", 1)[1]}"]')
        r["table_note_after_reload"] = tb.locator(".tnotes textarea").input_value()
        r["table_class_after_reload"] = tb.get_attribute("class")
        if r["table_note_after_reload"] != "table note on " + tag or "mk-red" not in (r["table_class_after_reload"] or ""):
            res["errors"].append(f"{page_path.name}: table note/mark did not survive reload under {tkey}")
        tb.screenshot(path=f"out/{tag}_table_tools.png")
    b.close()
for name, (s, val) in expect.items():
    res["numbers"][name] = {"printed": s, "value": round(val, 6), "found": s in text}
bad = [k for k, v in res["numbers"].items() if not v["found"]]
res["missing_numbers"] = bad
Path("out/check_2_1.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"missing": bad, "errors": res["errors"], "pages": res["pages"]}, ensure_ascii=False))
sys.exit(1 if bad or res["errors"] else 0)
