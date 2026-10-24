"""The numbers of the Think problems (2.2a, 2.2b, 2.2c, 2.2d), computed independently of the pages.
check_2_2d.py imports compute() from here (the cloud copy sits next to it). Run alone, it prints every value:
    python think_calc.py          (cloud machine only, never on the owner's PC)
"""
import json
import math
import sys

sq = math.sqrt
sind = lambda d: math.sin(math.radians(d))
cosd = lambda d: math.cos(math.radians(d))
asind = lambda x: math.degrees(math.asin(x))
atan2d = lambda y, x: math.degrees(math.atan2(y, x))
g, gD = 9.8062, 9.7965
rho, rhoD = 1.225, 1.049


def compute():
    c = {}
    # ---------------- Think 1 (2.2a): two legs, two speeds
    L, vo, vb = 120.0, 8.0, 24.0
    c["k1_tout"] = L / vo
    c["k1_tback"] = L / vb
    c["k1_T"] = c["k1_tout"] + c["k1_tback"]
    c["k1_speed"] = 2 * L / c["k1_T"]
    c["k1_naive"] = (vo + vb) / 2
    c["k1_slow_share"] = c["k1_tout"] / c["k1_T"] * 100
    c["k1_T16"] = 2 * L / 16.0
    c["k1_tleft"] = c["k1_T16"] - c["k1_tout"]
    # ---------------- Think 2 (2.2a): the third second
    c["k2_a"] = 10.0 / 2.5
    c["k2_x3"] = 0.5 * c["k2_a"] * 9
    c["k2_v3"] = c["k2_a"] * 3
    c["k2_d1"] = 0.5 * c["k2_a"] * 1
    c["k2_d2"] = 0.5 * c["k2_a"] * (4 - 1)
    c["k2_d3"] = 0.5 * c["k2_a"] * (9 - 4)
    c["k2_vmid"] = c["k2_a"] * 2.5
    c["k2_a_w"] = (10.0 - 5.0) / 2.5
    c["k2_x3_w"] = 5.0 * 3 + 0.5 * c["k2_a_w"] * 9
    # ---------------- Think 3 (2.2a): braking with a delay
    v0, ab, vg, dly, D = 24.0, 8.0, 8.0, 0.25, 35.0
    c["k3_dbrake"] = (v0 ** 2 - vg ** 2) / (2 * ab)
    c["k3_ddelay"] = v0 * dly
    c["k3_dneed"] = c["k3_dbrake"] + c["k3_ddelay"]
    c["k3_dleft"] = D - c["k3_ddelay"]
    c["k3_v2"] = v0 ** 2 - 2 * ab * c["k3_dleft"]
    c["k3_v"] = sq(c["k3_v2"])
    c["k3_anead"] = (v0 ** 2 - vg ** 2) / (2 * c["k3_dleft"])
    c["k3_early"] = c["k3_dneed"] - D
    c["k3_w_delay"] = v0 * 0.10
    c["k3_w_total"] = c["k3_dbrake"] + c["k3_w_delay"]
    c["k3_w_left"] = D - c["k3_w_delay"]
    c["k3_w_v2"] = v0 ** 2 - 2 * ab * c["k3_w_left"]
    c["k3_w_v"] = sq(c["k3_w_v2"])
    # ---------------- Think 4 (2.2a): a short run, rest to rest
    Ls, a1, a2, vcap = 60.0, 6.0, 9.0, 24.0
    c["k4_k"] = 1 / (2 * a1) + 1 / (2 * a2)
    c["k4_v2"] = Ls / c["k4_k"]
    c["k4_v"] = sq(c["k4_v2"])
    c["k4_t1"] = c["k4_v"] / a1
    c["k4_t2"] = c["k4_v"] / a2
    c["k4_T"] = c["k4_t1"] + c["k4_t2"]
    c["k4_d1"] = c["k4_v2"] / (2 * a1)
    c["k4_d2"] = c["k4_v2"] / (2 * a2)
    c["k4_up"] = vcap ** 2 / (2 * a1)
    c["k4_dn"] = vcap ** 2 / (2 * a2)
    c["k4_need"] = c["k4_up"] + c["k4_dn"]
    # ---------------- Think 5 (2.2b): two limits, one gate
    dE, dN, dz, vh, vzm = 90.0, 60.0, 20.0, 10.0, 3.0
    c["k5_dh"] = sq(dE ** 2 + dN ** 2)
    c["k5_th"] = c["k5_dh"] / vh
    c["k5_tv"] = dz / vzm
    c["k5_T"] = max(c["k5_th"], c["k5_tv"])
    c["k5_vz"] = dz / c["k5_T"]
    c["k5_vE"] = dE / c["k5_T"]
    c["k5_vN"] = dN / c["k5_T"]
    c["k5_speed"] = sq(vh ** 2 + c["k5_vz"] ** 2)
    c["k5_sum"] = c["k5_th"] + c["k5_tv"]
    dz2 = 40.0
    c["k5_tv2"] = dz2 / vzm
    c["k5_vh2"] = c["k5_dh"] / c["k5_tv2"]
    c["k5_speed2"] = sq(c["k5_vh2"] ** 2 + vzm ** 2)
    # ---------------- Think 6 (2.2b): the lap, one phase better
    c["k6_ds3_old"] = 0.5 * (20 + 8) * 3.0
    c["k6_t3"] = (8 - 20) / (-6.0)
    c["k6_ds3"] = 0.5 * (20 + 8) * c["k6_t3"]
    c["k6_freed"] = c["k6_ds3_old"] - c["k6_ds3"]
    c["k6_extra"] = c["k6_freed"] / 20.0
    c["k6_cruise"] = 3.0 + c["k6_extra"]
    c["k6_T"] = 4.0 + c["k6_cruise"] + c["k6_t3"] + 5.0 + 2.0
    c["k6_saved"] = 17.0 - c["k6_T"]
    c["k6_naive"] = 3.0 - c["k6_t3"]
    c["k6_stot"] = 40.0 + (60.0 + c["k6_freed"]) + c["k6_ds3"] + 40.0 + 28.0
    c["k6_t5"] = (20 - 8) / 8.0
    c["k6_ds5"] = 0.5 * (8 + 20) * c["k6_t5"]
    c["k6_freed5"] = 28.0 - c["k6_ds5"]
    c["k6_extra5"] = c["k6_freed5"] / 20.0
    c["k6_saved5"] = (2.0 - c["k6_t5"]) - c["k6_extra5"]
    c["k6_T5"] = 17.0 - c["k6_saved5"]
    # ---------------- Think 7 (2.2b): the parachute
    tfree, adec, vend = 2.0, 15.0, 5.0
    c["k7_vfall"] = g * tfree
    c["k7_dfall"] = 0.5 * g * tfree ** 2
    c["k7_dslow"] = (c["k7_vfall"] ** 2 - vend ** 2) / (2 * adec)
    c["k7_h"] = c["k7_dfall"] + c["k7_dslow"]
    c["k7_tslow"] = (c["k7_vfall"] - vend) / adec
    c["k7_T"] = tfree + c["k7_tslow"]
    c["k7_hleft"] = 25.0 - c["k7_dfall"]
    c["k7_v2"] = c["k7_vfall"] ** 2 - 2 * adec * c["k7_hleft"]
    c["k7_v"] = sq(c["k7_v2"])
    # ---------------- Think 8 (2.2b): a heavier fixed wing
    mu, V0, a0 = 1.2, 25.0, 2.5
    c["k8_s0"] = V0 ** 2 / (2 * a0)
    c["k8_V2"] = V0 ** 2 * mu
    c["k8_V"] = sq(c["k8_V2"])
    c["k8_a"] = a0 / mu
    c["k8_2a"] = 2 * c["k8_a"]
    c["k8_s"] = c["k8_V2"] / c["k8_2a"]
    c["k8_k"] = sq(150.0 / c["k8_s0"])
    c["k8_pct"] = (c["k8_k"] - 1) * 100
    # ---------------- Think 9 (2.2b): the sign and the size of the error
    ab9, t9 = 8.0, 3.0
    c["k9_coef"] = 0.5 * ab9 * t9
    c["k9_e02"] = c["k9_coef"] * 0.02
    c["k9_e003"] = c["k9_coef"] * 0.003
    c["k9_dtmax"] = 0.05 / c["k9_coef"]
    c["k9_rate"] = 1 / c["k9_dtmax"]
    # ---------------- Think 10 (2.2c): from the time to the speed
    A, J = 3.0, 4.0
    c["k10_thr"] = 2 * A / J
    c["k10_AJ"] = A / J
    c["k10_V1"] = (2.75 - A / J) * A
    c["k10_V2"] = J * (1.0 / 2) ** 2
    c["k10_amax2"] = J * 1.0 / 2
    c["k10_sat_thr_V"] = A ** 2 / J
    # ---------------- Think 11 (2.2c): the distance sets the top speed
    half = 6.0
    c["k11_b"] = A ** 2 / J
    c["k11_c"] = 2 * A * half
    c["k11_V"] = 0.5 * (-c["k11_b"] + sq(c["k11_b"] ** 2 + 4 * c["k11_c"]))
    c["k11_tr"] = c["k11_V"] / A + A / J
    c["k11_T"] = 2 * c["k11_tr"]
    c["k11_xr"] = c["k11_V"] * c["k11_tr"] / 2
    c["k11_full"] = 6.0 * (6.0 / A + A / J) / 2
    # ---------------- Think 12 (2.2c): the slow-down distance
    V12 = 6.0
    d12 = lambda a_: V12 ** 2 / (2 * a_) + (2 * a_ / J) * V12
    c["k12_d2"] = d12(2.0)
    c["k12_d3"] = d12(3.0)
    c["k12_d4"] = d12(4.0)
    c["k12_Astar"] = sq(V12 * J / 4)
    c["k12_dmin"] = d12(c["k12_Astar"])
    c["k12_first2"] = V12 ** 2 / (2 * 2.0)
    c["k12_second2"] = (2 * 2.0 / J) * V12
    c["k12_first3"] = V12 ** 2 / (2 * 3.0)
    c["k12_second3"] = (2 * 3.0 / J) * V12
    # ---------------- Think 13 (2.2c): NED and the path follower
    gE, gN, gU = 40 - 10, 15 - 35, 8 - 3
    c["k13_dist"] = sq(gE ** 2 + gN ** 2 + gU ** 2)
    sp = 5.0
    c["k13_vN"] = sp * gN / c["k13_dist"]
    c["k13_vE"] = sp * gE / c["k13_dist"]
    c["k13_vD"] = sp * (-gU) / c["k13_dist"]
    c["k13_t_plain"] = c["k13_dist"] / sp
    c["k13_brk"] = min(max(sp * 0.5, 1.0), 3.0)
    c["k13_t_brake_zone"] = c["k13_brk"] / 1.0
    c["k13_t_main"] = (c["k13_dist"] - c["k13_brk"]) / sp
    c["k13_t"] = c["k13_t_main"] + c["k13_t_brake_zone"]
    c["k13_t_slow"] = c["k13_dist"] / 2.5
    # ---------------- Think 14 (2.2d): the fall after a motor failure, the model problem
    Va, wspd, wfrom, h = 12.0, 6.0, 240.0, 40.0
    wto = (wfrom + 180) % 360
    c["k14_toward"] = wto
    c["k14_wE"] = wspd * sind(wto)
    c["k14_wN"] = wspd * cosd(wto)
    c["k14_vE"] = c["k14_wE"]
    c["k14_vN"] = Va + c["k14_wN"]
    c["k14_speed"] = sq(c["k14_vE"] ** 2 + c["k14_vN"] ** 2)
    c["k14_angle"] = atan2d(c["k14_vE"], c["k14_vN"])
    c["k14_arg"] = 2 * h / g
    c["k14_t"] = sq(c["k14_arg"])
    t = round(c["k14_t"], 3)
    c["k14_dN"] = 15.0 * t
    c["k14_dE"] = round(c["k14_wE"], 3) * t
    c["k14_d"] = sq(c["k14_dN"] ** 2 + c["k14_dE"] ** 2)
    c["k14_vz"] = g * t
    c["k14_vtot"] = sq(252.0 + round(c["k14_vz"], 2) ** 2)
    c["k14_check"] = c["k14_speed"] * t
    # ---------------- Think 15 (2.2d): the motors stop in a 30 degree climb with a tailwind
    Va, clb, wsp, z0 = 10.0, 30.0, 5.0, 25.0
    c["k15_vh_air"] = Va * cosd(clb)
    c["k15_vz0"] = Va * sind(clb)
    c["k15_vN"] = c["k15_vh_air"] + wsp
    c["k15_disc"] = c["k15_vz0"] ** 2 + 2 * g * z0
    c["k15_root"] = sq(c["k15_disc"])
    c["k15_t"] = (c["k15_vz0"] + c["k15_root"]) / g
    t15 = round(c["k15_t"], 3)
    c["k15_x"] = round(c["k15_vN"], 3) * t15
    c["k15_vtot"] = sq(round(c["k15_vN"], 3) ** 2 + round(c["k15_root"], 2) ** 2)
    c["k15_x_nowind"] = round(c["k15_vh_air"], 3) * t15
    c["k15_extra"] = wsp * t15
    c["k15_ttop"] = c["k15_vz0"] / g
    c["k15_ztop"] = z0 + c["k15_vz0"] ** 2 / (2 * g)
    c["k15_angle_imp"] = atan2d(c["k15_root"], c["k15_vN"])
    # ---------------- Think 16 (2.2d): a lap of four legs in a steady wind
    V, wsp, wfrom, Ll = 15.0, 6.0, 300.0, 300.0
    wto = (wfrom + 180) % 360
    wE, wN = wsp * sind(wto), wsp * cosd(wto)
    c["k16_toward"] = wto
    c["k16_wE"], c["k16_wN"] = wE, wN
    legs = []
    for name, track in (("north", 0.0), ("east", 90.0), ("south", 180.0), ("west", 270.0)):
        uE, uN = sind(track), cosd(track)
        along = wE * uE + wN * uN
        cross = wE * uN - wN * uE            # positive: the wind pushes to the right of the track
        cc = sq(V ** 2 - cross ** 2)
        gs = cc + along
        tt = Ll / gs
        crab = asind(abs(cross) / V)
        head = (track - (1 if cross > 0 else -1) * crab) % 360
        legs.append(dict(name=name, track=track, along=along, cross=cross, cc=cc, gs=gs, t=tt, crab=crab, head=head))
    c["k16_legs"] = legs
    c["k16_T"] = sum(l["t"] for l in legs)
    c["k16_T0"] = 4 * Ll / V
    c["k16_loss"] = c["k16_T"] - c["k16_T0"]
    c["k16_pct"] = c["k16_loss"] / c["k16_T0"] * 100
    # ---------------- Think 17 (2.2d): takeoff and landing rolls at Denver in a wind
    V0e, a0e, mu = 27.0, 2.141, 1.05
    f = round(sq(mu), 4)
    V = round(V0e * f, 2)
    a = round(a0e / mu, 3)
    c["k17_f"], c["k17_V"], c["k17_a"] = f, V, a
    w, wfrom = 8.0, 240.0
    c["k17_hw27"] = w * cosd(wfrom - 270)
    c["k17_cw27"] = abs(w * sind(wfrom - 270))
    c["k17_hw09"] = w * cosd(wfrom - 90)
    c["k17_cw09"] = abs(w * sind(wfrom - 90))
    hw = round(c["k17_hw27"], 2)
    c["k17_gs27"] = V - hw
    c["k17_s27"] = c["k17_gs27"] ** 2 / (2 * a)
    c["k17_gs09"] = V + hw
    c["k17_s09"] = c["k17_gs09"] ** 2 / (2 * a)
    c["k17_s0"] = V ** 2 / (2 * a)
    Vtd, dec = 25.0, 2.0
    c["k17_ls27_gs"] = Vtd - hw
    c["k17_ls27"] = c["k17_ls27_gs"] ** 2 / (2 * dec)
    c["k17_ls09_gs"] = Vtd + hw
    c["k17_ls09"] = c["k17_ls09_gs"] ** 2 / (2 * dec)
    c["k17_gsneed2"] = 2 * a * 150.0
    c["k17_gsneed"] = sq(c["k17_gsneed2"])
    c["k17_wh"] = V - c["k17_gsneed"]
    c["k17_wspeed"] = c["k17_wh"] / 0.866
    # ---------------- Think 18 (2.2d): stop over a pad with a parcel and a tailwind
    m, Tmax, W = 3.5, 60.0, 3.5 * g
    c["k18_W"] = W
    vgnd = 12.0 + 6.0
    D18, vdesc = 20.0, 3.0
    c["k18_vg"] = vgnd
    c["k18_ah"] = vgnd ** 2 / (2 * D18)
    c["k18_t"] = 2 * D18 / vgnd
    c["k18_aup"] = vdesc / c["k18_t"]
    c["k18_depth"] = 0.5 * vdesc * c["k18_t"]
    c["k18_gsum"] = g + c["k18_aup"]
    c["k18_T"] = m * sq(c["k18_ah"] ** 2 + c["k18_gsum"] ** 2)
    c["k18_rotor"] = c["k18_T"] / 4
    c["k18_share"] = c["k18_rotor"] / 15.0 * 100
    c["k18_hover_share"] = (W / 4) / 15.0 * 100
    c["k18_rise"] = (c["k18_T"] / W - 1) * 100
    c["k18_dstop_A3"] = vgnd ** 2 / (2 * 3.0)
    c["k18_vmax_A3"] = sq(2 * 3.0 * D18)
    c["k18_air_A3"] = c["k18_vmax_A3"] - 6.0
    c["k18_dstop_nowind"] = 12.0 ** 2 / (2 * 3.0)
    vg2 = 24.0
    c["k18_w_ah"] = vg2 ** 2 / (2 * D18)
    c["k18_w_t"] = 2 * D18 / vg2
    c["k18_w_aup"] = vdesc / c["k18_w_t"]
    c["k18_w_T"] = m * sq(c["k18_w_ah"] ** 2 + (g + c["k18_w_aup"]) ** 2)
    # ---------------- Think 19 (2.2d): a climb to a gate with a headwind and an updraft
    dx, dzg = 60.0, 18.0
    vair_h, vair_z, wx, wz = 10.0, 3.0, -4.0, 1.5
    c["k19_gx"] = vair_h + wx
    c["k19_gz"] = vair_z + wz
    c["k19_t"] = dx / c["k19_gx"]
    c["k19_z"] = 2.0 + c["k19_gz"] * c["k19_t"]
    c["k19_vz_need"] = dzg / c["k19_t"]
    c["k19_air_need"] = c["k19_vz_need"] - wz
    wz2 = -1.5
    c["k19_air_need2"] = c["k19_vz_need"] - wz2
    c["k19_gz_max"] = vair_z + wz2
    c["k19_t_min"] = dzg / c["k19_gz_max"]
    c["k19_gx_max"] = dx / c["k19_t_min"]
    c["k19_air_h_max"] = c["k19_gx_max"] - wx
    # ---------------- Think 20 (2.2d): gusts, phase by phase
    Vair, Lg = 12.0, 120.0
    phases = [(0, 6, 4.0, 4.0), (6, 8, 4.0, 10.0), (8, 12, 10.0, 10.0), (12, 14, 10.0, 4.0)]
    x = 0.0
    rows = []
    for t0, t1, w0, w1 in phases:
        v0_, v1_ = Vair - w0, Vair - w1
        dxp = 0.5 * (v0_ + v1_) * (t1 - t0)
        x += dxp
        rows.append(dict(t0=t0, t1=t1, w0=w0, w1=w1, v0=v0_, v1=v1_, dx=dxp, x=x))
    c["k20_rows"] = rows
    c["k20_x14"] = x
    c["k20_rest"] = Lg - x
    c["k20_trest"] = c["k20_rest"] / (Vair - 4.0)
    c["k20_T"] = 14.0 + c["k20_trest"]
    c["k20_plan"] = Lg / (Vair - 4.0)
    c["k20_extra"] = c["k20_T"] - c["k20_plan"]
    c["k20_x_plan_time"] = x + (c["k20_plan"] - 14.0) * (Vair - 4.0)
    c["k20_short"] = Lg - c["k20_x_plan_time"]
    c["k20_avg_gs"] = c["k20_x_plan_time"] / c["k20_plan"]
    # ---------------- Think 21 (2.2d): an accelerometer bias without GPS
    b, tol, vfl, spacing = 0.04, 1.5, 10.0, 40.0
    c["k21_t_miss"] = sq(2 * tol / b)
    c["k21_x_miss"] = vfl * c["k21_t_miss"]
    gt = [spacing / vfl * k for k in (1, 2, 3)]
    c["k21_gate_t"] = gt
    c["k21_gate_e"] = [0.5 * b * t_ ** 2 for t_ in gt]
    c["k21_v12"] = b * gt[2]
    c["k21_tilt"] = asind(b / g)
    c["k21_tilt_rad"] = b / g
    b2 = 0.01
    c["k21_t_miss2"] = sq(2 * tol / b2)
    c["k21_e16"] = 0.5 * b2 * 16.0 ** 2
    c["k21_e20"] = 0.5 * b2 * 20.0 ** 2
    c["k21_ratio"] = c["k21_t_miss2"] / c["k21_t_miss"]
    # ---------------- Think 22 (2.2d): the S-curve with a parcel
    Jd, Vc, Dd = 4.0, 6.0, 20.0
    for tag, A_ in (("d", 3.0), ("p", 2.0)):
        c[f"k22_{tag}_thr"] = A_ ** 2 / Jd
        c[f"k22_{tag}_T"] = Vc / A_ + A_ / Jd
        c[f"k22_{tag}_x"] = Vc * c[f"k22_{tag}_T"] / 2
        c[f"k22_{tag}_two"] = 2 * c[f"k22_{tag}_x"]
    c["k22_d_cruise"] = Dd - c["k22_d_two"]
    c["k22_d_tcruise"] = c["k22_d_cruise"] / Vc
    c["k22_d_total"] = 2 * c["k22_d_T"] + c["k22_d_tcruise"]
    A_ = 2.0
    c["k22_b"] = A_ ** 2 / Jd
    c["k22_c"] = 2 * A_ * (Dd / 2)
    c["k22_Vpeak"] = 0.5 * (-c["k22_b"] + sq(c["k22_b"] ** 2 + 4 * c["k22_c"]))
    c["k22_tr"] = c["k22_Vpeak"] / A_ + A_ / Jd
    c["k22_total"] = 2 * c["k22_tr"]
    c["k22_xr"] = c["k22_Vpeak"] * c["k22_tr"] / 2
    c["k22_lost"] = c["k22_total"] - c["k22_d_total"]
    bb = 4 * A_ ** 2 / Jd
    c["k22_px4_v"] = 0.5 * (-bb + sq(bb ** 2 + 8 * A_ * c["k22_p_x"]))
    # ---------------- Think 23 (2.2d): the wind from two ground speeds
    Va, g1, g2, g3 = 15.0, 20.22, 22.29, 13.00
    dd = (g1 ** 2 - g2 ** 2) / (2 * Va)
    q = Va + dd
    B, C = 2 * q, q * q - g1 ** 2
    disc = B * B - 8 * C
    sols = []
    for sgn in (1, -1):
        wx = (-B + sgn * sq(disc)) / 4
        wy = wx + dd
        sp_ = sq(wx ** 2 + wy ** 2)
        toward = atan2d(wx, wy) % 360
        sols.append(dict(wx=wx, wy=wy, speed=sp_, toward=toward, frm=(toward + 180) % 360, g3=sq(wx ** 2 + (wy - Va) ** 2)))
    c["k23_d"] = dd
    c["k23_q"] = q
    c["k23_B"], c["k23_C"], c["k23_disc"] = B, C, disc
    c["k23_sols"] = sols
    # ---------------- Think 24 (2.2d): the best plan
    Lb, vmax, ab = 200.0, 16.0, 4.0
    plan = {}
    for v_ in (16.0, 14.0, 12.0, 10.0):
        plan[v_] = dict(tup=v_ / ab, xup=v_ ** 2 / (2 * ab), cr=Lb - v_ ** 2 / ab, T=v_ / ab + Lb / v_)
        plan[v_]["tcr"] = plan[v_]["cr"] / v_
    c["k24_plan"] = plan
    c["k24_vopt"] = sq(ab * Lb)
    c["k24_Ttri"] = 2 * sq(Lb / ab)
    c["k24_J"] = 4.0
    c["k24_tr_S"] = vmax / ab + ab / c["k24_J"]
    c["k24_xr_S"] = vmax * c["k24_tr_S"] / 2
    c["k24_cr_S"] = Lb - 2 * c["k24_xr_S"]
    c["k24_T_S"] = 2 * c["k24_tr_S"] + c["k24_cr_S"] / vmax
    # ---------------- Think 10a (2.2b): the Earth rate on the gyroscope (exact values, no rounded inputs)
    Om = 7.292115e-5
    phi = math.radians(39.74)
    om_deg = math.degrees(Om)                              # deg/s, exact conversion (no 57.30)
    down = om_deg * math.sin(phi)
    c["k10a_om_deg"] = om_deg
    c["k10a_down"] = down
    c["k10a_h17"], c["k10a_h300"], c["k10a_h3600"] = down * 17, down * 300, down * 3600
    eps = Om * math.sin(phi)                               # rad/s
    c["k10a_eps"] = eps
    c["k10a_pos"] = 0.5 * 20.0 * eps * 300.0 ** 2          # small angle, sideways speed v*eps*t
    c["k10a_posB"] = 0.5 * 20.0 * math.radians(0.020) * 300.0 ** 2
    c["k10a_ratio"] = 0.020 / down
    c["k10a_h300_60"] = om_deg * math.sin(math.radians(60.0)) * 300.0
    # exact straight-line path with a heading that grows as eps*t: cross-track error = v * integral of sin(eps*t) dt (numerical sum, 0.01 s steps)
    s = 0.0
    for i in range(30000):
        s += 20.0 * math.sin(eps * (i + 0.5) * 0.01) * 0.01
    c["k10a_pos_sin"] = s
    # ---------------- Think 10b (2.2b): the error state, covariance step by step
    Vi = (0.5 * 0.01) ** 2
    P, PP = 0.0, {}
    for i in range(1, 1001):
        P += Vi                                            # Fx = 1: P <- P + Vi at every step
        if i in (100, 1000):
            PP[i] = P
    c["k10b_P1"], c["k10b_P10"] = PP[100], PP[1000]
    Vg = 0.10 ** 2
    Kg = P / (P + Vg)
    c["k10b_K"] = Kg
    c["k10b_dv"] = Kg * (12.10 - 12.30)
    c["k10b_v"] = 12.30 + c["k10b_dv"]
    c["k10b_Pafter"] = (1 - Kg) * P
    c["k10b_Pafter_sum"] = 1.0 / (1.0 / P + 1.0 / Vg)
    Kw = P / (P + 0.50 ** 2)
    c["k10b_Kw"] = Kw
    c["k10b_vw"] = 12.30 + Kw * (12.10 - 12.30)
    c["k10b_Pw"] = (1 - Kw) * P
    # ---------------- Think 10c (2.2b): SGal(3) step as a 5x5 matrix, the exponential by its series
    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(5)) for j in range(5)] for i in range(5)]
    def expm(Nn):
        out_ = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
        term = [row[:] for row in out_]
        for n in range(1, 12):
            term = [[x / n for x in row] for row in mm(term, Nn)]
            out_ = [[out_[i][j] + term[i][j] for j in range(5)] for i in range(5)]
        return out_
    dt_, a_ = 0.5, 2.0
    # layout of the paper's 5x5 matrix: rows 0-2 and columns 0-2 hold R, column 3 holds v, column 4 holds p, and the time t sits in row 3, column 4.
    # The matrix of the step vector (0, a, 0, 1)*dt has a*dt in row 0, column 3 (velocity column, x component) and dt in row 3, column 4.
    Nn = [[0.0, 0.0, 0.0, a_ * dt_, 0.0], [0.0] * 5, [0.0] * 5, [0.0, 0.0, 0.0, 0.0, dt_], [0.0] * 5]
    S_ = expm(Nn)
    c["k10c_S_dv"], c["k10c_S_dp"], c["k10c_S_dt"] = S_[0][3], S_[0][4], S_[3][4]
    G = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    G[3][3] = 1.0
    G[4][4] = 1.0
    traj = []
    for k in range(4):
        G = mm(G, S_)
        traj.append(dict(v=G[0][3], p=G[0][4], t=G[3][4]))
    c["k10c_traj"] = traj
    return c


if __name__ == "__main__":
    out = compute()
    print(json.dumps(out, indent=1, default=str))
