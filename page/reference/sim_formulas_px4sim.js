window.SIM_FORMULAS_PX4SIM = [
 {
  "id": "g01-sih-isa",
  "group": 1,
  "group_name": "atmosphere and gravity",
  "quantity": "ISA pressure, temperature, density vs altitude (aero segments only)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "459-462",
  "code": "_pressure = P0 * powf(1.0f - 0.0065f * alt / T0_K, 5.2561f);\n_temperature = T0_K + TEMP_GRADIENT * alt;\n_rho = _pressure / R / _temperature;",
  "formula": "p = p0·(1 − 0.0065·h/T0)^5.2561 ; T = T0 − 0.0065·h ; ρ = p/(R·T)",
  "variables": {
   "h": "altitude AMSL from _lla.altitude() [m]",
   "p0": "101325 Pa",
   "T0": "288.15 K",
   "R": "287.04 J/(kg·K)",
   "ρ": "air density [kg/m³]"
  },
  "defaults": [
   {
    "name": "P0",
    "value": "101325",
    "unit": "Pa",
    "where": "aero.hpp:323"
   },
   {
    "name": "R",
    "value": "287.04",
    "unit": "J/(kg K)",
    "where": "aero.hpp:324"
   },
   {
    "name": "T0_K",
    "value": "288.15",
    "unit": "K",
    "where": "aero.hpp:325"
   },
   {
    "name": "TEMP_GRADIENT",
    "value": "-6.5e-3",
    "unit": "K/m",
    "where": "aero.hpp:326"
   }
  ],
  "assumptions": "Troposphere layer only; computed separately inside every AeroSeg; the multicopter branch of SIH never calls it (multicopter has no density dependence). Constant R=287.04 here, but atmosphere.h uses 287.1.",
  "notes": "Comment says 'ISA model taken from Mustafa Cavcar'. Fixed-wing/VTOL only. For the tailsitter, altitude is passed as _lpos_ref_alt - _lpos(2) (sih.cpp:550) rather than _lla.altitude()."
 },
 {
  "id": "g01-sih-gravity",
  "group": 1,
  "group_name": "atmosphere and gravity",
  "quantity": "gravity vs latitude (Somigliana normal gravity, ellipsoid surface)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/lat_lon_alt/lat_lon_alt.hpp",
  "lines": "119-126",
  "code": "const double sin_lat = sin(latitude_rad);\nconst double sin_lat2 = sin_lat * sin_lat;\nconst double g = gravity_equator * (1.0 + somigliana_coefficient * sin_lat2)\n\t\t / std::sqrt(1.0 - eccentricity2 * sin_lat2);\nreturn static_cast<float>(g);",
  "formula": "g(φ) = g_e·(1 + k·sin²φ)/√(1 − e²·sin²φ)",
  "variables": {
   "φ": "geodetic latitude [rad]",
   "g_e": "9.7803253359 m/s²",
   "k": "0.001931851353 (Somigliana)",
   "e²": "first eccentricity squared (0.0818191908425²)"
  },
  "defaults": [
   {
    "name": "gravity_equator",
    "value": "9.7803253359",
    "unit": "m/s^2",
    "where": "lat_lon_alt.hpp:114"
   },
   {
    "name": "somigliana_coefficient",
    "value": "0.001931851353",
    "unit": "-",
    "where": "lat_lon_alt.hpp:115"
   },
   {
    "name": "eccentricity",
    "value": "0.0818191908425",
    "unit": "-",
    "where": "lat_lon_alt.hpp:111"
   }
  ],
  "assumptions": "Gravity on the ellipsoid surface; NO decrease with altitude (free-air term absent).",
  "notes": "Used by sih.cpp:641-642 as gravity_acceleration_E = R_N2E.col(2) * gravity(lat) (along local Down in ECEF). Elsewhere PX4 uses CONSTANTS_ONE_G (atmosphere.cpp)."
 },
 {
  "id": "g01-sih-ecef",
  "group": 1,
  "group_name": "atmosphere and gravity",
  "quantity": "WGS84 geodetic to ECEF position (Earth shape used by SIH)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/lat_lon_alt/lat_lon_alt.cpp",
  "lines": "75-80",
  "code": "const double r_e = Wgs84::equatorial_radius / sqrt(1.0 - std::pow(Wgs84::eccentricity * sin_lat, 2.0));\nconst double r_total = r_e + static_cast<double>(_altitude);\nreturn Vector3d(r_total * cos_lat * cos_lon,\n\tr_total * cos_lat * sin_lon,\n\t((1.0 - Wgs84::eccentricity2) * r_e + static_cast<double>(_altitude)) * sin_lat);",
  "formula": "x = (N+h)cosφcosλ ; y = (N+h)cosφsinλ ; z = ((1−e²)N + h)sinφ ; N = a/√(1−e²sin²φ)",
  "variables": {
   "a": "6378137 m",
   "N": "prime-vertical radius",
   "h": "altitude [m]"
  },
  "defaults": [
   {
    "name": "equatorial_radius",
    "value": "6378137.0",
    "unit": "m",
    "where": "lat_lon_alt.hpp:110"
   }
  ],
  "assumptions": "Position state of SIH is kept in ECEF; the inverse fromEcef (lat_lon_alt.cpp:44-65) is the Borkowski closed form.",
  "notes": "Frames: the aircraft state is integrated in ECEF, then converted to NED each step (sih.cpp ecefToNed 716-728). Also meridian/transverse radii at lat_lon_alt.cpp:108-111."
 },
 {
  "id": "g02-sih-multirotor-linear",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "quadcopter total thrust and reaction torque (linear in command)",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "459-462",
  "code": "_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));\n_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]),\n\t\t _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),\n\t\t _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));",
  "formula": "T_B = (0, 0, −T_max·Σu_i) ; M_x = L_roll·T_max·(−u0+u1+u2−u3) ; M_y = L_pitch·T_max·(u0−u1+u2−u3) ; M_z = Q_max·(u0+u1−u2−u3)",
  "variables": {
   "u_i": "lagged motor command in [0,1]",
   "T_max": "SIH_T_MAX max thrust per rotor [N]",
   "Q_max": "SIH_Q_MAX max torque per rotor [Nm]",
   "L_roll,L_pitch": "arm lengths SIH_L_ROLL, SIH_L_PITCH [m]"
  },
  "defaults": [
   {
    "name": "SIH_T_MAX",
    "value": "5.0",
    "unit": "N",
    "where": "sih_params.yaml:95"
   },
   {
    "name": "SIH_Q_MAX",
    "value": "0.1",
    "unit": "Nm",
    "where": "sih_params.yaml:111"
   },
   {
    "name": "SIH_L_ROLL",
    "value": "0.2",
    "unit": "m",
    "where": "sih_params.yaml:125"
   },
   {
    "name": "SIH_L_PITCH",
    "value": "0.2",
    "unit": "m",
    "where": "sih_params.yaml:139"
   }
  ],
  "assumptions": "Thrust LINEAR in u (not u²); no density, advance-ratio or battery-voltage dependence; thrust acts along body -Z; x-quad layout hard-coded; also this row is the control mixer of group 12 and the moment arm model of group 9.",
  "notes": "Hexacopter uses u·|u| (quadratic) instead, see g02-sih-hexa. Inconsistency inside SIH: quad linear in u, hexa quadratic in u."
 },
 {
  "id": "g02-sih-hexa",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "hexacopter thrust and torque (quadratic in command)",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "477-484",
  "code": "u_sq[i] = _u[i] * fabsf(_u[i]);\n_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+u_sq[0] + u_sq[1] + u_sq[2] + u_sq[3] + u_sq[4] + u_sq[5]));\n_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-.5f * u_sq[0] - u_sq[1] - .5f * u_sq[2] + .5f * u_sq[3] + u_sq[4] + .5f * u_sq[5]),\n\t\t _L_PITCH * _T_MAX * (M_SQRT3_F / 2.f) * (+u_sq[0] - u_sq[2] - u_sq[3] + u_sq[5]),\n\t\t _Q_MAX * (+u_sq[0] - u_sq[1] + u_sq[2] - u_sq[3] + u_sq[4] - u_sq[5]));",
  "formula": "T_i = T_max·u_i·|u_i| ; T_B = (0,0,−ΣT_i) ; M_x = L_roll·T_max·(−½u0²−u1²−½u2²+½u3²+u4²+½u5²) ; M_y = L_pitch·T_max·(√3/2)(u0²−u2²−u3²+u5²) ; M_z = Q_max·(u0²−u1²+u2²−u3²+u4²−u5²)",
  "variables": {
   "u_i": "lagged motor command [0,1]",
   "T_max": "SIH_T_MAX [N]",
   "Q_max": "SIH_Q_MAX [Nm]"
  },
  "defaults": [
   {
    "name": "SIH_T_MAX",
    "value": "5.0",
    "unit": "N",
    "where": "sih_params.yaml:95"
   }
  ],
  "assumptions": "Command interpreted as proportional to rotor speed so thrust ~ u²; hexagon geometry per the ASCII diagram at sih.cpp:468-474.",
  "notes": "Different from the quad branch (linear u). Same SIH_T_MAX default is used."
 },
 {
  "id": "g02-sih-thruster-ct",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "propeller thrust coefficient vs advance ratio C_T(J)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "134-151",
  "code": "return vx_ms / n_rev_s / _d_m;   // advance_ratio\n...\n// do not allow ct < 0 (i.e. windmill)\nreturn fmaxf(CT0 + CT1 * J + CT2 * J * J, 0.0f);",
  "formula": "J = v_x/(n·D) ; C_T(J) = max(CT0 + CT1·J + CT2·J², 0)",
  "variables": {
   "J": "advance ratio",
   "v_x": "air velocity along prop axis [m/s]",
   "n": "rev/s",
   "D": "diameter [m]"
  },
  "defaults": [
   {
    "name": "SIH_F_CT0",
    "value": "0.0",
    "unit": "-",
    "where": "sih_params.yaml:330"
   },
   {
    "name": "SIH_F_CT1",
    "value": "0.0",
    "unit": "-",
    "where": "sih_params.yaml:338"
   },
   {
    "name": "SIH_F_CT2",
    "value": "0.0",
    "unit": "-",
    "where": "sih_params.yaml:345"
   }
  ],
  "assumptions": "Second-order fit of UIUC propeller database; default CT0=0 selects the static model instead (T = T_MAX·u).",
  "notes": "Used by class Thruster for fixed-wing, tailsitter and the standard-VTOL pusher. Not used for the multicopter rotors."
 },
 {
  "id": "g02-sih-thruster-thrust",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "propeller thrust with density scaling",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "173-181, 190-198",
  "code": "float J = advance_ratio(n_rev_s, vx_ms);\nreturn fCT(J) * rho * n_rev_s * n_rev_s * _d_m * _d_m * _d_m * _d_m;\n...\nif (CT0 <= 0.0f) { return T_MAX * u; }\nfloat n_rev_s = throttle_to_rev_s(u, _rpm_max);",
  "formula": "T = C_T(J)·ρ·n²·D⁴ ; static fallback T = T_max·u",
  "variables": {
   "ρ": "air density (argument, default 1.225)",
   "n": "rev/s",
   "D": "prop diameter [m]"
  },
  "defaults": [
   {
    "name": "SIH_F_DIA_INCH",
    "value": "0.1",
    "unit": "inch",
    "where": "sih_params.yaml:374"
   },
   {
    "name": "SIH_F_RPM_MAX",
    "value": "6000",
    "unit": "rpm",
    "where": "sih_params.yaml:381"
   },
   {
    "name": "SIH_F_T_MAX",
    "value": "2.0",
    "unit": "N",
    "where": "sih_params.yaml:309"
   }
  ],
  "assumptions": "rho is not passed by sih.cpp (calls at 490, 499, 509 use the default 1.225), so there is no altitude dependence of thrust in practice.",
  "notes": "Thrust command law: n = rpm_max·√u/60 (aero.hpp:262-280), i.e. thrust ~ u at J=0."
 },
 {
  "id": "g02-sih-thruster-throttle",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "throttle to rpm map",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "248-280",
  "code": "return rpm / 60.0f;\nreturn rpm_max * sqrtf(fminf(fmaxf(u, 0.0f), 1.0f));",
  "formula": "rpm = rpm_max·√(clamp(u,0,1)) ; n = rpm/60",
  "variables": {
   "u": "throttle [0,1]",
   "rpm_max": "SIH_F_RPM_MAX"
  },
  "defaults": [
   {
    "name": "SIH_F_RPM_MAX",
    "value": "6000",
    "unit": "rpm",
    "where": "sih_params.yaml:381"
   }
  ],
  "assumptions": "Static thrust is proportional to throttle (T ~ n² ~ u) by construction.",
  "notes": "Also reused for the ESC rpm report at sih.cpp:433 and by group 16 (command-to-thrust model)."
 },
 {
  "id": "g02-sih-thruster-torque",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "propeller torque from power coefficient C_P(J)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "157-164, 213-215",
  "code": "return fmaxf(CP0 + CP1 * J + CP2 * J * J, 0.0f);\n...\nfloat cq = fCP(J) / 2.0f / M_PI_F;\nreturn cq * rho * n_rev_s * n_rev_s * _d_m * _d_m * _d_m * _d_m * _d_m;",
  "formula": "C_P(J) = max(CP0 + CP1·J + CP2·J², 0) ; C_Q = C_P/(2π) ; Q = C_Q·ρ·n²·D⁵",
  "variables": {
   "Q": "propeller torque [Nm]",
   "C_P": "power coefficient",
   "C_Q": "torque coefficient"
  },
  "defaults": [
   {
    "name": "SIH_F_CP0",
    "value": "0.0",
    "unit": "-",
    "where": "sih_params.yaml:352"
   },
   {
    "name": "SIH_F_Q_MAX",
    "value": "0.0165",
    "unit": "Nm",
    "where": "sih_params.yaml:322"
   }
  ],
  "assumptions": "Static fallback Q = Q_max·u when CP0 <= 0 (aero.hpp:225-229).",
  "notes": "The torque is put on the body X axis for fixed-wing (sih.cpp:493) and on body Z for tailsitter hover (sih.cpp:504)."
 },
 {
  "id": "g02-sih-tmax-derived",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "derived max thrust/torque when dynamic propeller model enabled",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "121-124",
  "code": "T_MAX = 0.0f;\nQ_MAX = 0.0f;\nT_MAX = compute_thrust_from_throttle(1.0f);\nQ_MAX = compute_torque_from_throttle(1.0f);",
  "formula": "T_max = C_T(0)·ρ0·(rpm_max/60)²·D⁴ ; Q_max = C_P(0)/(2π)·ρ0·n_max²·D⁵",
  "variables": {
   "ρ0": "1.225"
  },
  "defaults": [],
  "assumptions": "Evaluated at J=0 (static), ρ=1.225.",
  "notes": "sih.cpp:311-326 then overwrites the SIH_F_T_MAX / SIH_F_Q_MAX parameters with these values."
 },
 {
  "id": "g03-sih-motor-lag",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "first-order motor lag",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "378-384",
  "code": "if ((_vehicle == VehicleType::FixedWing && i < 3) || (_vehicle == VehicleType::TailsitterVTOL && i > 3)) {\n\t_u[i] = actuators_out.output[i];\n} else {\n\tfloat u_sp = actuators_out.output[i];\n\t_u[i] = _u[i] + dt / _T_TAU * (u_sp - _u[i]);",
  "formula": "u ← u + (dt/τ)·(u_sp − u) (forward-Euler first-order lag); control surfaces (fixed-wing i<3, tailsitter i>3) bypass the lag",
  "variables": {
   "u": "actuator state [0,1]",
   "u_sp": "actuator_outputs",
   "τ": "SIH_T_TAU [s]",
   "dt": "loop dt [s]"
  },
  "defaults": [
   {
    "name": "SIH_T_TAU",
    "value": "0.05",
    "unit": "s",
    "where": "sih_params.yaml:263"
   }
  ],
  "assumptions": "Same time constant up and down (no asymmetry); no rate or rpm saturation beyond the [0,1] command; no ESC/battery model.",
  "notes": "Lag is applied to throttle motors only. The comment at sih.cpp:377 says 'saturate' but no clamp is coded here."
 },
 {
  "id": "g03-sih-esc-rpm",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "ESC rpm reported",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "417-421, 433",
  "code": "float max_rpm = 10000.f;\n...\nmax_rpm = _sih_forward_rpm_max.get();\n...\n_esc_status.esc[motor_idx].esc_rpm = (int32_t)roundf(Thruster::throttle_to_rpm(_u[i], max_rpm));",
  "formula": "rpm = rpm_max·√u (rpm_max = 10000 for multirotor, SIH_F_RPM_MAX for fixed-wing/VTOL)",
  "variables": {
   "u": "lagged command"
  },
  "defaults": [
   {
    "name": "max_rpm multirotor",
    "value": "10000",
    "unit": "rpm",
    "where": "sih.cpp:417"
   }
  ],
  "assumptions": "Reporting only; the multicopter thrust model does not use this rpm.",
  "notes": "Inconsistent: for quad, thrust is linear in u while the reported rpm goes as √u (thrust ~ rpm² would be consistent with that)."
 },
 {
  "id": "g04-sih-rotor-aero",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "rotor drag / rolling moment / induced drag",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "457-466",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "Looked in sih.cpp generate_force_and_torques and aero.hpp. Multicopter rotors have no rotor drag, rolling moment or inflow model; only the lumped linear body drag -K_DV·v and damper -K_DW·ω exist (see g05). The only inflow-like term is slipstream momentum theory for wing segments (g06) and the advance ratio in Thruster (g02) for fixed-wing."
 },
 {
  "id": "g05-sih-mr-drag",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "multicopter linear drag and angular damping",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "464-465",
  "code": "_Fa_E = -_KDV * _R_N2E * _v_apparent_N; // first order drag to slow down the aircraft\n_Ma_B = -_KDW * _w_B; // first order angular damper",
  "formula": "F_a = −K_DV·R_N2E·(v_N + v_wind) ; M_a = −K_DW·ω_B",
  "variables": {
   "K_DV": "SIH_KDV [N/(m/s)]",
   "K_DW": "SIH_KDW [Nm/(rad/s)]",
   "v_apparent_N": "ground velocity plus wind, NED"
  },
  "defaults": [
   {
    "name": "SIH_KDV",
    "value": "1.0",
    "unit": "N/(m/s)",
    "where": "sih_params.yaml:154"
   },
   {
    "name": "SIH_KDW",
    "value": "0.025",
    "unit": "Nm/(rad/s)",
    "where": "sih_params.yaml:169"
   }
  ],
  "assumptions": "Isotropic linear drag, same coefficient for all axes and all airspeeds; independent of density, attitude and rotor speed; applied in the Earth frame.",
  "notes": "Same expression for hexacopter (sih.cpp:485-486). Because v_apparent includes wind, wind pushes the vehicle through this drag (sign: see g07)."
 },
 {
  "id": "g05-sih-fw-body-drag",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "fixed-wing / tailsitter / VTOL added linear drag and damping",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "537-542, 567-569",
  "code": "const Vector3f Fa_B = _wing_l.get_Fa() + _wing_r.get_Fa() + _tailplane.get_Fa() + _fin.get_Fa() + _fuselage.get_Fa() -\n\t\t      _KDV * _v_B;\n...\n_Ma_B = _wing_l.get_Ma() + _wing_r.get_Ma() + _tailplane.get_Ma() + _fin.get_Ma() + _fuselage.get_Ma() - _KDW * _w_B;",
  "formula": "F_a,B = Σ F_seg − K_DV·v_B ; M_a,B = Σ M_seg − K_DW·ω_B (tailsitter: R_S2B·ΣF_ts − K_DV·v_B)",
  "variables": {
   "v_B": "air-relative velocity in body frame [m/s]"
  },
  "defaults": [
   {
    "name": "SIH_KDV",
    "value": "1.0",
    "unit": "N/(m/s)",
    "where": "sih_params.yaml:154"
   },
   {
    "name": "SIH_KDW",
    "value": "0.025",
    "unit": "Nm/(rad/s)",
    "where": "sih_params.yaml:169"
   }
  ],
  "assumptions": "Linear extra drag on top of the segment aerodynamics.",
  "notes": "VTOL notes: StandardVTOL calls the same function with the surface commands at u[4..6]."
 },
 {
  "id": "g05-sih-rover-drag",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "rover friction and second-order drag",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "611-635",
  "code": "F_f = _v_E.unit_or_zero() * MU_R * _MASS * 9.81f;\nVector3f v_E_squared = Vector3f(sign(_v_E(0)) * _v_E(0) * _v_E(0), sign(_v_E(1)) * _v_E(1) * _v_E(1),\n\t\t\t\tsign(_v_E(2)) * _v_E(2) * _v_E(2));\nF_a = _KDV * v_E_squared; // [N] Second order drag\n...\n_Fa_E = -F_a - F_f;   // [N] Second order drag and friction",
  "formula": "F_f = μ_r·m·g·v̂ (rolling) ; F_a = K_DV·v|v| (per component, ECEF) ; F_E = −F_a − F_f ; static: F_f = sign(F)·min(|F|, μ_s·m·g); kinetic (|v|<0.05): μ_k·m·g·v̂ or m·v/dt",
  "variables": {
   "μ_s,μ_k,μ_r": "0.5, 0.4, 0.3",
   "m": "SIH_MASS",
   "K_DV": "SIH_KDV used as quadratic coefficient here [N/(m/s)²]"
  },
  "defaults": [
   {
    "name": "MU_S",
    "value": "0.5",
    "unit": "-",
    "where": "sih.cpp:579"
   },
   {
    "name": "MU_K",
    "value": "0.4",
    "unit": "-",
    "where": "sih.cpp:580"
   },
   {
    "name": "MU_R",
    "value": "0.3",
    "unit": "-",
    "where": "sih.cpp:581"
   },
   {
    "name": "ROLLING_THRESHOLD",
    "value": "0.05",
    "unit": "m/s",
    "where": "sih.cpp:582"
   },
   {
    "name": "STATIC_THRESHOLD",
    "value": "0.01",
    "unit": "m/s",
    "where": "sih.cpp:583"
   }
  ],
  "assumptions": "Rover only (VehicleType::RoverAckermann); uses ground velocity _v_E, NOT airspeed, so wind is ignored here; g hard-coded 9.81.",
  "notes": "This is the 'second order drag and friction' line sih.cpp:634. Note SIH_KDV default 1.0 has units N/(m/s) for the quad branch but is reused as N/(m/s)² here. The ordering in the code has a gap: speeds between 0.01 and 0.05 m/s use 'kinetic' friction branch; the static branch is for |v|<0.01. Rover is not a drone."
 },
 {
  "id": "g05-sih-rover-bicycle",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "rover bicycle-model lateral dynamics (tire cornering)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "588-604",
  "code": "const float delta = MAX_STEER_ANGLE * steering_cmd;\nconst float F_x = MAX_THROTTLE_FORCE * throttle_cmd;\nconst float a_y = C * delta / _MASS - fabsf(v_B(0)) * _w_B(2)\n\t\t  - 2 * C * v_B(1) / (_MASS * fabsf(v_B(0)));\nconst float psi_dot_dot = WHEEL_BASE * C * delta / _sih_izz.get()\n\t\t\t  - C * WHEEL_BASE * WHEEL_BASE  * _w_B(2) / (_sih_izz.get() * fabsf(v_B(0)));",
  "formula": "a_y = C·δ/m − |v_x|·r − 2C·v_y/(m·|v_x|) ; r_dot = L·C·δ/I_zz − C·L²·r/(I_zz·|v_x|)",
  "variables": {
   "C": "cornering stiffness 500 N/rad",
   "L": "wheel base 0.321 m",
   "δ": "steering angle = 30°·cmd"
  },
  "defaults": [
   {
    "name": "MAX_THROTTLE_FORCE",
    "value": "400",
    "unit": "N",
    "where": "sih.cpp:575"
   },
   {
    "name": "C",
    "value": "500",
    "unit": "N/rad",
    "where": "sih.cpp:578"
   }
  ],
  "assumptions": "Ground vehicle only, not a drone; kept for completeness.",
  "notes": "Included because scope says scan every physics expression in the file; excluded from course if only air vehicles are shown."
 },
 {
  "id": "g06-sih-segment-force",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "aerodynamic force and moment of one wing/tail/fin/fuselage segment",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "464-490",
  "code": "_v_S = _C_BS.transpose() * (v_B + w_B % _p_B);\nfloat vxz2 = _v_S(0) * _v_S(0) + _v_S(2) * _v_S(2);\n_alpha = matrix::wrap_pi(atan2f(_v_S(2), _v_S(0)) - _alpha_0);\n_Fa = _C_BS * (0.5f * _rho * vxz2 * _span * _mac) * matrix::Vector3f(_CL * sinf(_alpha) - _CD * cosf(_alpha), 0.0f, -_CL * cosf(_alpha) - _CD * sinf(_alpha));\n_Ma = _C_BS * (0.5f * _rho * vxz2 * _span * _mac * _mac) * matrix::Vector3f(0.0f, _CM, 0.0f) + _p_B % _Fa;",
  "formula": "v_S = C_BS^T (v_B + ω×p_B) ; α = atan2(v_Sz, v_Sx) − α0 ; F = ½ρ(v_x²+v_z²)·b·c·C_BS·[C_L sinα − C_D cosα ; 0 ; −C_L cosα − C_D sinα] ; M = ½ρ V² b c² C_M ŷ + p_B × F",
  "variables": {
   "b,c": "segment span, mean chord [m]",
   "α0": "zero-lift AoA",
   "p_B": "segment aerodynamic centre from CoM",
   "C_BS": "segment-to-body rotation (dihedral)"
  },
  "defaults": [],
  "assumptions": "Strip theory in the segment x-z plane (sideslip velocity component v_Sy ignored); aerodynamic quantities computed with local velocity including rotation-induced velocity ω×p; if v_x²+v_z² < 0.01 force and moment set to zero.",
  "notes": "Wind is not a separate input: v_B already includes wind via v_apparent."
 },
 {
  "id": "g06-sih-slipstream",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "propeller slipstream velocity (momentum theory)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "466-472",
  "code": "if (_prop_radius > 1e-4f) {\n\t_v_S(0) += sqrtf(2.0f * thrust / (_rho * M_PI_F * _prop_radius * _prop_radius));\n}",
  "formula": "Δv = √(2T/(ρ·π·R_p²)) added to the segment x-velocity",
  "variables": {
   "T": "thrust [N]",
   "R_p": "prop radius, RP=0.1 m (tail), TS_RP=0.0625 m (tailsitter)"
  },
  "defaults": [
   {
    "name": "RP",
    "value": "0.1",
    "unit": "m",
    "where": "sih.hpp:188"
   },
   {
    "name": "TS_RP",
    "value": "0.0625",
    "unit": "m",
    "where": "sih.hpp:288"
   }
  ],
  "assumptions": "Applied only to tailplane, fin (fixed-wing) and prop-wash segments of the tailsitter; whole segment assumed fully in the slipstream.",
  "notes": "Uses the static induced-velocity expression Δv = √(2T/ρA) (not the far-wake 2·v_i). StandardVTOL passes thrust 0 (sih.cpp:517)."
 },
 {
  "id": "g06-sih-lift-coeffs",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "low angle-of-attack lift / drag / moment coefficients (flat plate with partial stall)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "414, 439, 566-570, 650-661",
  "code": "_kp = cl_alpha / (1.0f + 2.0f * (_ar + 4.0f) / (_ar * (_ar + 2.0f)));\n_CL_ = fCL(_alpha_eff);\n_CD_ = CD0 + fabsf(_CL * tanf(_alpha_eff));\n_CM_ = -fCM(_alpha_eff);\nreturn 0.25f * (1.0f + sqrtf(_fte)) * (1.0f + sqrtf(_fte)) * (_kp * sinf(a) * cosf(a) * cosf(a) + _fle * _fle * KV * fabsf(sinf(a)) * sinf(a) * cosf(a));",
  "formula": "K_p = C_Lα/(1 + 2(AR+4)/(AR(AR+2))) ; C_L = ¼(1+√f_te)²·(K_p sinα cos²α + f_le²·K_v |sinα| sinα cosα) ; C_D = C_D0 + |C_L tanα| ; C_M = −fCM(α) with fCM = −¼(1+√f_te)²·(1/16)(−1+6√f_te−5f_te)K_p sinα cosα + 0.17 f_le² K_v |sinα| sinα",
  "variables": {
   "AR": "aspect ratio",
   "C_Lα": "2D lift slope, default 2π/rad",
   "K_v": "π (total vortex lift parameter KV)",
   "f_te,f_le": "set to 1 (separation functions commented out)"
  },
  "defaults": [
   {
    "name": "CD0",
    "value": "0.04",
    "unit": "-",
    "where": "aero.hpp:329"
   },
   {
    "name": "KV",
    "value": "π",
    "unit": "-",
    "where": "aero.hpp:328"
   }
  ],
  "assumptions": "From Khan (McGill PhD, 2016) sec 3.1-3.3; f_te = f_le = 1 are hard-coded so the dynamic-stall delay model is disabled ('this doesn't seem to work'); then C_L reduces to ¼·4·(...)= K_p sinα cos²α + K_v|sinα| sinα cosα; the alternative parabolic drag _kD·C_L² is commented out.",
  "notes": "Oswald K0=0.87 is used only by the unused _kD = 1/(π K0 AR) (aero.hpp:351, 439)."
 },
 {
  "id": "g06-sih-highaoa",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "post-stall flat-plate normal/tangential force model",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "586-599",
  "code": "float mac_eff = sqrtf((_mac - _cf) * (_mac - _cf) + _cf * _cf + 2.0f * (_mac - _cf) * _cf * cosf(fabsf(def)));\na += asinf(_cf / mac_eff * sinf(def));\nfloat cd90_eff = CD90 + 0.21f * def - 0.0426f * def * def;\nfloat CN = cd90_eff * sinf(a) * (1.0f / (0.56f + 0.44f * sinf(fabsf(a))) - _kn);\nfloat CT = 0.5f * CD0 * cosf(a);\n_CL = CN * cosf(a) - CT * sinf(a);\n_CD = CN * sinf(a) + CT * cosf(a);\n_CM = -CN * (0.25f - 7.0f / 40.0f * (1.0f - 2.0f / M_PI_F * fabsf(a)));",
  "formula": "C_N = C_D90·sin a·(1/(0.56 + 0.44|sin a|) − k_n) ; C_T = ½C_D0 cos a ; C_L = C_N cos a − C_T sin a ; C_D = C_N sin a + C_T cos a ; C_M = −C_N(0.25 − (7/40)(1 − 2|a|/π)) ; C_D90 = 1.98 + 0.21δ − 0.0426δ² ; k_n = 0.41(1 − e^(−17/AR))",
  "variables": {
   "δ": "flap deflection [rad]",
   "a": "effective AoA incl. flap geometry [rad]"
  },
  "defaults": [
   {
    "name": "CD90",
    "value": "1.98",
    "unit": "-",
    "where": "aero.hpp:330"
   }
  ],
  "assumptions": "Flat plate model for the full 360 deg range; the flap-deflection fit for C_D90 'might not be accurate' per its own comment (aero.hpp:591).",
  "notes": "k_n at aero.hpp:415."
 },
 {
  "id": "g06-sih-blend",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "low-to-high AoA blending function and stall angles",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "573-582",
  "code": "if (_alpha_eff > 0.0f) {\n\t_f_blend = 0.5f * (1.0f - tanhf(4.0f * (_alpha_eff - _alpha_eff_max) / ALPHA_BLEND));\n} else {\n\t_f_blend = 0.5f * (1.0f - tanhf(-4.0f * (_alpha_eff - _alpha_eff_min) / ALPHA_BLEND));\n}\n_CL = _CL_ * _f_blend + _CL * (1.0f - _f_blend);",
  "formula": "f = ½(1 − tanh(4(α_eff − α_max)/Δ)) (α>0) ; C = f·C_low + (1−f)·C_high ; Δ = 10°",
  "variables": {
   "α_max,α_min": "stall AoA from table afs_tab(AR) (20..49 deg) or constructor"
  },
  "defaults": [
   {
    "name": "ALPHA_BLEND",
    "value": "π/18",
    "unit": "rad",
    "where": "aero.hpp:332"
   }
  ],
  "assumptions": "Same blend applies to C_L, C_D, C_M.",
  "notes": "Stall angle lookup tables at aero.hpp:400-405 (afs_tab etc.), linear interpolation in AR (aero.hpp:602-634)."
 },
 {
  "id": "g06-sih-flap",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "control surface (flap) effectiveness on lift",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/aero.hpp",
  "lines": "523-531",
  "code": "_def_a = fminf(fabsf(def), math::radians(70.0f));\n_eta_f = _def_a * _def_a * ETA_POLY[0] + _def_a * ETA_POLY[1] + ETA_POLY[2];\n_theta_f = acosf(2.0f * _cf / _mac - 1.0f);\n_tau_f = 1.0f - (_theta_f - sinf(_theta_f)) / M_PI_F;\n_deltaCL = _kp * _tau_f * _eta_f * def;\n_dCLmax = (1.0f - _cf / _mac) * _deltaCL;\n_alf0eff = solve_alpha_eff(_kp, KV, _deltaCL, _alpha_0);",
  "formula": "η = 0.0535δ² − 0.2688δ + 0.5817 ; θ_f = acos(2c_f/c − 1) ; τ = 1 − (θ_f − sinθ_f)/π ; ΔC_L = K_p·τ·η·δ ; ΔC_Lmax = (1 − c_f/c)ΔC_L ; solve for α0,eff by 3 Newton iterations on eq. 3.15 (aero.hpp:636-648)",
  "variables": {
   "δ": "flap deflection [rad], max 15° for fixed-wing (FLAP_MAX=π/12), 39° tailsitter (TS_DEF_MAX)",
   "c_f": "flap chord [m]"
  },
  "defaults": [
   {
    "name": "FLAP_MAX",
    "value": "π/12",
    "unit": "rad",
    "where": "sih.hpp:189"
   },
   {
    "name": "TS_DEF_MAX",
    "value": "radians(39)",
    "unit": "rad",
    "where": "sih.hpp:289"
   }
  ],
  "assumptions": "Full-flap branch (c_f/c >= 0.999) uses α_eff = α + δ (aero.hpp:549-563); flap model from Khan thesis sec 3.2.3.",
  "notes": "Flap deflection commands: wing left +roll·FLAP_MAX, right −roll·FLAP_MAX, tailplane −pitch·FLAP_MAX, fin yaw·FLAP_MAX (sih.cpp:529-533)."
 },
 {
  "id": "g06-sih-fw-geometry",
  "group": 6,
  "group_name": "wings and fixed-wing aerodynamics",
  "quantity": "fixed-wing segment layout and VTOL tailsitter segments",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.hpp",
  "lines": "276-282, 285-304",
  "code": "AeroSeg _wing_l = AeroSeg(SPAN / 2.0f, MAC, -4.0f, matrix::Vector3f(0.0f, -SPAN / 4.0f, 0.0f), 3.0f, ...\nAeroSeg _tailplane = AeroSeg(0.3f, 0.1f, 0.0f, matrix::Vector3f(-0.4f, 0.0f, 0.0f), 0.0f, -1.0f, 0.05f, RP);\nAeroSeg _fin = AeroSeg(0.25, 0.18, 0.0f, matrix::Vector3f(-0.45f, 0.0f, -0.1f), -90.0f, -1.0f, 0.12f, RP);\nAeroSeg _fuselage = AeroSeg(0.2, 0.8, 0.0f, matrix::Vector3f(0.0f, 0.0f, 0.0f), -90.0f);",
  "formula": "geometry data (no formula): wing span 0.86 m, MAC 0.21 m, 11 tailsitter segments (TS_AR=3.13, CM 0.115 m behind TE)",
  "variables": {
   "SPAN": "0.86 m",
   "MAC": "0.21 m"
  },
  "defaults": [
   {
    "name": "SPAN",
    "value": "0.86",
    "unit": "m",
    "where": "sih.hpp:186"
   },
   {
    "name": "MAC",
    "value": "0.21",
    "unit": "m",
    "where": "sih.hpp:187"
   }
  ],
  "assumptions": "A small hard-coded foam flying wing; wing AoA0 −4°, dihedral ±3°. Tailsitter aerodynamics are resolved in the segment frame rotated 90° about pitch (R_S2B, sih.hpp:290).",
  "notes": "VTOL: tailsitter and standard VTOL use these geometries; VTOL tagged fixed-wing. Tailsitter airspeed is taken along −z (sih.cpp:784)."
 },
 {
  "id": "g07-sih-wind",
  "group": 7,
  "group_name": "wind and gusts",
  "quantity": "wind velocity and apparent (air-relative) velocity; constant wind only, no gusts",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "367, 725, 455",
  "code": "_v_wind_N = Vector3f(_sih_wind_n.get(), _sih_wind_e.get(), 0.f);\n_v_apparent_N = _v_N + _v_wind_N;\n_v_B = _q_E.rotateVectorInverse(_R_N2E * _v_apparent_N);",
  "formula": "v_apparent = v_N + v_wind ; v_wind = (SIH_WIND_N, SIH_WIND_E, 0)",
  "variables": {
   "v_N": "vehicle ground velocity, NED [m/s]",
   "v_wind": "(SIH_WIND_N, SIH_WIND_E, 0) [m/s]",
   "v_apparent": "vehicle velocity relative to the air, NED [m/s]"
  },
  "defaults": [
   {
    "name": "SIH_WIND_N",
    "value": "0.0",
    "unit": "m/s",
    "where": "sih_params.yaml:282"
   },
   {
    "name": "SIH_WIND_E",
    "value": "0.0",
    "unit": "m/s",
    "where": "sih_params.yaml:288"
   }
  ],
  "assumptions": "SIGN CONVENTION: parameter text is 'Wind velocity from north direction' (sih_params.yaml:280). The code ADDS the parameter to the ground velocity. For a hovering vehicle (v_N=0) the air-relative velocity is +SIH_WIND_N along north, so the air moves toward SOUTH: positive SIH_WIND_N is a wind blowing FROM the north (the meteorological 'from' convention), i.e. SIH_WIND = minus the velocity vector of the air mass in NED. Equivalent: v_air,NED = −(SIH_WIND_N, SIH_WIND_E, 0). Vertical wind is fixed to 0.",
  "notes": "Constant wind, no gust/turbulence model, no altitude profile. The apparent velocity feeds the multirotor drag (g05), all wing segments (g06) and the airspeed output (g07-sih-airspeed). Rover dynamics use ground velocity and ignore wind (sih.cpp:585, 614-632)."
 },
 {
  "id": "g07-sih-airspeed",
  "group": 7,
  "group_name": "wind and gusts",
  "quantity": "airspeed along pitot axis and indicated airspeed",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "783-786",
  "code": "const Vector3f v_apparent_B = _q.rotateVectorInverse(_v_apparent_N);\nconst float v_pitot = (_vehicle == VehicleType::TailsitterVTOL) ? -v_apparent_B(2) : v_apparent_B(0);\nairspeed.true_airspeed_m_s = fmaxf(0.1f, v_pitot + generate_wgn() * 0.2f);\nairspeed.indicated_airspeed_m_s = airspeed.true_airspeed_m_s * sqrtf(_wing_l.get_rho() / RHO);",
  "formula": "TAS = max(0.1, v_pitot + 0.2·N(0,1)) ; IAS = TAS·√(ρ/ρ0)",
  "variables": {
   "v_pitot": "apparent velocity along body x (tailsitter: −z)",
   "ρ0": "1.225"
  },
  "defaults": [
   {
    "name": "RHO",
    "value": "1.225",
    "unit": "kg/m^3",
    "where": "sih.hpp:185"
   }
  ],
  "assumptions": "Noise std 0.2 m/s; no differential pressure sensor model here (comment: 'TODO: send differential pressure instead?'); published at 20 Hz (50 ms).",
  "notes": "The apparent velocity uses the attitude _q (NED) while the aerodynamic body velocity uses _q_E; equivalent. Wind therefore enters airspeed with the SAME sign as in g07-sih-wind. Sensor row, see also g14."
 },
 {
  "id": "g08-sih-translation",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "translational equations of motion in ECEF with Coriolis and transport rate",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "641-646, 686-694",
  "code": "const Vector3f gravity_acceleration_E = Vector3f(_R_N2E.col(2)) * LatLonAlt::Wgs84::gravity(_lla.latitude_rad());\nconst Vector3f coriolis_acceleration_E = -2.f * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE).cross(_v_E);\nVector3f sum_of_forces_E = _Fa_E + _q_E.rotateVector(_T_B) + weight_E;\nconst Vector3f acceleration_E = sum_of_forces_E / _MASS;\n_v_E_dot = acceleration_E + coriolis_acceleration_E;\n_v_N_dot = _R_N2E.transpose() * _v_E_dot + transport_rate;",
  "formula": "a_E = (F_aero + R_E,B·T_B + m·g)/m ; v̇_E = a_E − 2Ω_E×v_E ; v̇_N = R_E→N v̇_E − ω_en × v_N",
  "variables": {
   "m": "SIH_MASS [kg]",
   "Ω_E": "Earth spin rate along ECEF z",
   "ω_en": "nav-frame transport rate (lat_lon_alt.cpp:83-92)"
  },
  "defaults": [
   {
    "name": "SIH_MASS",
    "value": "1.0",
    "unit": "kg",
    "where": "sih_params.yaml:10"
   }
  ],
  "assumptions": "Centrifugal term is not added separately (gravity() is the 'normal gravity' which already includes it); specific force for the accelerometer = a_E − g (sih.cpp:687).",
  "notes": "Frames: body FRD, thrust T_B along −z, forces rotated to ECEF with _q_E. Earth rate constant CONSTANTS_EARTH_SPIN_RATE (value not read from this repo section)."
 },
 {
  "id": "g09-sih-rotation",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Euler's equation and quaternion integration",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "702-708",
  "code": "const Quatf dq(AxisAnglef(_w_B * dt));\n_q_E = _q_E  * dq;\n_q_E.normalize();\nconst Vector3f w_B_dot = _Im1 * (_Mt_B + _Ma_B - _w_B.cross(_I * _w_B));\n_w_B = constrain(_w_B + w_B_dot * dt, -6.0f * M_PI_F, 6.0f * M_PI_F);",
  "formula": "I·ω̇ = M_thrust + M_aero − ω×(Iω) ; ω ← clamp(ω + ω̇·dt, ±6π) ; q ← q ⊗ exp(ω·dt) normalized",
  "variables": {
   "I": "inertia matrix (full 3x3 with products)",
   "ω": "body angular rate [rad/s]",
   "M": "moments [Nm]"
  },
  "defaults": [
   {
    "name": "SIH_IXX",
    "value": "0.025",
    "unit": "kg m^2",
    "where": "sih_params.yaml:22"
   },
   {
    "name": "SIH_IYY",
    "value": "0.025",
    "unit": "kg m^2",
    "where": "sih_params.yaml:34"
   },
   {
    "name": "SIH_IZZ",
    "value": "0.03",
    "unit": "kg m^2",
    "where": "sih_params.yaml:46"
   }
  ],
  "assumptions": "Angular rate clamped to ±6π rad/s (3 rev/s) per axis (a numerical-safety limit). Inertia inverse is computed with a scaling guard: _Im1 = 100·inv(100·I) (sih.cpp:357). Rotor gyroscopic/spin angular momentum not modeled. Quaternion update uses the old ω (rotation applied before the rate update).",
  "notes": "Rotation of ECEF-frame quaternion; attitude in NED is q = q_N2E-part ⊗ q_E (sih.cpp:727). Earth rotation seen by gyro added at 755-756."
 },
 {
  "id": "g10-sih-integration",
  "group": 10,
  "group_name": "time integration scheme",
  "quantity": "integrator and time step",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "195-203, 251, 696-700",
  "code": "int interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);\nconst float dt = (now - _last_run) * 1e-6f;\n_v_E = _v_E + _v_E_dot * dt;\n_p_E = _p_E + Vector3d(_v_E + v_E_prev) * 0.5 * static_cast<double>(dt);",
  "formula": "v ← v + v̇·dt (forward Euler) ; p ← p + ½(v_new + v_old)·dt (trapezoidal) ; q ← q⊗exp(ω dt) ; ω ← ω + ω̇ dt",
  "variables": {
   "dt": "measured wall/lockstep time since last step [s]"
  },
  "defaults": [
   {
    "name": "default rate",
    "value": "250",
    "unit": "Hz",
    "where": "sih.cpp:198"
   }
  ],
  "assumptions": "Variable dt (measured); interval clamped to 0.5..5 ms (200..2000 Hz); lockstep mode min of IMU gyro max rate and integration rate (sih.cpp:105). Module description says 'Forward Euler is used for integration' (sih.cpp:1093); position is actually trapezoidal.",
  "notes": "Mixed scheme: semi-implicit-like because the position uses the new velocity (average of old and new)."
 },
 {
  "id": "g11-sih-ground",
  "group": 11,
  "group_name": "ground contact and collision",
  "quantity": "fake ground plane (inelastic stop)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "649-685",
  "code": "if ((_lla.altitude() - _lpos_ref_alt) < 0.f && force_down > 0.f) {\n\tground_force_E = -sum_of_forces_E;\n\tif (!_grounded) { ground_force_E += -_v_E / dt * _MASS; }\n...\nground_force_E = -down_u * sum_of_forces_E * down_u;\nground_force_E += down_u * (-_v_N(2) / dt) * _MASS;",
  "formula": "multirotor/VTOL: F_ground = −ΣF (− m·v/dt at impact) ; fixed-wing/rover: only the down component is cancelled (−(ΣF·d)d, − m·v_down/dt at impact)",
  "variables": {
   "d": "local Down unit vector in ECEF",
   "_grounded": "state flag"
  },
  "defaults": [],
  "assumptions": "Perfectly inelastic, frictionless for fixed-wing (horizontal motion kept), fully pinned (all forces zero) for multirotor; ground at altitude _lpos_ref_alt (SIH_LOC_H0); no restitution.",
  "notes": "Condition: below reference altitude AND net force pointing down. Quote condensed from several code lines."
 },
 {
  "id": "g12-sih-mixer",
  "group": 12,
  "group_name": "control allocation and motor mixing",
  "quantity": "fixed quad/hexa/VTOL mixing built into the dynamics",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "459-462, 481-484, 503-504, 511-514",
  "code": "_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]),\n\t\t _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),\n\t\t _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));",
  "formula": "[M_x; M_y; M_z; T] = B·[u0..u3], B rows: L_r T_max(−1,1,1,−1), L_p T_max(1,−1,1,−1), Q_max(1,1,−1,−1), −T_max(1,1,1,1) — physical (forward) effectiveness matrix hard-coded in the simulator; tailsitter: M_x = L_roll(T1−T0), M_z = Q1−Q0, T = −T0−T1",
  "variables": {
   "u_i": "motor commands"
  },
  "defaults": [],
  "assumptions": "This is the plant side; the controller side mixer is PX4 control_allocator (see PX4 autopilot rows). Motor order/spin directions must match the airframe's CA_ROTOR*; sign of yaw (Q) per motor: motors 0,1 positive, 2,3 negative.",
  "notes": "StandardVTOL = quad rows + pusher (sih.cpp:511-514), pusher torque Q[0] added to M_x."
 },
 {
  "id": "g13-sih-battery",
  "group": 13,
  "group_name": "battery",
  "quantity": "battery model",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "1-1108",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "sih.cpp/sih.hpp have no battery model (only BAT1_SOURCE parameter listed in sih.hpp:354). Battery is simulated by the separate module battery_simulator, see 'PX4 autopilot' battery rows. Thrust does not depend on battery voltage in SIH."
 },
 {
  "id": "g14-sih-imu",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "IMU accelerometer and gyro (specific force, Earth rate, white noise)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "746-756",
  "code": "accel_noise = noiseGauss3f(0.1f, 0.1f, 0.1f);\ngyro_noise = noiseGauss3f(0.01f, 0.01f, 0.01f);\nVector3f specific_force_B = R_E2B * _specific_force_E;\nVector3f accel = specific_force_B + accel_noise;\nconst Vector3f earth_spin_rate_B = R_E2B * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE);\nVector3f gyro = _w_B + earth_spin_rate_B + gyro_noise;",
  "formula": "a_meas = f_B + n_a, n_a ~ N(0, 0.1²) [m/s²] per axis ; ω_meas = ω_B + R_E→B Ω_E + n_g, n_g ~ N(0, 0.01²) [rad/s]",
  "variables": {
   "f_B": "specific force = (a_E − g) rotated into body",
   "Ω_E": "Earth rotation"
  },
  "defaults": [
   {
    "name": "accel noise std",
    "value": "0.1",
    "unit": "m/s^2",
    "where": "sih.cpp:748"
   },
   {
    "name": "gyro noise std",
    "value": "0.01",
    "unit": "rad/s",
    "where": "sih.cpp:749"
   }
  ],
  "assumptions": "NO bias, NO bias random walk, NO scale factor or misalignment; the armed noise branch (0.5,1.7,1.4 and 0.14,0.07,0.03) is disabled by 'if (false && ...)' (sih.cpp:742-744); noise generated at every sim step (white, so noise density depends on the step rate).",
  "notes": "Noise levels cited from Bulka & Nahon ICRA 2018 (sih.cpp:733-735). Sensor temperature fixed to 15 C (T1_C)."
 },
 {
  "id": "g14-sih-wgn",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "Gaussian noise generator (polar Box-Muller)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "951-958",
  "code": "V1 = 2.0f * U1 - 1.0f;\nV2 = 2.0f * U2 - 1.0f;\nS = V1 * V1 + V2 * V2;\nX = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));",
  "formula": "X = V1·√(−2 ln S / S), S = V1² + V2² < 1 (Marsaglia polar method, unit-variance)",
  "variables": {
   "U": "uniform rand()/RAND_MAX"
  },
  "defaults": [
   {
    "name": "seed",
    "value": "1234",
    "unit": "-",
    "where": "sih.cpp:61"
   }
  ],
  "assumptions": "Fixed seed => deterministic noise sequence.",
  "notes": ""
 },
 {
  "id": "g14-sih-distance",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "distance sensor (ray to flat ground)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "800-810",
  "code": "current_distance = -_lpos(2) / _q.dcm_z()(2);\nif (current_distance > _distance_snsr_max) {\n\tcurrent_distance = UINT16_MAX / 100.f;\n}",
  "formula": "d = h / (R_33) where h = height above ground and R_33 = cos of tilt (z-z element of the DCM) ; beyond max range -> 655.35 m",
  "variables": {
   "h": "-_lpos(2) [m]"
  },
  "defaults": [
   {
    "name": "SIH_DISTSNSR_MAX",
    "value": "100.0",
    "unit": "m",
    "where": "sih_params.yaml:244"
   }
  ],
  "assumptions": "Flat ground; no noise; published at 50 Hz; override parameter SIH_DISTSNSR_OVR.",
  "notes": "Based on Lightware LW20 behaviour per source comment."
 },
 {
  "id": "g14-sih-beacon",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "ranging beacon range with Gaussian noise",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "834-839",
  "code": "const matrix::Vector3d delta_ecef = beacon_ecef - _p_E;\nconst double true_range_m = delta_ecef.norm();\nconst float noise_m = (noise_std > 0.f) ? generate_wgn() * noise_std : 0.f;\nconst double measured_range_m = math::max(0.0, true_range_m + static_cast<double>(noise_m));",
  "formula": "r = max(0, ‖p_beacon − p‖ + σ·N(0,1))",
  "variables": {
   "σ": "SIH_RNGBC_NOISE [m]"
  },
  "defaults": [
   {
    "name": "SIH_RNGBC_NOISE",
    "value": "30.0",
    "unit": "m",
    "where": "sih_params.yaml:295"
   }
  ],
  "assumptions": "Straight-line range in ECEF; beacon positions offsets 10-50 km (sih.hpp:173-177).",
  "notes": ""
 },
 {
  "id": "g15-sih-notapplicable",
  "group": 15,
  "group_name": "PX4 estimators using physics models",
  "quantity": "estimators",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "1-1108",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "SIH contains no estimator code; estimators are in the PX4 autopilot rows."
 },
 {
  "id": "g16-sih-thrust-model",
  "group": 16,
  "group_name": "thrust curve (command to thrust)",
  "quantity": "command to thrust in SIH",
  "simulator": "PX4 SIH",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_sih/sih.cpp",
  "lines": "459, 481",
  "code": "_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));\n...\nu_sq[i] = _u[i] * fabsf(_u[i]);",
  "formula": "quad: T = T_max·u (linear) ; hexa: T = T_max·u|u| ; fixed-wing: T = C_T(J)ρn²D⁴ with n = n_max√u",
  "variables": {},
  "defaults": [],
  "assumptions": "SIH does not use THR_MDL_FAC: the SIH plant has its own fixed command-to-thrust law, so the controller's THR_MDL_FAC linearisation (when non-zero) is NOT matched to the quad plant (u is linear thrust there, hexa is quadratic).",
  "notes": "THR_MDL_FAC lives in the control allocator (see PX4 autopilot rows)."
 },
 {
  "id": "g01-sihsim-baro-isa",
  "group": 1,
  "group_name": "atmosphere and gravity",
  "quantity": "barometer pressure vs altitude (simulated baro, 288 K lapse model)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_baro_sim/SensorBaroSim.cpp",
  "lines": "146-152",
  "code": "const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)\nconst float temperature_msl = 288.0f; // temperature at MSL (Kelvin)\nconst float temperature_local = temperature_msl - lapse_rate * alt_msl;\nconst float pressure_ratio = powf(temperature_msl / temperature_local, 5.256f);\nconst float pressure_msl = 101325.0f; // pressure at MSL\nconst float absolute_pressure = pressure_msl / pressure_ratio;",
  "formula": "T = 288 − 0.0065·h ; p = 101325 / (288/T)^5.256",
  "variables": {
   "h": "gpos.alt, AMSL ground-truth altitude [m]",
   "T": "local temperature [K]"
  },
  "defaults": [
   {
    "name": "temperature_msl",
    "value": "288.0",
    "unit": "K",
    "where": "SensorBaroSim.cpp:147"
   }
  ],
  "assumptions": "Troposphere ISA with T0 = 288 K (not 288.15) and exponent 5.256 (vs 5.2561 in aero.hpp); the report temperature is T − 273.0 (not 273.15, line 163).",
  "notes": "Inconsistent constants within PX4: SIH aero.hpp uses 288.15 K and 5.2561; atmosphere.cpp uses 288.15 K and exponent g/(R·0.0065). Baro runs at 20 Hz (ScheduleOnInterval 50 ms)."
 },
 {
  "id": "g14-sihsim-baro-noise",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "barometer noise, drift and offsets",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_baro_sim/SensorBaroSim.cpp",
  "lines": "155-163",
  "code": "const float abs_pressure_noise = 1.f * generate_wgn();  // 1 Pa RMS noise\n_baro_drift_pa += _baro_drift_pa_per_sec * dt;\nconst float absolute_pressure_noisy = absolute_pressure + abs_pressure_noise + _baro_drift_pa;\nfloat pressure = absolute_pressure_noisy + _sim_baro_off_p.get();\nfloat temperature = temperature_local - 273.0f + _sim_baro_off_t.get();",
  "formula": "p_meas = p(h) + N(0, 1 Pa²) + drift·t + SIM_BARO_OFF_P",
  "variables": {
   "drift": "_baro_drift_pa_per_sec [Pa/s]"
  },
  "defaults": [
   {
    "name": "SIM_BARO_OFF_P",
    "value": "0.0",
    "unit": "Pa",
    "where": "sensor_baro_sim/parameters.yaml:22"
   },
   {
    "name": "_baro_drift_pa_per_sec",
    "value": "0.0",
    "unit": "Pa/s",
    "where": "SensorBaroSim.hpp:85"
   }
  ],
  "assumptions": "White noise std 1 Pa per 20 Hz sample; drift rate default 0 (only changed by failure injection); no temperature dependence of noise; uses vehicle_global_position_groundtruth.",
  "notes": ""
 },
 {
  "id": "g14-sihsim-mag",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "magnetometer (WMM field rotated to body + white noise + offsets)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_mag_sim/SensorMagSim.cpp",
  "lines": "135-155",
  "code": "_mag_earth_pred = Dcmf(Eulerf(0, -inclination_rad, declination_rad)) * Vector3f(field_strength_gauss, 0, 0);\nVector3f expected_field = Dcmf{Quatf{attitude.q}} .transpose() * _mag_earth_pred;\nexpected_field += noiseGauss3f(0.02f, 0.02f, 0.03f);\nexpected_field += Vector3f(_sim_mag_offset_x.get(), _sim_mag_offset_y.get(), _sim_mag_offset_z.get());",
  "formula": "B_earth = R(0, −I, D)·[F,0,0] ; B_body = R_nb^T·B_earth + N(0, diag(0.02²,0.02²,0.03²)) + offset [gauss]",
  "variables": {
   "D,I,F": "declination, inclination, strength from the World Magnetic Model tables at the ground-truth lat/lon [rad, rad, gauss]"
  },
  "defaults": [
   {
    "name": "noise std",
    "value": "0.02, 0.02, 0.03",
    "unit": "gauss",
    "where": "SensorMagSim.cpp:152"
   },
   {
    "name": "SIM_MAG_OFFSET_X/Y/Z",
    "value": "0.0",
    "unit": "gauss",
    "where": "sensor_mag_sim/parameters.yaml:22-34"
   }
  ],
  "assumptions": "Field depends on lat/lon only (not altitude); noise is white with no bias walk; sampled at 50 Hz using the ground-truth attitude.",
  "notes": "Field strength, declination and inclination come from lookup tables (src/lib/world_magnetic_model, geo_magnetic_tables.hpp), not read formula by formula here: only the call sites."
 },
 {
  "id": "g14-sihsim-gps",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "GNSS position and velocity noise (first-order Gauss-Markov)",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp",
  "lines": "123-145",
  "code": "_gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n + _pos_random_walk * generate_wgn() * _pos_noise_amplitude;\nconst double latitude = gpos.lat + math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);\n_gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n + _vel_noise_density * generate_wgn() * _vel_noise_amplitude;\nconst Vector3f gps_vel = Vector3f{lpos.vx + _gps_vel_noise_n, lpos.vy + _gps_vel_noise_e, lpos.vz + _gps_vel_noise_d};",
  "formula": "n_{k+1} = a·n_k + b·w·A  (pos: a=0.76, b=0.02, A=0.8 m; down axis ×1.5 ; vel: a=0.54, b=0.4, A=0.05 m/s; down axis ×1.2) ; lat += n_N/R_E, lon += n_E/R_E (no cosφ in the sim noise path) ; v = v_true + n_v",
  "variables": {
   "R_E": "6371000 m (CONSTANTS_RADIUS_OF_EARTH)",
   "w": "N(0,1)"
  },
  "defaults": [
   {
    "name": "_pos_noise_amplitude",
    "value": "0.8",
    "unit": "m",
    "where": "SensorGpsSim.hpp:119"
   },
   {
    "name": "_pos_random_walk",
    "value": "0.02",
    "unit": "-",
    "where": "SensorGpsSim.hpp:120"
   },
   {
    "name": "_pos_markov_time",
    "value": "0.76",
    "unit": "-",
    "where": "SensorGpsSim.hpp:121"
   },
   {
    "name": "_vel_noise_amplitude",
    "value": "0.05",
    "unit": "m/s",
    "where": "SensorGpsSim.hpp:122"
   },
   {
    "name": "_vel_noise_density",
    "value": "0.4",
    "unit": "-",
    "where": "SensorGpsSim.hpp:123"
   },
   {
    "name": "_vel_markov_time",
    "value": "0.54",
    "unit": "-",
    "where": "SensorGpsSim.hpp:124"
   }
  ],
  "assumptions": "The 'markov_time' constants are used directly as per-step decay factors (dimensionless), so the correlation time depends on the update rate; the longitude conversion omits 1/cos(latitude) for the noise (only the antenna-offset path at lines 210-211 has it).",
  "notes": "Code comment says it matches the gz-bridge model. Reported ground speed sqrt(vN²+vE²), course atan2(vE,vN) (lines 186-191). eph/epv fixed 0.9/1.78 m (3D) or 0.02/0.04 m (RTK)."
 },
 {
  "id": "g14-sihsim-airspeed-sensor",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "airspeed sensor differential pressure (simulated pitot)",
  "simulator": "PX4 SIH",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_airspeed_sim/SensorAirspeedSim.cpp",
  "lines": "127-159",
  "code": "Vector3f body_velocity = Dcmf{Quatf{attitude.q}} .transpose() * local_velocity;\nconst float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;\nconst float density_ratio = powf(TEMPERATURE_MSL / temperature_local, 4.256f);\nconst float air_density = AIR_DENSITY_MSL / density_ratio;\nconst float diff_pressure_noise = (float)generate_wgn() * 0.01f;\nfloat diff_pressure = sign(body_velocity(0)) * 0.005f * air_density  * body_velocity(0) * body_velocity(0) + diff_pressure_noise;",
  "formula": "ρ = 1.225·(T/288.15)^4.256 ; Δp[hPa] = sign(v_x)·0.005·ρ·v_x² + N(0,0.01²) ; Δp[Pa] = 100·Δp[hPa] ⇒ Δp = ½ρv_x² (+1 Pa noise)",
  "variables": {
   "v_x": "ground-truth NED velocity rotated to body x (forward) [m/s]",
   "ρ": "air density [kg/m³]"
  },
  "defaults": [
   {
    "name": "TEMPERATURE_MSL",
    "value": "288.15",
    "unit": "K",
    "where": "SensorAirspeedSim.hpp:58"
   },
   {
    "name": "LAPSE_RATE",
    "value": "0.0065",
    "unit": "K/m",
    "where": "SensorAirspeedSim.hpp:60"
   },
   {
    "name": "AIR_DENSITY_MSL",
    "value": "1.225",
    "unit": "kg/m^3",
    "where": "SensorAirspeedSim.hpp:61"
   }
  ],
  "assumptions": "Uses vehicle_local_position_groundtruth (GROUND velocity) and the estimator's vehicle_attitude: wind is NOT subtracted, so this simulated pitot reads ground speed along body x; dynamic pressure incompressible ½ρv²; 0.5·... factor equals 0.005·100.",
  "notes": "Contrast: sih.cpp send_airspeed (g07-sih-airspeed) includes wind. Density exponent 4.256 = g/(R L) − 1 (ρ ratio), pressure exponent 5.256 elsewhere. Pitot blockage ramps differential pressure down to 30% over 30 s (lines 149-157)."
 },
 {
  "id": "g14-sihsim-agp",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "auxiliary global position (AGP) noise and drift",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/sensor_agp_sim/SensorAgpSim.cpp",
  "lines": "120-132",
  "code": "_position_bias += Vector3f(1.5f, -5.f, 0.f) * dt;\nconst double latitude = _measured_lla.latitude_deg() + math::degrees((double)generate_wgn() * 2.0 / CONSTANTS_RADIUS_OF_EARTH);\nconst double altitude = (double)(_measured_lla.altitude() + (generate_wgn() * 0.5f));",
  "formula": "lat,lon += N(0, 2 m)/R_E ; alt += N(0, 0.5 m) ; drift failure mode: bias += (1.5, −5, 0) m/s·dt",
  "variables": {},
  "defaults": [],
  "assumptions": "White noise std 2 m horizontal, 0.5 m vertical; drift only in failure mode.",
  "notes": ""
 },
 {
  "id": "g13-sihsim-battery",
  "group": 13,
  "group_name": "battery",
  "quantity": "simulated battery discharge and voltage",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/battery_simulator/BatterySimulator.cpp",
  "lines": "122-148",
  "code": "const float ibatt = -1.0f; // no current sensor in simulation\nconst float discharge_interval_us = math::max(fullDischargeTime(i), 1.0f) * 1000 * 1000;\n_battery_percentage[i] -= (now_us - _last_integration_us) / discharge_interval_us;\n_battery_percentage[i] = math::max(_battery_percentage[i], minimumPercentage(i) / 100.f);\nfloat vbatt = math::interpolate(_battery_percentage[i], 0.f, 1.f, battery.empty_cell_voltage(), battery.full_cell_voltage());\nvbatt *= battery.cell_count();",
  "formula": "SoC ← SoC − Δt/T_drain (only while armed; =1 when disarmed) ; SoC ≥ SIM_BAT_MIN_PCT/100 ; V = N_cells·(V_empty + SoC·(V_full − V_empty)) ; I = −1 (none)",
  "variables": {
   "T_drain": "SIM_BAT_DRAIN [s]",
   "N_cells": "BAT1_N_CELLS"
  },
  "defaults": [
   {
    "name": "SIM_BAT_DRAIN",
    "value": "60",
    "unit": "s",
    "where": "battery_simulator_params.yaml:11"
   },
   {
    "name": "SIM_BAT_MIN_PCT",
    "value": "50.0",
    "unit": "%",
    "where": "battery_simulator_params.yaml:36"
   }
  ],
  "assumptions": "Time-based (not load-based) linear discharge, no internal resistance, no sag, no current; cell voltage limits come from BAT1_V_EMPTY / BAT1_V_CHARGED parameters (not read here).",
  "notes": "The battery has no connection to thrust: SIH motor thrust is independent of voltage."
 },
 {
  "id": "g01-px4-atmosphere",
  "group": 1,
  "group_name": "atmosphere and gravity",
  "quantity": "standard atmosphere (first layer, up to 11 km): pressure, temperature, density",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/atmosphere/atmosphere.cpp",
  "lines": "45-77",
  "code": "return (pressure_pa / (kAirGasConstant * (temperature_celsius - kAbsoluteNullCelsius)));\nreturn kPressRefSeaLevelPa * powf((altitude_m * kTempGradient + kTempRefKelvin) / kTempRefKelvin, -CONSTANTS_ONE_G / (kTempGradient * kAirGasConstant));\nreturn 15.0f + kTempGradient * altitude_m;",
  "formula": "ρ = p/(R·T) ; p(h) = p0·((T0 + L·h)/T0)^(−g/(L·R)) ; T(h) = 15 °C + L·h",
  "variables": {
   "R": "287.1 J/(kg·K)",
   "L": "−6.5e-3 K/m",
   "T0": "288.15 K",
   "p0": "101325 Pa",
   "g": "CONSTANTS_ONE_G = 9.80665 m/s²"
  },
  "defaults": [
   {
    "name": "kAirGasConstant",
    "value": "287.1",
    "unit": "J/(kg K)",
    "where": "atmosphere.h:53"
   },
   {
    "name": "kTempGradient",
    "value": "-6.5e-3",
    "unit": "K/m",
    "where": "atmosphere.h:56"
   },
   {
    "name": "kPressRefSeaLevelPa",
    "value": "101325",
    "unit": "Pa",
    "where": "atmosphere.h:57"
   },
   {
    "name": "kAirDensitySeaLevelStandardAtmos",
    "value": "1.225",
    "unit": "kg/m^3",
    "where": "atmosphere.h:47"
   }
  ],
  "assumptions": "Header notes the model is valid only up to 11 km; gravity constant 9.80665 here, exponent g/(L R) = 5.2559.",
  "notes": "R=287.1 here vs 287.04 in SIH aero.hpp."
 },
 {
  "id": "g02-pxgz-motor-thrust",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "PX4 gz GenericMotorModel thrust (velocity motor type)",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/generic_motor/GenericMotorModel.cpp",
  "lines": "802-820",
  "code": "double realMotorVelocity = motorRotVel * this->rotorVelocitySlowdownSim;\nint realMotorVelocitySign = (realMotorVelocity > 0) - (realMotorVelocity < 0);\ndouble thrust = this->turningDirection * realMotorVelocitySign * realMotorVelocity * realMotorVelocity * this->motorConstant;\nlink.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));",
  "formula": "T = s_dir·sign(ω)·ω²·k_f , ω = 10·joint rate, force along link +z",
  "variables": {
   "k_f": "motorConstant",
   "s_dir": "turningDirection (cw −1, ccw +1)"
  },
  "defaults": [
   {
    "name": "motorConstant",
    "value": "8.54858e-06",
    "unit": "N/(rad/s)^2",
    "where": "GenericMotorModel.cpp:208"
   },
   {
    "name": "maxRotVelocity",
    "value": "838.0",
    "unit": "rad/s",
    "where": "GenericMotorModel.cpp:184"
   },
   {
    "name": "rotorVelocitySlowdownSim",
    "value": "10.0",
    "unit": "-",
    "where": "GenericMotorModel.cpp:227"
   }
  ],
  "assumptions": "Quadratic in rotor speed, no density/advance ratio; this is PX4's own copy of the gz-sim MulticopterMotorModel; scanning order: file read lines 1-931 in full.",
  "notes": "PX4 simulation plugin; the standard x500 model in PX4-gazebo-models uses gz-sim's MulticopterMotorModel instead (outside this scope). Simulator tag 'Gazebo' because the code runs inside gz-sim."
 },
 {
  "id": "g03-pxgz-motor-lag",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "PX4 gz GenericMotorModel first-order lag with up/down time constants",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/generic_motor/GenericMotorModel.cpp",
  "lines": "77-91, 911-920",
  "code": "double alphaUp = exp(-_samplingTime / timeConstantUp);\noutputState = alphaUp * previousState + (1 - alphaUp) * _inputState;\n...\nrefMotorRotVel = this->rotorVelocityFilter->UpdateFilter(this->refMotorInput, this->samplingTime);\n*jointVelCmd = components::JointVelocityCmd({this->turningDirection *refMotorRotVel / this->rotorVelocitySlowdownSim});",
  "formula": "ω(k+1) = e^{−Δt/τ}ω(k) + (1−e^{−Δt/τ})ω_cmd with τ = τ_up if ω_cmd > ω else τ_down ; joint cmd = s_dir·ω/10 ; command clamp ω_cmd ≤ ω_max (lines 702-704)",
  "variables": {
   "τ": "time constants [s]"
  },
  "defaults": [
   {
    "name": "timeConstantUp",
    "value": "0.0125 (1/80)",
    "unit": "s",
    "where": "GenericMotorModel.cpp:235"
   },
   {
    "name": "timeConstantDown",
    "value": "0.025 (1/40)",
    "unit": "s",
    "where": "GenericMotorModel.cpp:231"
   }
  ],
  "assumptions": "Exact ZOH discretisation; asymmetric; saturation only on the upper side.",
  "notes": ""
 },
 {
  "id": "g04-pxgz-rotor-drag",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "PX4 gz GenericMotorModel rotor drag, rolling moment and reaction torque",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/generic_motor/GenericMotorModel.cpp",
  "lines": "858-864, 885-886, 894-898",
  "code": "Vector3 relativeWindVelocityWorld = bodyVelocityWorld - windSpeedWorld;\nVector3 bodyVelocityPerpendicular = relativeWindVelocityWorld - (relativeWindVelocityWorld.Dot(jointAxis) * jointAxis);\nVector3 airDrag = -std::abs(realMotorVelocity) * this->rotorDragCoefficient * bodyVelocityPerpendicular;\nVector3 dragTorque(0, 0, -this->turningDirection * thrust * this->momentConstant);\nrollingMoment = -std::abs(realMotorVelocity) * this->rollingMomentCoefficient * bodyVelocityPerpendicular;",
  "formula": "F_drag = −|ω|·C_D·v_⊥ ; M_roll = −|ω|·C_M·v_⊥ ; τ_react = −s_dir·T·k_m (about axis) ; v_⊥ = (v − v_wind) minus its component along the rotor axis",
  "variables": {
   "C_D": "rotorDragCoefficient",
   "C_M": "rollingMomentCoefficient",
   "k_m": "momentConstant"
  },
  "defaults": [
   {
    "name": "rotorDragCoefficient",
    "value": "1.0e-4",
    "unit": "kg/rad",
    "where": "GenericMotorModel.cpp:220"
   },
   {
    "name": "rollingMomentCoefficient",
    "value": "1.0e-6",
    "unit": "-",
    "where": "GenericMotorModel.cpp:216"
   },
   {
    "name": "momentConstant",
    "value": "0.016",
    "unit": "m",
    "where": "GenericMotorModel.cpp:204"
   }
  ],
  "assumptions": "Source cites Martin and Salaun ICRA 2010 (comment at 851-854); wind = world Wind entity velocity, so relative velocity is v_body − v_wind (wind is the air velocity, opposite sign to SIH_WIND).",
  "notes": ""
 },
 {
  "id": "g16-pxgz-polynomial-thrust",
  "group": 16,
  "group_name": "thrust curve (command to thrust)",
  "quantity": "force_polynomial motor type: thrust and torque as polynomials of the command",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/generic_motor/GenericMotorModel.cpp",
  "lines": "741-783",
  "code": "for (unsigned int i = 0; i < this->positiveThrustPolynomial.size(); i++) {\n\tthrust += this->positiveThrustPolynomial[i] * std::pow(this->refMotorInput, i);\n}\nlink.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));\n...\nlink.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, torque)));",
  "formula": "T = Σ c_i·u^i (negative branch: −Σ c_i|u|^i) ; torque polynomial likewise; duty-cycle mapping u = (d − (d_min + (d_max−d_min)b/2)) / ((d_max−d_min) − (d_max−d_min)b/2) with b = bidirectionalMotor",
  "variables": {
   "u": "command (rpm or normalised duty cycle)",
   "c_i": "polynomial coefficients from SDF (at least 4 values)"
  },
  "defaults": [
   {
    "name": "minCommand",
    "value": "0.0",
    "unit": "-",
    "where": "GenericMotorModel.cpp:188"
   }
  ],
  "assumptions": "The torque polynomial result is applied through AddWorldForce (line 782) i.e. as a FORCE along z on the link, not as a torque: this looks like a bug in the plugin (reported as found, not fixed).",
  "notes": "No lag filter is applied in this branch (filter code is in the velocity branch)."
 },
 {
  "id": "g14-pxgz-airspeed-plugin",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "PX4 gz airspeed plugin: differential pressure from air-relative body velocity",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/airspeed/AirSpeed.cpp",
  "lines": "117-148",
  "code": "const float density_ratio = powf(TEMPERATURE_MSL / temperature_local, 4.256f);\nconst float air_density = AIR_DENSITY_MSL / density_ratio;\ngz::math::Vector3d air_relative_velocity = _vehicle_velocity - _wind_velocity;\ngz::math::Vector3d body_velocity = _vehicle_attitude.RotateVectorReverse(air_relative_velocity);\ndouble diff_pressure = sign(body_velocity.X()) * 0.005 * (double)air_density  * body_velocity.X() * body_velocity.X() + (double)diff_pressure_noise;\nairspeed_msg.set_diff_pressure(diff_pressure * 100.0);",
  "formula": "Δp[Pa] = 100·(sign(v_x)·0.005·ρ·v_x² + n) = ½ρv_x² + 100n ; v = R^T(v_vehicle − v_wind) ; ρ = 1.225·(T/288.15)^4.256",
  "variables": {
   "n": "N(0, diff_pressure_stddev_²) in hPa"
  },
  "defaults": [],
  "assumptions": "WIND SIGN: wind = air velocity in the gz world (ENU), relative velocity = vehicle − wind (line 142); opposite sign to SIH_WIND (which is added to ground velocity); no gust model; wind from /world/<w>/wind_info topic or Wind entity.",
  "notes": "Constants TEMPERATURE_MSL, LAPSE_RATE, AIR_DENSITY_MSL come from the sensor_airspeed_sim header style; noise stddev member not read in this pass."
 },
 {
  "id": "g16-pxgz-esc-command",
  "group": 16,
  "group_name": "thrust curve (command to thrust)",
  "quantity": "PX4 gz bridge: mixer output to rotor speed command",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp",
  "lines": "GZMixingInterfaceESC.cpp:98-100 ; mixer_module.cpp:565 ; 4001_gz_x500:35-50",
  "code": "vel->Add(static_cast<double>(outputs[i]));\n...\noutput = math::interpolate(value, -1.f, 1.f, static_cast<float>(_min_value[i]), static_cast<float>(_max_value[i]));\n...\nparam set-default SIM_GZ_EC_MIN1 150\nparam set-default SIM_GZ_EC_MAX1 1000",
  "formula": "ω_cmd [rad/s] = 150 + (1000 − 150)·(x+1)/2 where x∈[−1,1] is the mixer value (motors: normalised thrust u∈[0,1] mapped to x = 2u−1 in FunctionMotors.hpp:116) ⇒ ω_cmd = 150 + 850·u ; then plant thrust = k_f·ω² ⇒ T(u) = k_f(150 + 850u)²",
  "variables": {
   "u": "normalised thrust command after THR_MDL_FAC inversion",
   "SIM_GZ_EC_MIN/MAX": "output limits [rad/s]"
  },
  "defaults": [
   {
    "name": "SIM_GZ_EC_MIN1..4",
    "value": "150",
    "unit": "rad/s",
    "where": "4001_gz_x500:40-43"
   },
   {
    "name": "SIM_GZ_EC_MAX1..4",
    "value": "1000",
    "unit": "rad/s",
    "where": "4001_gz_x500:45-48"
   },
   {
    "name": "MPC_THR_HOVER",
    "value": "0.6",
    "unit": "-",
    "where": "4001_gz_x500:50"
   }
  ],
  "assumptions": "Gazebo thrust is quadratic in the command with an offset (idle speed 150 rad/s), while the allocator assumes thrust ∝ u unless THR_MDL_FAC is set (default 0, the airframe file sets none): a built-in mismatch between plant and allocator; the allocator itself is outside this scope.",
  "notes": "ESC rpm report: esc_rpm = actuators.velocity(i) (GZMixingInterfaceESC.cpp:132). Also in: src/lib/mixer_module/mixer_module.cpp; ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500. Quote condensed from several code lines."
 },
 {
  "id": "g02-pxgz-spacecraft-thruster",
  "group": 2,
  "group_name": "rotor thrust and torque",
  "quantity": "spacecraft thruster bang-bang PWM model",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/spacecraft_thruster/SpacecraftThrusterModel.cpp",
  "lines": "364-379",
  "code": "double force = this->samplingTime <= targetDutyCycle ? this->maxThrust : 0.0;\nlink.AddWorldForce(_ecm, worldPose->Rot().RotateVector(math::Vector3d(0, 0, force)));",
  "formula": "F = F_max if t_phase ≤ duty else 0 (time-proportional on/off thruster), along link z",
  "variables": {
   "F_max": "max_thrust from SDF"
  },
  "defaults": [],
  "assumptions": "Not an aircraft (cold-gas spacecraft thruster); included for completeness.",
  "notes": ""
 },
 {
  "id": "g07-pxgz-platform-noise",
  "group": 7,
  "group_name": "wind and gusts",
  "quantity": "moving-platform disturbance: low-pass filtered white noise force and torque",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/gz_plugins/moving_platform_controller/MovingPlatformController.cpp",
  "lines": "152-212",
  "code": "const double time_constant_force = 1. / (2 * GZ_PI * cutoff_freq_force);\nconst double filter_coef_force = dt / (dt + time_constant_force);\n_noise_lowpass_force = (1. - filter_coef_force) * _noise_lowpass_force + filter_coef_force * noise_force;\nconst double noise_ampl_force = noise_ampl_force_scaling * _platform_mass;",
  "formula": "y ← (1−a)y + a·w, a = dt/(dt+τ), τ = 1/(2π·5 Hz) ; platform force noise amplitude = platform mass [N] (1 N/kg), torque amplitude = diagonal moments",
  "variables": {
   "w": "N(0,1) per axis"
  },
  "defaults": [
   {
    "name": "cutoff_freq_force/torque",
    "value": "5.0",
    "unit": "Hz",
    "where": "MovingPlatformController.cpp:169-170"
   }
  ],
  "assumptions": "Disturbs a ship/vehicle platform (waves, road), not the drone; no wind.",
  "notes": "Only the noise section of the file (lines 100-212) was read in detail."
 },
 {
  "id": "g14-pxsim-wmm-lookup",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "World Magnetic Model lookup (declination, inclination, strength) used by the mag sim",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/world_magnetic_model/geo_mag_declination.cpp",
  "lines": "96-125",
  "code": "const float data_min = lon_scale * (data_se - data_sw) + data_sw;\nconst float data_max = lon_scale * (data_ne - data_nw) + data_nw;\nreturn lat_scale * (data_max - data_min) + data_min;\n...\nreturn get_mag_strength_tesla(latitude_deg, longitude_deg) * 1e4f;",
  "formula": "bilinear interpolation on a precomputed WMM table (SAMPLING_RES grid): D, I, F ; F[gauss] = F[tesla]·1e4",
  "variables": {},
  "defaults": [],
  "assumptions": "Tables in geo_magnetic_tables.hpp (stored scaled integers: declination/inclination scale factors, intensity in scaled nT); no time evolution; no altitude dependence.",
  "notes": "Table contents not read; the comment at line 118 says '1 Gauss = 1e4 Tesla' which has the conversion inverted (1 T = 1e4 G), the code multiplies by 1e4 as required for tesla to gauss."
 },
 {
  "id": "g08-pxsim-map-projection",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "azimuthal equidistant map projection (lat/lon to local NED metres) used by SIH ground truth",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/geo/geo.cpp",
  "lines": "67-88",
  "code": "const double c = acos(arg);\ndouble k = 1.0;\nif (fabs(c) > 0) { k = (c / sin(c)); }\nx = static_cast<float>(k * (_ref_cos_lat * sin_lat - _ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);\ny = static_cast<float>(k * cos_lat * sin(lon_rad - _ref_lon) * CONSTANTS_RADIUS_OF_EARTH);",
  "formula": "c = acos(sinφ0 sinφ + cosφ0 cosφ cosΔλ) ; k = c/sin c ; x_N = k(cosφ0 sinφ − sinφ0 cosφ cosΔλ)·R ; y_E = k cosφ sinΔλ·R ; R = 6371000 m",
  "variables": {
   "R": "CONSTANTS_RADIUS_OF_EARTH"
  },
  "defaults": [
   {
    "name": "CONSTANTS_RADIUS_OF_EARTH",
    "value": "6371000",
    "unit": "m",
    "where": "src/lib/geo/geo.h:55"
   }
  ],
  "assumptions": "Spherical Earth (R=6371 km) here while the SIH state uses WGS84 ECEF: local-position ground truth is a spherical projection of a WGS84 position (small inconsistency of order 0.3 %).",
  "notes": "Remaining geo.cpp functions (distance, bearing, waypoint) are navigation geometry, not scanned for physics. CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5 rad/s (geo.h:57), CONSTANTS_ONE_G = 9.80665 (geo.h:54)."
 },
 {
  "id": "g14-pxsim-mavlink-notfound",
  "group": 14,
  "group_name": "sensors and noise",
  "quantity": "simulator_mavlink (HIL) sensor processing",
  "simulator": "PX4 SIH",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp",
  "lines": "165, 213, 289, 308",
  "code": "esc_status.esc[i].esc_temperature = 20.0 + math::abs_t(hil_act_control.controls[i]) * 40.0;\nstatic constexpr float ACCEL_FIFO_SCALE = CONSTANTS_ONE_G / 2048.f;\n_px4_baro[sensors.id].update(time, sensors.abs_pressure * 100.f); // hPa to Pa\nreport.differential_pressure_pa = sensors.diff_pressure * 100.f * airspeed_blockage_scale;",
  "formula": "not found as a physics model: unit conversions only (hPa to Pa, cm to m, 1e-7 deg), ESC temperature T = 20 + 40·|cmd| °C, accel FIFO scale g/2048; sensor values come from the external simulator",
  "variables": {},
  "defaults": [],
  "assumptions": "Grep-level scan only of the 1500+ line file (no noise or physics equations found).",
  "notes": "pwm_out_sim/PWMSim.cpp and system_power_simulator/SystemPowerSimulator.cpp: grep found no physics expressions (system power: constant 5 V)."
 }
];
