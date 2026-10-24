window.SIM_FORMULAS_PX4AP = [
 {
  "id": "px4ap-001",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Hover-thrust estimator: predicted vertical acceleration from normalized thrust",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp",
  "lines": "102-105",
  "code": "float ZeroOrderHoverThrustEkf::computePredictedAccZ(const float thrust) const\n{\n\treturn CONSTANTS_ONE_G * thrust / _hover_thr - CONSTANTS_ONE_G;\n}",
  "formula": "a_z,pred = g·(T / T_hover) − g   (z positive UP; a_z is the specific-force-free kinematic acceleration)",
  "variables": {
   "g": "standard gravity CONSTANTS_ONE_G = 9.80665 [m/s^2]",
   "T": "measured/allocated collective thrust, normalized [0..1], positive up [-]",
   "T_hover": "estimated state: normalized thrust that holds hover [-]",
   "a_z,pred": "predicted vertical acceleration, positive up [m/s^2]"
  },
  "defaults": [
   {
    "name": "_hover_thr initial",
    "value": "0.5",
    "unit": "normalized thrust",
    "where": "zero_order_hover_thrust_ekf.hpp:97"
   },
   {
    "name": "_hover_thr_min / _hover_thr_max",
    "value": "0.1 / 0.9",
    "unit": "normalized thrust",
    "where": "zero_order_hover_thrust_ekf.hpp:98-99"
   },
   {
    "name": "HTE_THR_RANGE",
    "value": "0.2",
    "unit": "normalized thrust",
    "where": "hover_thrust_estimator_params.yaml:81"
   }
  ],
  "assumptions": "Thrust force is linear in the normalized thrust command and acts along the NED vertical after rotation by attitude; mass constant; drag and prop wash ignored (the noise scaling in the caller handles fast motion).",
  "notes": "Linear thrust-to-acceleration model: at T = T_hover the prediction is 0. The estimate is clamped to [MPC_THR_HOVER − HTE_THR_RANGE, MPC_THR_HOVER + HTE_THR_RANGE] by the caller (MulticopterHoverThrustEstimator.cpp:90-91)."
 },
 {
  "id": "px4ap-002",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Hover-thrust estimator: measurement Jacobian",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp",
  "lines": "84-87",
  "code": "inline float ZeroOrderHoverThrustEkf::computeH(const float thrust) const\n{\n\treturn -CONSTANTS_ONE_G * thrust / (_hover_thr * _hover_thr);\n}",
  "formula": "H = ∂a_z,pred/∂T_hover = −g·T / T_hover²",
  "variables": {
   "H": "scalar measurement Jacobian [m/s^2 per unit thrust]",
   "T": "normalized thrust [-]",
   "T_hover": "hover thrust state [-]"
  },
  "defaults": [],
  "assumptions": "Derivative of the model in px4ap-001.",
  "notes": "Innovation = a_z,meas − a_z,pred (computeInnov, line 96-100). Innovation variance max(H·P·H + R, R) with R = acc_var·scale² (line 89-94)."
 },
 {
  "id": "px4ap-003",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Hover-thrust estimator: process model (random-walk hover thrust)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp",
  "lines": "44-50",
  "code": "void ZeroOrderHoverThrustEkf::predict(const float dt)\n{\n\t// State is constant\n\t// Predict state covariance only\n\t_state_var += _process_var * dt * dt;\n}",
  "formula": "T_hover,k+1 = T_hover,k ;  P_k+1 = P_k + q·dt²",
  "variables": {
   "q": "process-noise variance = HTE_HT_NOISE² [(normalized thrust/s)^2]",
   "P": "hover-thrust state variance [thrust^2]",
   "dt": "step [s]"
  },
  "defaults": [
   {
    "name": "HTE_HT_NOISE",
    "value": "0.0036",
    "unit": "normalized_thrust/s",
    "where": "hover_thrust_estimator_params.yaml:13"
   },
   {
    "name": "_process_var initial",
    "value": "12.5e-6",
    "unit": "thrust^2/s^2",
    "where": "zero_order_hover_thrust_ekf.hpp:103 (overwritten by param at MulticopterHoverThrustEstimator.cpp:82)"
   },
   {
    "name": "HTE_HT_ERR_INIT",
    "value": "0.1",
    "unit": "normalized_thrust",
    "where": "hover_thrust_estimator_params.yaml:38"
   },
   {
    "name": "HTE_ACC_GATE",
    "value": "3.0",
    "unit": "SD",
    "where": "hover_thrust_estimator_params.yaml:25"
   }
  ],
  "assumptions": "Hover thrust is a slowly varying constant (battery sag, payload); P grows with dt² (q is treated per-step as a rate, not a PSD).",
  "notes": "Covariance growth uses dt² not dt, as written in code. Also: bumpStateVariance adds 1e3·q·dt² when large offset detected (lines 132-140). Measurement noise is adapted online: acc_var ← (1−α)acc_var + α(res² + H P H), clamped 1..400 (lines 150-156); initial acc_var 5 m^2/s^3 (hpp:104)."
 },
 {
  "id": "px4ap-004",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Hover-thrust estimator: measurement construction (NED vertical acceleration, thrust projected by attitude)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp",
  "lines": "173-190",
  "code": "matrix::Vector3f thrust_body_allocated = thrust_body_sp - thrust_body_unallocated;\nconst matrix::Quatf q_att{vehicle_attitude.q};\nmatrix::Vector3f thrust_allocated = q_att.rotateVector(thrust_body_allocated);\n// ... positive acceleration is up ... positive thrust is up\n_hover_thrust_ekf.fuseAccZ(-local_pos.az, -thrust_allocated(2));",
  "formula": "T = −(R(q)·(T_sp,body − T_unalloc,body))_z ;  a_meas = −a_z,NED",
  "variables": {
   "T_sp,body": "vehicle_thrust_setpoint xyz, body FRD [normalized]",
   "T_unalloc,body": "control_allocator_status.unallocated_thrust [normalized]",
   "R(q)": "body-to-NED rotation from vehicle_attitude",
   "a_z,NED": "local_position.az, positive DOWN [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Thrust that the allocator could not realize is subtracted; vector rotated to NED so only vertical component is used; valid only when armed, in air, dt in (0.001,1) s.",
  "notes": "Sign convention: NED z down, so both inputs are negated to make 'up' positive. Estimator marked valid when state variance < 0.001 (line 192)."
 },
 {
  "id": "px4ap-005",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Hover-thrust estimator: measurement-noise inflation with speed (drag / prop-wash guard)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp",
  "lines": "184-189",
  "code": "const float meas_noise_coeff_z = fmaxf((fabsf(local_pos.vz) - _param_hte_vz_thr.get()) + 1.f, 1.f);\nconst float meas_noise_coeff_xy = fmaxf((matrix::Vector2f(local_pos.vx, local_pos.vy).norm() - _param_hte_vxy_thr.get()) + 1.f, 1.f);\n_hover_thrust_ekf.setMeasurementNoiseScale(fmaxf(meas_noise_coeff_xy, meas_noise_coeff_z));",
  "formula": "s = max( max(|v_z| − V_z,thr + 1, 1), max(‖v_xy‖ − V_xy,thr + 1, 1) );  R = acc_var · s²",
  "variables": {
   "v_z, v_xy": "local-position velocities [m/s]",
   "V_z,thr": "HTE_VZ_THR [m/s]",
   "V_xy,thr": "HTE_VXY_THR [m/s]",
   "s": "noise scale factor [-]"
  },
  "defaults": [
   {
    "name": "HTE_VZ_THR",
    "value": "2.0",
    "unit": "m/s",
    "where": "hover_thrust_estimator_params.yaml:66"
   },
   {
    "name": "HTE_VXY_THR",
    "value": "10.0",
    "unit": "m/s",
    "where": "hover_thrust_estimator_params.yaml:52"
   }
  ],
  "assumptions": "Fast vertical/horizontal motion adds unmodeled drag and prop wash that bias the a_z-vs-thrust relation; model them as extra noise rather than modeling drag.",
  "notes": "setMeasurementNoiseScale stores scale² (zero_order_hover_thrust_ekf.hpp:83)."
 },
 {
  "id": "px4ap-006",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Multicopter land detector: free-fall from specific-force norm",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/land_detector/MulticopterLandDetector.cpp",
  "lines": "161-165",
  "code": "bool MulticopterLandDetector::_get_freefall_state()\n{\n\t// norm of specific force. Should be close to 9.8 m/s^2 when landed.\n\treturn _acceleration.norm() < 2.f;\n}",
  "formula": "free-fall ⇔ ‖f_b‖ < 2 m/s²  (specific force ≈ 0 in free fall, ≈ g at rest)",
  "variables": {
   "f_b": "accelerometer specific force [m/s^2]"
  },
  "defaults": [
   {
    "name": "freefall hysteresis time",
    "value": "300 ms",
    "unit": "ms",
    "where": "MulticopterLandDetector.cpp:84"
   }
  ],
  "assumptions": "Accelerometer measures specific force (no gravity), so free fall reads near zero. Source comment line 57 notes 'TODO: verify that free fall is reliable'.",
  "notes": "Threshold is hard-coded."
 },
 {
  "id": "px4ap-007",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Multicopter land detector: low-throttle thresholds between minimum and hover thrust",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/land_detector/MulticopterLandDetector.cpp",
  "lines": "231-235, 283-288",
  "code": "const float thr_pct_hover = _hover_thrust_estimate_valid ? 0.6f : 0.3f;\nconst float hover_thrust = PX4_ISFINITE(_hover_thrust_estimate) ? _hover_thrust_estimate : _params.mpc_thr_hover;\nconst float sys_low_throttle = _params.minThrottle + (hover_thrust - _params.minThrottle) * thr_pct_hover;\n// 10% of throttle range between min and hover\nminimum_thrust_threshold = _params.minThrottle + (hover_thrust - _params.minThrottle) * 0.1f;",
  "formula": "T_low = T_min + (T_hover − T_min)·p,  p = 0.6 (hover estimate valid) or 0.3;  T_minimum = T_min + 0.1(T_hover − T_min)",
  "variables": {
   "T_min": "MPC_THR_MIN [normalized]",
   "T_hover": "hover-thrust estimate or MPC_THR_HOVER [normalized]"
  },
  "defaults": [
   {
    "name": "LNDMC_Z_VEL_MAX",
    "value": "0.25",
    "unit": "m/s",
    "where": "land_detector_params_mc.yaml:14"
   },
   {
    "name": "LNDMC_XY_VEL_MAX",
    "value": "1.5",
    "unit": "m/s",
    "where": "land_detector_params_mc.yaml:23"
   },
   {
    "name": "LNDMC_ROT_MAX",
    "value": "20.0",
    "unit": "deg/s",
    "where": "land_detector_params_mc.yaml:32"
   }
  ],
  "assumptions": "Thrust is linear in normalized throttle; landed when thrust near idle while not moving.",
  "notes": "Detection logic, not a force model; included because it uses the hover-thrust estimate."
 },
 {
  "id": "px4ap-008",
  "group": 9,
  "group_name": "rotation",
  "quantity": "EKF2 attitude prediction from gyro delta-angle with Earth-rate and transport-rate compensation",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "245-256",
  "code": "const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;\nVector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;\nconst Vector3f transport_rate = _gpos.computeAngularRateNavFrame(_state.vel);\ncorrected_delta_ang -= _R_to_earth.transpose() * (_earth_rate_NED + transport_rate) * imu_delayed.delta_ang_dt;\nconst Quatf dq(AxisAnglef{corrected_delta_ang});\n_state.quat_nominal = (_state.quat_nominal * dq).normalized();",
  "formula": "Δθ_c = Δθ_gyro − b_g·Δt − Rᵀ(ω_ie^n + ω_en^n)·Δt ;  q ← normalize(q ⊗ dq(Δθ_c))",
  "variables": {
   "Δθ_gyro": "IMU delta angle [rad]",
   "b_g": "gyro bias state [rad/s]",
   "ω_ie^n": "Earth rotation rate in NED [rad/s]",
   "ω_en^n": "transport rate of NED frame over curved Earth [rad/s]",
   "R": "body-to-NED rotation",
   "q": "quaternion body→NED (Hamilton, state quat_nominal)"
  },
  "defaults": [],
  "assumptions": "Strapdown integration over one IMU delta; NED frame rotation due to Earth spin and vehicle motion is removed from gyro measurement.",
  "notes": "Uses Hamilton quaternions, right-multiplication of body increment."
 },
 {
  "id": "px4ap-009",
  "group": 8,
  "group_name": "translation",
  "quantity": "EKF2 velocity prediction: rotated specific force (delta velocity) minus accel bias",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "259-267",
  "code": "const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;\nconst Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;\nconst Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;\nconst Vector3f vel_last = _state.vel;\n_state.vel += corrected_delta_vel_ef;",
  "formula": "v ← v + R·(Δv_imu − b_a·Δt)",
  "variables": {
   "Δv_imu": "IMU delta velocity = ∫ specific force dt in body frame [m/s]",
   "b_a": "accel bias state [m/s^2]",
   "R": "body-to-NED rotation",
   "v": "NED velocity [m/s]"
  },
  "defaults": [],
  "assumptions": "Specific force (no gravity) measured by accelerometer; gravity, Coriolis, transport added separately (px4ap-010..012). Uses orientation after the attitude update of this step.",
  "notes": "Velocity is then clamped to ±EKF2 velocity limit 100 m/s (ekf.cpp:280, common.h:323 ekf2_vel_lim)."
 },
 {
  "id": "px4ap-010",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "EKF2 gravity acceleration term in velocity prediction (NED, +z down)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "269-273",
  "code": "const Vector3f gravity_acceleration(0.f, 0.f, _gravity);\nconst Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);\nconst Vector3f transport_rate_acceleration = -transport_rate.cross(vel_last);\n_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate_acceleration) * imu_delayed.delta_vel_dt;",
  "formula": "v ← v + (g_n + a_cor + a_transport)·Δt ;  g_n = [0, 0, g(φ)]ᵀ (down positive)",
  "variables": {
   "g(φ)": "latitude-dependent gravity magnitude (_gravity) [m/s^2]",
   "Δt": "IMU delta_vel_dt [s]"
  },
  "defaults": [
   {
    "name": "_gravity initial",
    "value": "CONSTANTS_ONE_G = 9.80665",
    "unit": "m/s^2",
    "where": "ekf.h:546; geo.h:54"
   }
  ],
  "assumptions": "Gravity is vertical (NED +z) and constant over the step; no altitude dependence.",
  "notes": "Sign: NED z down, so gravity ADDS +g to v_z. The symbolic derivation (python/ekf_derivation/derivation.py:170,182) uses a plain constant g: vel + (R*accel + [0,0,g])*dt without Coriolis (covariance model only)."
 },
 {
  "id": "px4ap-011",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "EKF2 Coriolis acceleration",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "271",
  "code": "const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);",
  "formula": "a_cor = −2 ω_ie^n × v",
  "variables": {
   "ω_ie^n": "Earth spin vector in NED [rad/s]",
   "v": "NED velocity from previous step [m/s]"
  },
  "defaults": [],
  "assumptions": "Uses velocity at start of step (vel_last).",
  "notes": "Present in code (not neglected). Applied in the same Δt step as gravity."
 },
 {
  "id": "px4ap-012",
  "group": 8,
  "group_name": "translation",
  "quantity": "EKF2 transport-rate (curved Earth) acceleration",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "272",
  "code": "const Vector3f transport_rate_acceleration = -transport_rate.cross(vel_last);",
  "formula": "a_transport = −ω_en^n × v",
  "variables": {
   "ω_en^n": "NED-frame angular rate due to motion over curved Earth (px4ap-016) [rad/s]"
  },
  "defaults": [],
  "assumptions": "Spherical/ellipsoidal Earth via radii of curvature.",
  "notes": "Same sign convention as Coriolis term."
 },
 {
  "id": "px4ap-013",
  "group": 8,
  "group_name": "translation",
  "quantity": "EKF2 position prediction by trapezoidal integration on the ellipsoid",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "275-277",
  "code": "// predict position states via trapezoidal integration of velocity\n_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;\n_state.pos(2) = -_gpos.altitude();",
  "formula": "p ← p + ½(v_prev + v_new)·Δt  (applied as lat/lon/alt increment, see px4ap-017); z_down = −altitude",
  "variables": {
   "p": "geodetic position (_gpos, LatLonAlt)",
   "Δt": "delta_vel_dt [s]"
  },
  "defaults": [],
  "assumptions": "Position kept as latitude/longitude/altitude (this commit), not local NED; altitude is up-positive, state.pos(2) down-positive.",
  "notes": "The derivation file uses Euler pos + vel·dt (derivation.py:171,183) for the covariance."
 },
 {
  "id": "px4ap-014",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Latitude-dependent gravity (Somigliana formula)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/lat_lon_alt/lat_lon_alt.hpp",
  "lines": "109-126",
  "code": "static constexpr double gravity_equator = 9.7803253359;\nstatic constexpr double somigliana_coefficient = 0.001931851353;\nconst double sin_lat2 = sin_lat * sin_lat;\nconst double g = gravity_equator * (1.0 + somigliana_coefficient * sin_lat2)\n\t\t / std::sqrt(1.0 - eccentricity2 * sin_lat2);",
  "formula": "g(φ) = g_e·(1 + k·sin²φ) / √(1 − e²·sin²φ),  g_e = 9.7803253359, k = 0.001931851353, e = 0.0818191908425",
  "variables": {
   "φ": "geodetic latitude [rad]",
   "g_e": "equatorial normal gravity [m/s^2]",
   "k": "Somigliana coefficient [-]",
   "e": "WGS-84 first eccentricity [-]"
  },
  "defaults": [
   {
    "name": "WGS84 equatorial radius",
    "value": "6378137.0",
    "unit": "m",
    "where": "lat_lon_alt.hpp:110"
   }
  ],
  "assumptions": "Normal gravity on the ellipsoid surface; no free-air altitude correction. Used in Ekf::predictState when the global origin is valid and latitude moved > 1 deg from the reference (ekf.cpp:235-242).",
  "notes": "Contradiction with rest of PX4 autopilot: hover estimator, TECS, atmosphere, baro compensation all use constant 9.80665 (CONSTANTS_ONE_G); EKF2 uses g(φ) (≈9.780 to 9.832)."
 },
 {
  "id": "px4ap-015",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Earth rotation vector in NED",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf_helper.cpp",
  "lines": "59-64",
  "code": "Vector3f Ekf::calcEarthRateNED(float lat_rad) const\n{\n\treturn Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),\n\t\t\t0.0f,\n\t\t\t-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));\n}",
  "formula": "ω_ie^n = Ω·[cos φ, 0, −sin φ]ᵀ,  Ω = 7.2921150e-5 rad/s",
  "variables": {
   "Ω": "Earth spin rate CONSTANTS_EARTH_SPIN_RATE (src/lib/geo/geo.h:57) [rad/s]",
   "φ": "latitude [rad]"
  },
  "defaults": [
   {
    "name": "CONSTANTS_EARTH_SPIN_RATE",
    "value": "7.2921150e-5",
    "unit": "rad/s",
    "where": "src/lib/geo/geo.h:57"
   }
  ],
  "assumptions": "Recomputed when latitude changes by > 1 deg.",
  "notes": "NED: x north, z down, hence −sin φ in z."
 },
 {
  "id": "px4ap-016",
  "group": 9,
  "group_name": "rotation",
  "quantity": "Transport rate of the NED frame (angular rate from motion over Earth)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/lat_lon_alt/lat_lon_alt.cpp",
  "lines": "83-92",
  "code": "return Vector3f(\n\t       v_ned(1) / (static_cast<float>(r_e) + _altitude),\n\t       -v_ned(0) / (static_cast<float>(r_n) + _altitude),\n\t       -v_ned(1) * tanf(_latitude_rad) / (static_cast<float>(r_e) + _altitude));",
  "formula": "ω_en^n = [ v_E/(R_E+h),  −v_N/(R_N+h),  −v_E·tan φ/(R_E+h) ]ᵀ",
  "variables": {
   "R_N": "meridian radius of curvature [m]",
   "R_E": "transverse (prime vertical) radius of curvature [m]",
   "h": "altitude above ellipsoid [m]",
   "v_N, v_E": "NED velocity components [m/s]"
  },
  "defaults": [],
  "assumptions": "Standard strapdown nav-frame rate; radii from px4ap-017.",
  "notes": ""
 },
 {
  "id": "px4ap-017",
  "group": 8,
  "group_name": "translation",
  "quantity": "Radii of curvature and metric conversion of lat/lon increments",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/lat_lon_alt/lat_lon_alt.cpp",
  "lines": "94-112",
  "code": "const double tmp = 1.0 - pow(Wgs84::eccentricity * sin(latitude), 2);\nconst double sqrt_tmp = std::sqrt(tmp);\nmeridian_radius_of_curvature = Wgs84::meridian_radius_of_curvature_numerator / (tmp * sqrt_tmp);\ntransverse_radius_of_curvature = Wgs84::equatorial_radius / sqrt_tmp;\nconst double dn_dlat = r_n + static_cast<double>(altitude);\nconst double de_dlon = (r_e + static_cast<double>(altitude)) * cos(latitude);",
  "formula": "R_N = a(1−e²)/(1−e² sin²φ)^{3/2};  R_E = a/√(1−e² sin²φ);  dN/dφ = R_N + h;  dE/dλ = (R_E + h)cos φ;  Δφ = ΔN/(R_N+h), Δλ = ΔE/((R_E+h)cos φ), Δh = −ΔD",
  "variables": {
   "a": "WGS-84 equatorial radius 6378137.0 [m]",
   "e": "eccentricity 0.0818191908425 [-]",
   "φ, λ, h": "geodetic latitude, longitude [rad], altitude [m]"
  },
  "defaults": [],
  "assumptions": "Local linearization at current latitude/altitude; used by operator+= (lines 129-155) to apply velocity·Δt as lat/lon increments.",
  "notes": ""
 },
 {
  "id": "px4ap-018",
  "group": 8,
  "group_name": "translation",
  "quantity": "EKF2 output predictor: delta velocity with gravity (fast INS propagation)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/output_predictor/output_predictor.cpp",
  "lines": "216-227",
  "code": "_R_to_earth_now = Dcmf(_output_new.quat_nominal);\nVector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};\n// correct for measured acceleration due to gravity\ndelta_vel_earth(2) += _gravity * delta_velocity_dt;",
  "formula": "Δv_n = R·Δv_corrected + [0,0,g]ᵀ·Δt",
  "variables": {
   "g": "_gravity set from Ekf via set_gravity (ekf.cpp:241) [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Output predictor runs at IMU rate; gravity only (Coriolis/transport not seen in the lines scanned).",
  "notes": "Only lines 215-232 and grep for gravity/Coriolis were inspected; full file not read."
 },
 {
  "id": "px4ap-019",
  "group": 15,
  "group_name": "estimators",
  "quantity": "EKF2 initial tilt from accelerometer (gravity direction)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/ekf.cpp",
  "lines": "215-231",
  "code": "if (accel_norm < 0.8f * CONSTANTS_ONE_G ||\n    accel_norm > 1.2f * CONSTANTS_ONE_G ||\n    gyro_norm > math::radians(15.0f)) {\n\treturn false;\n}\n_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));",
  "formula": "q₀ = rotation taking filtered specific force f_b to up vector [0,0,−1];  valid if 0.8g < ‖f‖ < 1.2g and ‖ω‖ < 15°/s",
  "variables": {
   "f_b": "low-pass accelerometer specific force [m/s^2]",
   "ω": "low-pass gyro rate [rad/s]"
  },
  "defaults": [],
  "assumptions": "Vehicle static: specific force points up with magnitude g.",
  "notes": "Yaw is not set here."
 },
 {
  "id": "px4ap-020",
  "group": 15,
  "group_name": "estimators",
  "quantity": "EKF2 gravity-vector measurement (accelerometer direction as attitude observation)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp",
  "lines": "52-68",
  "code": "const Vector3f measurement = Vector3f(imu.delta_vel / imu.delta_vel_dt - _state.accel_bias).unit();\nconst float measurement_var = math::max(sq(_params.ekf2_grav_noise), sq(0.01f));\nconst float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;\nconst float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;\nVector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement;",
  "formula": "z = (f_b − b_a)/‖f_b − b_a‖ ;  ĥ = Rᵀ·[0,0,−1]ᵀ ;  innovation = ĥ − z",
  "variables": {
   "f_b": "IMU delta_vel/delta_vel_dt [m/s^2]",
   "b_a": "accel bias state",
   "R": "body-to-NED rotation"
  },
  "defaults": [
   {
    "name": "EKF2_GRAV_NOISE",
    "value": "1.0",
    "unit": "g0",
    "where": "src/modules/ekf2/params_gravity.yaml"
   },
   {
    "name": "innovation gate",
    "value": "0.25",
    "unit": "-",
    "where": "gravity_fusion.cpp:81"
   },
   {
    "name": "accel norm window",
    "value": "0.9g to 1.1g",
    "unit": "m/s^2",
    "where": "gravity_fusion.cpp:55-59"
   }
  ],
  "assumptions": "Quasi-static: only used when no horizontal aiding is active and filtered accel norm within 10% of g (or vehicle at rest).",
  "notes": "Observation is a unit vector, so measured magnitude is ignored. Measurement noise value 1.0 for a unit-vector component."
 },
 {
  "id": "px4ap-021",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 relative wind (air-relative velocity) in body frame",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/drag/drag_fusion.cpp",
  "lines": "66-73",
  "code": "Vector3f Ekf::getRelativeWindBody() const\n{\n\tconst Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),\n\t\t\t\t      _state.vel(1) - _state.wind_vel(1),\n\t\t\t\t      _state.vel(2));\n\treturn _state.quat_nominal.rotateVectorInverse(rel_wind_earth);\n}",
  "formula": "v_rel^b = Rᵀ·([v_N − w_N, v_E − w_E, v_D]ᵀ)",
  "variables": {
   "v": "vehicle NED velocity state [m/s]",
   "w": "wind velocity state (N,E), direction the wind blows TO [m/s]",
   "R": "body-to-NED rotation"
  },
  "defaults": [],
  "assumptions": "Vertical wind is zero (only 2 wind states). Note: the vector is VEHICLE velocity relative to air, not the air velocity seen by the vehicle (sign opposite to 'relative wind').",
  "notes": "SIGN: wind_vel is the wind velocity vector (to-direction), so air-relative vehicle velocity = v − w. Same convention in airspeed (px4ap-029) and sideslip (px4ap-034) models."
 },
 {
  "id": "px4ap-022",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "EKF2 multirotor bluff-body drag (quadratic) specific force",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/drag/drag_fusion.cpp",
  "lines": "139-140",
  "code": "const float pred_acc = -0.5f * bcoef_inv(axis_index) * rho * rel_wind_body(axis_index) * rel_wind_speed\n\t\t       - rel_wind_body(axis_index) * mcoef_corrrected;",
  "formula": "a_drag,i = −½·(ρ/β_i)·v_rel,i·‖v_rel‖  (bluff-body term), i ∈ {x,y} body axes",
  "variables": {
   "ρ": "air density (max(_air_density, 0.1)) [kg/m^3]",
   "β_i": "ballistic coefficient EKF2_BCOEF_X / _Y [kg/m^2]; bcoef_inv = 1/β",
   "v_rel,i": "i-th body component of air-relative velocity [m/s]",
   "‖v_rel‖": "norm of the 3D body-frame air-relative velocity [m/s]"
  },
  "defaults": [
   {
    "name": "EKF2_BCOEF_X",
    "value": "100.0",
    "unit": "kg/m^2",
    "where": "src/modules/ekf2/params_drag.yaml"
   },
   {
    "name": "EKF2_BCOEF_Y",
    "value": "100.0",
    "unit": "kg/m^2",
    "where": "src/modules/ekf2/params_drag.yaml"
   },
   {
    "name": "EKF2_DRAG_CTRL",
    "value": "0 (drag fusion disabled)",
    "unit": "-",
    "where": "src/modules/ekf2/params_drag.yaml"
   },
   {
    "name": "EKF2_DRAG_NOISE",
    "value": "2.5",
    "unit": "(m/s^2)^2",
    "where": "src/modules/ekf2/params_drag.yaml"
   }
  ],
  "assumptions": "Drag ∝ ρ·v² (m/β = ½ρ C_d A, β = m/(C_d A)); used only if bcoef > 1.0; innovation = pred − meas; gate 5 sigma; R_ACC = max(noise, 0.5).",
  "notes": "Symbolic definition: bluff_body_drag = −0.5·rho·cd·vel_rel_body_xy·‖vel_rel_body‖ (python/ekf_derivation/derivation.py:620-626), with cd = 1/β. Model predicts SPECIFIC FORCE (m/s^2), not force."
 },
 {
  "id": "px4ap-023",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "EKF2 multirotor rotor momentum drag, density-corrected (linear in airspeed)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/drag/drag_fusion.cpp",
  "lines": "81-84, 140",
  "code": "// correct rotor momentum drag for increase in required rotor mass flow with altitude\n// obtained from momentum disc theory\nconst float mcoef_corrrected = fmaxf(_params.ekf2_mcoef * sqrtf(rho / atmosphere::kAirDensitySeaLevelStandardAtmos),\n\t\t\t\t  0.f);\n... - rel_wind_body(axis_index) * mcoef_corrrected;",
  "formula": "a_mom,i = −K_m·√(ρ/ρ₀)·v_rel,i  (rotor-disc-parallel component), ρ₀ = 1.225",
  "variables": {
   "K_m": "EKF2_MCOEF momentum-drag coefficient [1/s]",
   "ρ": "air density [kg/m^3]",
   "ρ₀": "sea-level standard density kAirDensitySeaLevelStandardAtmos = 1.225 [kg/m^3]",
   "v_rel,i": "body-frame air-relative velocity component [m/s]"
  },
  "defaults": [
   {
    "name": "EKF2_MCOEF",
    "value": "0.15",
    "unit": "1/s",
    "where": "src/modules/ekf2/params_drag.yaml"
   }
  ],
  "assumptions": "Drag from rotor inflow momentum change, proportional to true airspeed parallel to the rotor disc and rotor mass flow; used if mcoef > 0.001. Symbolic: momentum_drag = −cm·vel_rel_body_xy (derivation.py:626).",
  "notes": "Total predicted drag = px4ap-022 + this term; the code comment cites momentum disc theory for the √ρ scaling (not derived in code)."
 },
 {
  "id": "px4ap-024",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "EKF2 elliptic interpolation of bluff-body ballistic coefficients X and Y",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/drag/drag_fusion.cpp",
  "lines": "108-115",
  "code": "// Interpolate between the X and Y bluff body drag coefficients using current relative velocity\n// This creates an elliptic drag distribution around the XY plane\nbcoef_inv(0) = Vector2f(bcoef_inv.emult(rel_wind_body_prev.xy()) / rel_wind_body_prev.xy().norm()).norm();\nbcoef_inv(1) = bcoef_inv(0);",
  "formula": "(1/β)_eff = ‖ [ (1/β_x)·v_x , (1/β_y)·v_y ] ‖ / ‖[v_x, v_y]‖  (applied to both axes)",
  "variables": {
   "β_x, β_y": "EKF2_BCOEF_X, EKF2_BCOEF_Y [kg/m^2]",
   "v_x, v_y": "body x,y air-relative velocity components at linearization point [m/s]"
  },
  "defaults": [],
  "assumptions": "Only when both bcoef_x and bcoef_y > 1; frozen at the pre-update linearization point.",
  "notes": "Result is the same effective 1/β for X and Y at that instant; direction-dependent drag area."
 },
 {
  "id": "px4ap-025",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "EKF2 drag measurement: IMU specific force in body X/Y",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/estimator_interface.cpp",
  "lines": "580-596 (and drag_fusion.cpp:134)",
  "code": "_drag_down_sampled.accelXY(0) += imu.delta_vel(0);\n_drag_down_sampled.accelXY(1) += imu.delta_vel(1);\n_drag_sample_time_dt += imu.delta_vel_dt;\n_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;\n// drag_fusion.cpp: const float mea_acc = drag_sample.accelXY(axis_index) - _state.accel_bias(axis_index);",
  "formula": "a_meas,i = (Σ Δv_i)/(Σ Δt) − b_a,i ,  i ∈ {x,y};  innovation_i = a_pred,i − a_meas,i",
  "variables": {
   "Δv": "IMU delta velocity (body) [m/s]",
   "b_a": "accel bias state [m/s^2]"
  },
  "defaults": [],
  "assumptions": "For a multirotor, thrust is along body −z, so the x,y accelerometer reading is the aerodynamic drag specific force; clipping samples discarded.",
  "notes": "Gravity does not appear (specific force)."
 },
 {
  "id": "px4ap-026",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 true-airspeed measurement model (wind states)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/python/ekf_derivation/derivation.py",
  "lines": "260-264",
  "code": "wind = sf.V3(state[\"wind_vel\"][0], state[\"wind_vel\"][1], 0.0)\nvel_rel = state[\"vel\"] - wind\nairspeed_pred = vel_rel.norm(epsilon=epsilon)\ninnov = airspeed_pred - airspeed",
  "formula": "V_TAS,pred = ‖[v_N − w_N, v_E − w_E, v_D]‖ ;  innovation = V_pred − V_meas",
  "variables": {
   "v": "vehicle NED velocity [m/s]",
   "w": "wind NE state, wind-to-direction [m/s]",
   "V_meas": "true airspeed from airspeed_validated [m/s]"
  },
  "defaults": [
   {
    "name": "EKF2_EAS_NOISE",
    "value": "1.4",
    "unit": "m/s",
    "where": "src/modules/ekf2/params_airspeed.yaml"
   },
   {
    "name": "EKF2_TAS_GATE",
    "value": "5.0",
    "unit": "SD",
    "where": "src/modules/ekf2/params_airspeed.yaml"
   },
   {
    "name": "EKF2_ARSP_THR",
    "value": "0.0 (airspeed fusion off unless > 0)",
    "unit": "m/s",
    "where": "src/modules/ekf2/params_airspeed.yaml"
   }
  ],
  "assumptions": "Air-relative speed = norm of (ground velocity − wind); zero vertical wind; pitot measures TAS (scale already applied upstream). Generated C++ (generated/compute_airspeed_innov_and_innov_var.h) not opened; model taken from the symbolic source.",
  "notes": "SIGN: innovation = pred − meas in EKF2; the standalone WindEstimator uses meas − pred (px4ap-036). Used in fixed-wing and VTOL transition (airspeed_fusion.cpp:98-100)."
 },
 {
  "id": "px4ap-027",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 airspeed observation variance scaled by EAS→TAS factor",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/airspeed/airspeed_fusion.cpp",
  "lines": "169-174",
  "code": "const float R = sq(math::constrain(_params.ekf2_eas_noise, 0.5f, 5.0f) *\n\t\t   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));",
  "formula": "R_TAS = ( clamp(σ_EAS,0.5,5) · clamp(eas2tas,0.9,10) )²",
  "variables": {
   "σ_EAS": "EKF2_EAS_NOISE [m/s]",
   "eas2tas": "TAS/CAS ratio [-]"
  },
  "defaults": [],
  "assumptions": "Equivalent-airspeed noise converts to TAS noise by multiplying by √(ρ₀/ρ).",
  "notes": ""
 },
 {
  "id": "px4ap-028",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 input: EAS-to-TAS factor from validated airspeed",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF2.cpp",
  "lines": "2269-2279, 2306",
  "code": "float cas2tas = 1.f;\nif (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s) && (airspeed_validated.calibrated_airspeed_m_s > FLT_EPSILON)) {\n\tcas2tas = airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s;\n}\n// fallback path: .eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,",
  "formula": "eas2tas = V_TAS / V_CAS  (CAS used as EAS)",
  "variables": {
   "V_TAS": "true airspeed [m/s]",
   "V_CAS": "calibrated airspeed [m/s]"
  },
  "defaults": [],
  "assumptions": "CAS ≈ EAS (compressibility ignored); fallback path scales TAS by ASPD_SCALE (line 2297).",
  "notes": ""
 },
 {
  "id": "px4ap-029",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 wind initialization from airspeed (horizontal flight, zero sideslip)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/python/ekf_derivation/derivation.py",
  "lines": "294-311",
  "code": "sideslip = sf.Symbol(\"beta\")\nwind = sf.V2(v_local[0] - airspeed * sf.cos(heading + sideslip), v_local[1] - airspeed * sf.sin(heading + sideslip))\n# Assume zero sideslip\nwind = wind.subs({sideslip: 0.0})",
  "formula": "w_N = v_N − V_TAS·cos(ψ+β),  w_E = v_E − V_TAS·sin(ψ+β), β=0",
  "variables": {
   "ψ": "heading (euler yaw) [rad]",
   "β": "sideslip [rad], assumed 0",
   "V_TAS": "true airspeed [m/s]"
  },
  "defaults": [
   {
    "name": "sideslip_var (init)",
    "value": "(15°)² via sq(radians(15))",
    "unit": "rad^2",
    "where": "airspeed_fusion.cpp:261"
   },
   {
    "name": "initial_wind_uncertainty",
    "value": "1.0",
    "unit": "m/s",
    "where": "src/modules/ekf2/EKF/common.h:341"
   }
  ],
  "assumptions": "Airspeed vector lies along body x (heading), no vertical velocity component; Jacobian wrt (v_N, v_E, ψ, β, V) propagates covariance (derivation.py:297-309).",
  "notes": "Called from Ekf::resetWindUsingAirspeed (airspeed_fusion.cpp:259-276). Inverse (velocity from airspeed and wind): v = w + V_TAS [cos ψ, sin ψ] (airspeed_fusion.cpp:284-285)."
 },
 {
  "id": "px4ap-030",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 wind from external wind speed/direction; direction convention conversion",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF2.cpp",
  "lines": "EKF2.cpp:610-612; derivation.py:319",
  "code": "// wind direction is given as azimuth where wind blows FROM\n// PX4 backend expects direction where wind blows TO\nconst float wind_direction_rad = wrap_pi(math::radians(vehicle_command.param3) + M_PI_F);\n// derivation: wind = sf.V2(wind_speed * sf.cos(wind_direction), wind_speed * sf.sin(wind_direction))",
  "formula": "θ_to = wrap_π(θ_from + π);  w = V_w·[cos θ_to, sin θ_to] (N,E)",
  "variables": {
   "θ_from": "MAVLink azimuth the wind blows FROM, from north [rad]",
   "θ_to": "direction the wind blows TO [rad]",
   "V_w": "wind speed [m/s]"
  },
  "defaults": [],
  "assumptions": "Azimuth measured from north toward east.",
  "notes": "SIGN CONVENTION: all EKF wind states are wind-to-direction velocities in NED. Also in: src/modules/ekf2/derivation.py."
 },
 {
  "id": "px4ap-031",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 wind-state process noise (scaled by vertical speed)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/covariance.cpp",
  "lines": "209-220",
  "code": "const float height_rate = _height_rate_lpf.update(_state.vel(2), imu_delayed.delta_vel_dt);\nconst float wind_vel_nsd_scaled = _params.ekf2_wind_nsd * (1.f + _params.wind_vel_nsd_scaler * fabsf(height_rate));\nconst float wind_vel_process_noise = sq(wind_vel_nsd_scaled) * dt;",
  "formula": "q_w = ( σ_w·(1 + 0.5·|ḣ_lpf|) )²·dt added to P(wind) while P < initial_wind_uncertainty²",
  "variables": {
   "σ_w": "EKF2_WIND_NSD [m/s^2/√Hz]",
   "ḣ_lpf": "low-pass filtered vertical velocity [m/s]",
   "wind_vel_nsd_scaler": "0.5 (common.h:343)"
  },
  "defaults": [
   {
    "name": "EKF2_WIND_NSD",
    "value": "0.05",
    "unit": "m/s^2/sqrt(Hz)",
    "where": "src/modules/ekf2/params_wind.yaml"
   }
  ],
  "assumptions": "Wind modeled as a random walk; wind changes faster during climb/descent.",
  "notes": "Applied only if P(i,i) < 1.0 m²/s²."
 },
 {
  "id": "px4ap-032",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 synthetic sideslip measurement model",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/python/ekf_derivation/derivation.py",
  "lines": "327-350",
  "code": "wind = sf.V3(state[\"wind_vel\"][0], state[\"wind_vel\"][1], 0.0)\nvel_rel = state[\"vel\"] - wind\nrelative_wind_body = state[\"quat_nominal\"].inverse() * vel_rel\nsideslip_pred = sf.atan2(relative_wind_body[1], relative_wind_body[0], epsilon)\ninnov = sideslip_pred - 0.0",
  "formula": "β_pred = atan2(v_rel,y^b, v_rel,x^b) ;  observation β = 0 ;  innovation = β_pred − 0",
  "variables": {
   "v_rel^b": "Rᵀ(v − [w_N, w_E, 0]) body frame [m/s]"
  },
  "defaults": [
   {
    "name": "EKF2_BETA_NOISE",
    "value": "0.3",
    "unit": "m/s (as listed in yaml; used as std dev in R = max(σ²,0.01²))",
    "where": "src/modules/ekf2/params_sideslip.yaml; sideslip_fusion.cpp:79"
   },
   {
    "name": "EKF2_BETA_GATE",
    "value": "5.0",
    "unit": "SD",
    "where": "src/modules/ekf2/params_sideslip.yaml"
   },
   {
    "name": "EKF2_FUSE_BETA",
    "value": "0",
    "unit": "-",
    "where": "src/modules/ekf2/params_sideslip.yaml"
   },
   {
    "name": "beta_avg_ft_us",
    "value": "150000",
    "unit": "us",
    "where": "src/modules/ekf2/EKF/common.h:433"
   }
  ],
  "assumptions": "Coordinated flight: lateral air-relative velocity in body frame is zero.",
  "notes": "Yaml unit 'm/s' for a quantity used as sideslip noise std (angle in rad) is inconsistent with the model; flagged. SIGN: β>0 when the air-relative vehicle velocity has +y (right) body component."
 },
 {
  "id": "px4ap-033",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Standalone WindEstimator: TAS measurement with scale state",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/wind_estimator/python/derivation.py",
  "lines": "43-62",
  "code": "vel_rel = sf.V3(v_local[0] - state[0], v_local[1] - state[1], v_local[2])\nairspeed_pred = vel_rel.norm(epsilon=epsilon) * state[2]\ninnov = airspeed - airspeed_pred",
  "formula": "V_pred = ‖[v_N − w_N, v_E − w_E, v_D]‖ · s ;  innovation = V_meas − V_pred",
  "variables": {
   "state": "[w_N, w_E, s]: wind NE [m/s] and TAS scale s [-]",
   "s": "airspeed scale state (1/s = airspeed calibration scale returned by get_tas_scale)"
  },
  "defaults": [
   {
    "name": "_tas_var",
    "value": "1.96 ((1.4 m/s)²)",
    "unit": "(m/s)^2",
    "where": "WindEstimator.hpp:129"
   },
   {
    "name": "kTasGate",
    "value": "4",
    "unit": "SD",
    "where": "WindEstimator.hpp:131"
   },
   {
    "name": "ASPD_TAS_NOISE",
    "value": "1.4",
    "unit": "m/s",
    "where": "src/modules/airspeed_selector/airspeed_selector_params.yaml"
   }
  ],
  "assumptions": "Raw TAS = (true air speed)/s. State [2] initial 1.0 (WindEstimator.hpp:116). get_tas_scale() returns 1/constrain(s,0.1,10) (WindEstimator.hpp:78).",
  "notes": "SIGN: innovation = meas − pred here but pred − meas in EKF2 (px4ap-026). Fusion limited to 10 Hz (WindEstimator.cpp:121)."
 },
 {
  "id": "px4ap-034",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Standalone WindEstimator: small-angle sideslip measurement",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/wind_estimator/python/derivation.py",
  "lines": "73-80",
  "code": "vel_rel = sf.V3(v_local[0] - state[0], v_local[1] - state[1], v_local[2])\nrelative_wind_body = quat_to_rot(q_att).T * vel_rel\n# Small angle approximation of side slip model\nbeta_pred = add_epsilon_sign(relative_wind_body[1] / relative_wind_body[0], relative_wind_body[0], epsilon)\ninnov = 0.0 - beta_pred",
  "formula": "β_pred ≈ v_rel,y^b / v_rel,x^b ;  innovation = 0 − β_pred",
  "variables": {
   "v_rel^b": "body-frame air-relative velocity [m/s]"
  },
  "defaults": [
   {
    "name": "kBetaVar",
    "value": "0.0225 ((0.15 rad)²)",
    "unit": "rad^2",
    "where": "WindEstimator.hpp:130"
   },
   {
    "name": "kBetaGate",
    "value": "1",
    "unit": "SD",
    "where": "WindEstimator.hpp:132"
   }
  ],
  "assumptions": "Small-angle; EKF2 uses atan2 (px4ap-032) instead of the ratio.",
  "notes": "SIGN: innovation = 0 − pred (EKF2: pred − 0)."
 },
 {
  "id": "px4ap-035",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Standalone WindEstimator: initialization of wind from TAS and groundspeed; no-airspeed fallback",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/wind_estimator/python/derivation.py",
  "lines": "99-115",
  "code": "wind = sf.V2(v_local[0] - airspeed * sm.cos(heading + sideslip), v_local[1] - airspeed * sm.sin(heading + sideslip))\n# Assume zero sideslip\nwind = wind.subs({sideslip: 0.0})",
  "formula": "w_N = v_N − V_TAS cos(ψ+β), w_E = v_E − V_TAS sin(ψ+β), β=0",
  "variables": {
   "ψ": "heading from quaternion Euler yaw [rad]"
  },
  "defaults": [
   {
    "name": "kInitialWindError",
    "value": "2.5",
    "unit": "m/s",
    "where": "WindEstimator.hpp:109"
   },
   {
    "name": "kInitialBetaErrorDeg",
    "value": "15.0",
    "unit": "deg",
    "where": "WindEstimator.hpp:111"
   },
   {
    "name": "kInitialHeadingErrorDeg",
    "value": "10.0",
    "unit": "deg",
    "where": "WindEstimator.hpp:113"
   }
  ],
  "assumptions": "Horizontal flight; same model as px4ap-029; without airspeed, wind = 0 with variance 2.5².",
  "notes": "Used in WindEstimator::initialise (WindEstimator.cpp:42-74)."
 },
 {
  "id": "px4ap-036",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Standalone WindEstimator: process noise, fast TAS-scale learning",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/wind_estimator/WindEstimator.cpp",
  "lines": "98-106",
  "code": "const float tas_psd_multiplier = (fabsf(_scale_init - 1.0f) < FLT_EPSILON && (time_now - _time_initialised < kTASScaleFastLearnTime)) ?\n\t\t\t\t kTASScalePSDMultiplier : 1.f;\nQk(INDEX_W_N, INDEX_W_N) = _wind_psd * dt;\nQk(INDEX_W_E, INDEX_W_E) = Qk(INDEX_W_N, INDEX_W_N);\nQk(INDEX_TAS_SCALE, INDEX_TAS_SCALE) = kTasScalePsd * tas_psd_multiplier * dt;",
  "formula": "Q = diag(σ_w²·dt, σ_w²·dt, q_s·m·dt), m = 100 for first 300 s if scale at default, else 1",
  "variables": {
   "σ_w²": "_wind_psd [(m/s^2)^2/Hz]",
   "q_s": "kTasScalePsd = 1e-8 [1/s^2/Hz]",
   "dt": "≥1 s covariance prediction step"
  },
  "defaults": [
   {
    "name": "_wind_psd",
    "value": "0.01 ((0.1 m/s^2/√Hz)²)",
    "unit": "m^2/s^4/Hz",
    "where": "WindEstimator.hpp:127"
   },
   {
    "name": "ASPD_WIND_NSD",
    "value": "0.1",
    "unit": "m/s^2/sqrt(Hz)",
    "where": "src/modules/airspeed_selector/airspeed_selector_params.yaml"
   },
   {
    "name": "kTASScaleFastLearnTime",
    "value": "300 s",
    "unit": "s",
    "where": "WindEstimator.hpp:142"
   }
  ],
  "assumptions": "Wind and scale random walks; covariance clamps: wind var ≤25, scale var ≤0.1 (WindEstimator.cpp:210-216).",
  "notes": "Different from EKF2_WIND_NSD default 0.05 (EKF2 uses vertical-speed scaling too)."
 },
 {
  "id": "px4ap-037",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "AirspeedValidator: IAS → CAS → TAS chain",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/AirspeedValidator.cpp",
  "lines": "189-194",
  "code": "_CAS = calc_CAS_from_IAS(_IAS, _CAS_scale_applied);\n_TAS = calc_TAS_from_CAS(_CAS, air_pressure_pa, air_temperature_celsius);",
  "formula": "V_CAS = V_IAS · scale ;  V_TAS = V_CAS · √(ρ₀/ρ(p,T))",
  "variables": {
   "scale": "_CAS_scale_applied = ASPD_SCALE_n (mode 0/1) or the validated scale (mode 2)",
   "ρ": "density from static pressure and temperature (px4ap-050)"
  },
  "defaults": [
   {
    "name": "ASPD_SCALE_APPLY",
    "value": "2",
    "unit": "-",
    "where": "src/modules/airspeed_selector/airspeed_selector_params.yaml"
   }
  ],
  "assumptions": "CAS ≈ EAS (no compressibility).",
  "notes": ""
 },
 {
  "id": "px4ap-038",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "AirspeedValidator: airspeed scale validation against GNSS ground speed (loop over course)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/AirspeedValidator.cpp",
  "lines": "125-152",
  "code": "const float course_over_ground_rad = matrix::wrap_2pi(atan2f(vI(1), vI(0)));\nconst float TAS_to_groundspeed_error_current = ground_speed_sum - TAS_sum * _CAS_scale_validated;\nconst float TAS_to_groundspeed_error_new = ground_speed_sum - TAS_sum * _wind_estimator.get_tas_scale();\nconst float new_scale_constrained = math::constrain(_wind_estimator.get_tas_scale(), _CAS_scale_validated - 0.05f,\n\t\t\t\t    _CAS_scale_validated + 0.05f);",
  "formula": "Σ_{12 course segments}(V_gs − s·V_TAS): accept new s if |error_new| < |error_current|; s moves ≤0.05 per accepted check",
  "variables": {
   "V_gs": "ground speed per 30° course segment [m/s]",
   "V_TAS": "raw true airspeed in that segment [m/s]",
   "s": "scale"
  },
  "defaults": [
   {
    "name": "SCALE_CHECK_SAMPLES",
    "value": "12",
    "unit": "-",
    "where": "AirspeedValidator.hpp:137"
   }
  ],
  "assumptions": "Averaging ground speed minus TAS over all headings cancels a constant wind (sum of V_gs = sum of s·V_TAS if no scale error); resets every 100 s.",
  "notes": "Uses the standalone wind estimator's scale as candidate."
 },
 {
  "id": "px4ap-039",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "AirspeedValidator: TAS innovation (air-relative speed) check",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/AirspeedValidator.cpp",
  "lines": "239-252",
  "code": "matrix::Vector2f wind_2d(_wind_estimator.get_wind());\nmatrix::Vector3f air_vel = vI - matrix::Vector3f {wind_2d(0), wind_2d(1), 0.f};\nconst float tas_innov = fabsf(_TAS - air_vel.norm());\n_aspd_innov_integ_state += dt_s * (tas_innov - _tas_innov_threshold); // integrate exceedance\n_innovations_check_failed = _aspd_innov_integ_state > _tas_innov_integ_threshold;",
  "formula": "e = |V_TAS − ‖v_gnd − w‖| ;  I += dt·(e − e_thr) when e > e_thr else I=0 ; fail if I > I_thr",
  "variables": {
   "v_gnd": "NED velocity [m/s]",
   "w": "wind estimate NE [m/s]"
  },
  "defaults": [
   {
    "name": "ASPD_FS_INNOV",
    "value": "5.0",
    "unit": "m/s",
    "where": "airspeed_selector_params.yaml"
   },
   {
    "name": "ASPD_FS_INTEG",
    "value": "10.0",
    "unit": "m",
    "where": "airspeed_selector_params.yaml"
   },
   {
    "name": "_tas_innov_threshold (header init)",
    "value": "1.0",
    "unit": "m/s",
    "where": "AirspeedValidator.hpp:161 (overridden by ASPD_FS_INNOV)"
   }
  ],
  "assumptions": "Wind uses same sign convention as EKF (air velocity = v − w).",
  "notes": ""
 },
 {
  "id": "px4ap-040",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "AirspeedValidator: load-factor check (lift ∝ V²)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/AirspeedValidator.cpp",
  "lines": "270-276",
  "code": "float max_lift_ratio = fmaxf(_CAS, 0.7f) / fmaxf(_airspeed_stall, 1.0f);\nmax_lift_ratio *= max_lift_ratio;\n_load_factor_ratio = 0.95f * _load_factor_ratio + 0.05f * (fabsf(accel_z) / 9.81f) / max_lift_ratio;\n_load_factor_check_failed = (_load_factor_ratio > 1.1f);",
  "formula": "n_max = (V_CAS / V_stall)²  (max lift ∝ V²);  ratio ← 0.95 ratio + 0.05·(|a_z|/9.81)/n_max;  fail if ratio > 1.1",
  "variables": {
   "a_z": "body z accelerometer [m/s^2]",
   "V_stall": "FW_AIRSPD_STALL (default header 8.0 m/s) [m/s]"
  },
  "defaults": [
   {
    "name": "_airspeed_stall (header init)",
    "value": "8.0",
    "unit": "m/s",
    "where": "AirspeedValidator.hpp:169"
   }
  ],
  "assumptions": "Maximum lift coefficient fixed, so the maximum load factor scales with CAS²/V_stall². Uses literal 9.81, not CONSTANTS_ONE_G.",
  "notes": "Ratio constrained to [0.25, 2.0]."
 },
 {
  "id": "px4ap-041",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "AirspeedValidator: first-principle check (throttle/pitch/airspeed derivative)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/AirspeedValidator.cpp",
  "lines": "321-327",
  "code": "const float high_throttle_threshold = math::min(throttle_trim + kHighThrottleDelta, _param_throttle_max);\nconst bool high_throttle = throttle_fw > high_throttle_threshold;\nconst bool pitching_down = _pitch_filtered.getState() < _param_psp_off;\nconst bool check_failing = _IAS_derivative.getState() < kIASDerivateThreshold && high_throttle && pitching_down;",
  "formula": "fail ⇔ dIAS/dt < 0.1 m/s² AND throttle > trim+0.05 AND pitch < pitch_offset",
  "variables": {
   "IAS derivative filter": "time constant 5 s, pitch filter 1.5 s (lines 314-315)"
  },
  "defaults": [
   {
    "name": "kHighThrottleDelta",
    "value": "0.05",
    "unit": "-",
    "where": "AirspeedValidator.hpp:139"
   },
   {
    "name": "kIASDerivateThreshold",
    "value": "0.1",
    "unit": "m/s^2",
    "where": "AirspeedValidator.hpp:141"
   },
   {
    "name": "ASPD_FP_T_WINDOW",
    "value": "2.0",
    "unit": "s",
    "where": "airspeed_selector_params.yaml"
   }
  ],
  "assumptions": "With high thrust and a nose-down attitude the energy balance requires accelerating.",
  "notes": "Qualitative energy argument."
 },
 {
  "id": "px4ap-042",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Indicated airspeed from differential pressure",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/airspeed/airspeed.cpp",
  "lines": "169-178",
  "code": "float calc_IAS(float differential_pressure)\n{\n\tif (differential_pressure > 0.0f) {\n\t\treturn sqrtf((2.0f * differential_pressure) / kAirDensitySeaLevelStandardAtmos);\n\t} else {\n\t\treturn -sqrtf((2.0f * fabsf(differential_pressure)) / kAirDensitySeaLevelStandardAtmos);",
  "formula": "V_IAS = sgn(Δp)·√(2|Δp| / ρ₀),  ρ₀ = 1.225 kg/m³",
  "variables": {
   "Δp": "pitot differential (dynamic) pressure [Pa]",
   "ρ₀": "kAirDensitySeaLevelStandardAtmos 1.225 [kg/m^3]"
  },
  "defaults": [],
  "assumptions": "Incompressible Bernoulli, q = ½ρV²; sign preserved for reverse flow.",
  "notes": ""
 },
 {
  "id": "px4ap-043",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "True airspeed from total and static pressure and temperature",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/airspeed/airspeed.cpp",
  "lines": "195-211",
  "code": "float density = getDensityFromPressureAndTemp(static_pressure, temperature_celsius);\nif (density < 0.0001f || !PX4_ISFINITE(density)) { density = kAirDensitySeaLevelStandardAtmos; }\nfloat pressure_difference = total_pressure - static_pressure;\nreturn sqrtf((2.0f * (pressure_difference)) / density);",
  "formula": "V_TAS = sgn(p_t − p_s)·√(2|p_t − p_s| / ρ),  ρ = p_s / (R (T_C + 273.15))",
  "variables": {
   "p_t, p_s": "total and static pressure [Pa]",
   "ρ": "air density [kg/m^3]",
   "R": "kAirGasConstant 287.1 [J/(kg K)]"
  },
  "defaults": [],
  "assumptions": "Incompressible; falls back to ρ₀ if density invalid.",
  "notes": ""
 },
 {
  "id": "px4ap-044",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Calibrated ↔ true airspeed conversion",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/airspeed/airspeed.cpp",
  "lines": "180-188, 213-220",
  "code": "return speed_calibrated * sqrtf(kAirDensitySeaLevelStandardAtmos / getDensityFromPressureAndTemp(pressure_ambient, temperature_celsius));\nreturn speed_true * sqrtf(air_density / kAirDensitySeaLevelStandardAtmos);\nreturn speed_calibrated * sqrtf(kAirDensitySeaLevelStandardAtmos / air_density);",
  "formula": "V_TAS = V_CAS·√(ρ₀/ρ) ;  V_CAS = V_TAS·√(ρ/ρ₀) ;  V_CAS = V_IAS · scale",
  "variables": {
   "ρ": "ambient density [kg/m^3]",
   "ρ₀": "1.225 kg/m^3"
  },
  "defaults": [
   {
    "name": "default temperature if non-finite",
    "value": "15",
    "unit": "deg C",
    "where": "airspeed.cpp:55, 183"
   }
  ],
  "assumptions": "CAS taken equal to EAS (ρ scaling only, no compressibility correction).",
  "notes": "calc_CAS_from_IAS is speed_indicated * scale (lines 190-193)."
 },
 {
  "id": "px4ap-045",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Pitot/SDP3x sensor and tube pressure-drop corrections (calc_IAS_corrected)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/airspeed/airspeed.cpp",
  "lines": "80-109, 159-166",
  "code": "const float dp_corr = dp * 96600.0f / pressure_ambient;\nfloat flow_SDP33 = (300.805f - 300.878f / (0.00344205f * powf(dp_corr, 0.68698f) + 1.0f)) * 1.29f / rho_air;\ndp_pitot = (0.0032f * flow_SDP33 * flow_SDP33 + 0.0123f * flow_SDP33 + 1.0f) * 1.29f / rho_air;\nconst float dp_tube = (flow_SDP33 * 0.674f) / 450.0f * tube_len * rho_air / 1.29f;\ndv = 0.125f * flow_SDP33;\nconst float airspeed_uncorrected = sqrtf(2.0f * dp_tot / kAirDensitySeaLevelStandardAtmos);",
  "formula": "dp_corr = dp·96600/p;  Q = (300.805 − 300.878/(0.00344205·dp_corr^0.68698 + 1))·1.29/ρ;  dp_pitot = (0.0032Q² + 0.0123Q + 1)·1.29/ρ;  dp_tube = 0.674·Q/450·L·ρ/1.29;  dv = 0.125 Q;  dp_tot = dp_corr + dp_tube + dp_pitot;  V = √(2 dp_tot/ρ₀) + dv",
  "variables": {
   "dp": "|differential pressure| [Pa]",
   "p": "ambient pressure [Pa]",
   "ρ": "air density from p and T [kg/m^3]",
   "Q": "flow through SDP33 sensor [units of source empirical fit]",
   "L": "tube_len [source units]"
  },
  "defaults": [],
  "assumptions": "Empirical fits for a specific sensor + 1.5 mm tube (comment line 74-75); applies for sensor model SDP3X with pitot or no-pitot compensation. Units of Q and L not documented in source.",
  "notes": "Active only when the airspeed sensor model SDP3X and compensation model are selected (see line 66-157)."
 },
 {
  "id": "px4ap-046",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Tube pressure-loss (Poiseuille) airspeed compensation",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/airspeed/airspeed.cpp",
  "lines": "112-141",
  "code": "const float d_tubePow4 = powf(tube_dia_mm * 1e-3f, 4);\nconst float denominator = M_PI_F * d_tubePow4 * rho_air * dp;\nconst float viscosity = (18.205f + 0.0484f * (temperature_celsius - 20.0f)) * 1e-6f;\neps = -64.0f * tube_len * viscosity * 4.79f * 1e-7f * (sqrtf(1.0f + 8.0f * dp / 59.3319f) - 1.0f) / denominator;\ndp_tot = dp / (1.0f + eps);",
  "formula": "μ = (18.205 + 0.0484(T−20))·1e-6 Pa·s;  ε = −64 L μ (4.79e-7)(√(1 + 8 dp/59.3319) − 1) / (π d⁴ ρ dp);  dp_tot = dp/(1+ε)",
  "variables": {
   "d": "tube diameter [m] (tube_dia_mm·1e-3)",
   "L": "tube length [m]",
   "μ": "dynamic viscosity of air [Pa s]",
   "T": "temperature [deg C]",
   "ε": "fractional pressure loss [-]",
   "4.79e-7": "mass-flow constant of sensor (source comment)"
  },
  "defaults": [],
  "assumptions": "Laminar Poiseuille flow in a tube; sensor constant 59.3 Pa where linear and quadratic dp–flow contributions are equal (source comment cites ~59.5); eps ignored if |ε| ≥ 1.",
  "notes": "Linear temperature fit of viscosity around 20 deg C."
 },
 {
  "id": "px4ap-047",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Airspeed selector: ground-minus-wind airspeed",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/airspeed_selector_main.cpp",
  "lines": "753-758",
  "code": "const float TAS_north = _vehicle_local_position.vx - _wind_estimate_sideslip.windspeed_north;\nconst float TAS_east = _vehicle_local_position.vy - _wind_estimate_sideslip.windspeed_east;\nconst float TAS_down = _vehicle_local_position.vz; // no wind estimate in z\n_ground_minus_wind_TAS = sqrtf(TAS_north * TAS_north + TAS_east * TAS_east + TAS_down * TAS_down);\n_ground_minus_wind_CAS = calc_calibrated_from_true_airspeed(_ground_minus_wind_TAS, _vehicle_air_data.rho);",
  "formula": "V_TAS = ‖v − [w_N, w_E, 0]‖;  V_CAS = V_TAS√(ρ/ρ₀)",
  "variables": {
   "w": "sideslip-only wind estimate [m/s]",
   "ρ": "vehicle_air_data.rho [kg/m^3]"
  },
  "defaults": [
   {
    "name": "ASPD_WERR_THR",
    "value": "2.0",
    "unit": "m/s",
    "where": "airspeed_selector_params.yaml"
   }
  ],
  "assumptions": "Wind estimate from the sideslip-only WindEstimator instance; used only if wind uncertainty (√(var_N+var_E)) < threshold.",
  "notes": "Sign: air velocity = ground velocity − wind."
 },
 {
  "id": "px4ap-048",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Airspeed selector: synthetic airspeed from throttle (piecewise linear)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/airspeed_selector/airspeed_selector_main.cpp",
  "lines": "947-972",
  "code": "} else if (throttle < _param_fw_thr_trim.get() && _param_fw_thr_aspd_min.get() > 0.f) {\n\tsynthetic_airspeed = interpolate(throttle, _param_fw_thr_aspd_min.get(), _param_fw_thr_trim.get(), _param_fw_airspd_min.get(), _param_fw_airspd_trim.get());\n} else if (throttle > _param_fw_thr_trim.get() && _param_fw_thr_aspd_max.get() > 0.f) {\n\tsynthetic_airspeed = interpolate(throttle, _param_fw_thr_trim.get(), _param_fw_thr_aspd_max.get(), _param_fw_airspd_trim.get(), _param_fw_airspd_max.get());",
  "formula": "V_syn = lerp over throttle: (thr_aspd_min→V_min, thr_trim→V_trim, thr_aspd_max→V_max); V_trim outside level flight",
  "variables": {
   "throttle": "filtered throttle [0..1]",
   "V_min, V_trim, V_max": "FW_AIRSPD_MIN, TRIM, MAX [m/s]"
  },
  "defaults": [
   {
    "name": "FW_THR_ASPD_MIN / MAX",
    "value": "0.0 / 0.0 (disabled)",
    "unit": "-",
    "where": "src/lib/fw_performance_model/performance_model_params.yaml:25,38"
   },
   {
    "name": "FW_THR_TRIM",
    "value": "0.6",
    "unit": "-",
    "where": "performance_model_params.yaml:11"
   }
  ],
  "assumptions": "Steady level flight: throttle ↔ airspeed monotonic linear (parasite-drag-style map).",
  "notes": "Used as the synthetic airspeed source (lines 897-910); outside LEVEL phase the trim airspeed is returned."
 },
 {
  "id": "px4ap-049",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Barometric (pressure) altitude from static pressure (QNH)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/sensors/vehicle_air_data/VehicleAirData.cpp",
  "lines": "VehicleAirData.cpp:303-304; atmosphere.cpp:70-71",
  "code": "const float pressure_sealevel_pa = _param_sens_baro_qnh.get() * 100.f;\nconst float altitude = getAltitudeFromPressure(pressure_pa, pressure_sealevel_pa);\n// atmosphere.cpp:\nreturn (((powf(pressure_ratio, (-(kTempGradient * kAirGasConstant) / CONSTANTS_ONE_G))) * kTempRefKelvin) -\n\tkTempRefKelvin) / kTempGradient;",
  "formula": "h = T₀·[(p/p₀)^(−aR/g) − 1]/a ;  a = −0.0065 K/m, R = 287.1 J/(kg K), g = 9.80665, T₀ = 288.15 K  (exponent ≈ 0.1903)",
  "variables": {
   "p": "averaged barometer pressure [Pa]",
   "p₀": "SENS_BARO_QNH × 100 [Pa]",
   "h": "baro altitude baro_alt_meter [m]"
  },
  "defaults": [
   {
    "name": "SENS_BARO_QNH",
    "value": "1013.25",
    "unit": "hPa",
    "where": "src/modules/sensors/vehicle_air_data/params.yaml"
   },
   {
    "name": "kPressRefSeaLevelPa",
    "value": "101325",
    "unit": "Pa",
    "where": "src/lib/atmosphere/atmosphere.h:57"
   },
   {
    "name": "kTempRefKelvin",
    "value": "288.15",
    "unit": "K",
    "where": "atmosphere.h:55"
   },
   {
    "name": "kTempGradient",
    "value": "−0.0065",
    "unit": "K/m",
    "where": "atmosphere.h:56"
   },
   {
    "name": "kAirGasConstant",
    "value": "287.1",
    "unit": "J/(kg K)",
    "where": "atmosphere.h:53"
   }
  ],
  "assumptions": "ICAO troposphere, constant lapse rate, hydrostatic balance; hypsometric equation. Formula lives in lib/atmosphere (covered by another helper); listed here because vehicle_air_data uses it.",
  "notes": "Uses R = 287.1, not the textbook 287.05. Also in: src/lib/atmosphere/atmosphere.cpp. Quote condensed from several code lines."
 },
 {
  "id": "px4ap-050",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Air density from pressure and temperature (published as vehicle_air_data.rho)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/sensors/vehicle_air_data/VehicleAirData.cpp",
  "lines": "VehicleAirData.cpp:301-307; atmosphere.cpp:47",
  "code": "const float ambient_temperature = AirTemperatureUpdate(temperature_baro, temperature_source, time_now_us);\nconst float air_density = getDensityFromPressureAndTemp(pressure_pa, ambient_temperature);\n// atmosphere.cpp:\nreturn (pressure_pa / (kAirGasConstant * (temperature_celsius - kAbsoluteNullCelsius)));",
  "formula": "ρ = p / (R·(T_C + 273.15))",
  "variables": {
   "p": "static pressure [Pa]",
   "T_C": "ambient temperature [deg C]",
   "R": "287.1 J/(kg K)"
  },
  "defaults": [
   {
    "name": "DEFAULT_TEMPERATURE_CELSIUS",
    "value": "15",
    "unit": "deg C",
    "where": "VehicleAirData.hpp:48"
   },
   {
    "name": "TEMPERATURE_MIN/MAX_CELSIUS",
    "value": "−60 / 60",
    "unit": "deg C",
    "where": "VehicleAirData.hpp:49-50"
   }
  ],
  "assumptions": "Ideal gas, dry air; temperature = external barometer or differential-pressure sensor temp if fresh (<1 s), else 15 deg C, clamped to [−60,60].",
  "notes": "At p = 101325 Pa, 15 deg C this gives 1.2248, vs kAirDensitySeaLevelStandardAtmos = 1.225 used for IAS and CAS/TAS; small inconsistency (≈0.02%). Also in: src/lib/atmosphere/atmosphere.cpp."
 },
 {
  "id": "px4ap-051",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "EKF2 baro compensation for dynamic (static-port) pressure error",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp",
  "lines": "215-236",
  "code": "const Vector3f velocity_earth = _state.vel - vel_imu_rel_body_ned;\nconst Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;\nconst Vector3f airspeed_body = _state.quat_nominal.rotateVectorInverse(airspeed_earth);\nconst Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.ekf2_aspd_max));\nconst float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));\nreturn baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);",
  "formula": "Δp_static = ½ρ·Σ_i K_i·min(v_b,i², V_max²) ;  h_corr = h_baro + Δp_static/(ρ·g₀) ;  v_b = Rᵀ(v_imu − w)",
  "variables": {
   "K_i": "EKF2_PCOEF_XP/XN/YP/YN/Z (sign chosen by sign of v_b,i) [-]",
   "V_max": "EKF2_ASPD_MAX [m/s]",
   "v_imu": "vehicle velocity minus R(ω × r_imu), IMU lever-arm correction [m/s]"
  },
  "defaults": [
   {
    "name": "EKF2_PCOEF_XP/XN/YP/YN/Z",
    "value": "0.0 (compensation off)",
    "unit": "-",
    "where": "src/modules/ekf2/params_barometer.yaml"
   },
   {
    "name": "EKF2_ASPD_MAX",
    "value": "20.0",
    "unit": "m/s",
    "where": "src/modules/ekf2/params_barometer.yaml"
   }
  ],
  "assumptions": "Static-pressure error ∝ dynamic pressure with body-direction-dependent coefficient; hydrostatic conversion uses sea-level g and local density; only when wind state active and horizontal position valid.",
  "notes": "Sign: positive K means measured pressure error raises indicated altitude after correction."
 },
 {
  "id": "px4ap-052",
  "group": 15,
  "group_name": "estimators",
  "quantity": "EKF2 baro height measurement and bias model",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp",
  "lines": "83-88",
  "code": "// vertical position innovation - baro measurement has opposite sign to earth z axis\nupdateVerticalPositionAidStatus(aid_src,\n\t\t\t\tbaro_sample.time_us,\n\t\t\t\t-(measurement - bias_est.getBias()),      // observation\n\t\t\t\tmeasurement_var + bias_est.getBiasVar(),  // observation variance\n\t\t\t\tmath::max(_params.ekf2_baro_gate, 1.f));",
  "formula": "z_obs = −(h_baro − b_baro) [m, NED down];  σ² = σ_baro² + Var(b)",
  "variables": {
   "h_baro": "baro altitude (up positive) [m]",
   "b_baro": "baro bias state of HeightBiasEstimator [m]"
  },
  "defaults": [
   {
    "name": "EKF2_BARO_NOISE",
    "value": "3.5",
    "unit": "m",
    "where": "src/modules/ekf2/params_barometer.yaml"
   },
   {
    "name": "EKF2_BARO_GATE",
    "value": "5.0",
    "unit": "SD",
    "where": "src/modules/ekf2/params_barometer.yaml"
   }
  ],
  "assumptions": "Baro altitude = true altitude + slowly varying bias.",
  "notes": "NED sign flip: observation is down-positive."
 },
 {
  "id": "px4ap-053",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "EKF2 ground-effect (rotor wash) deadzone on baro innovations",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp",
  "lines": "90-105",
  "code": "// Compensate for positive static pressure transients (negative vertical position innovations)\n// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.\nif (_control_status.flags.gnd_effect && (_params.ekf2_gnd_eff_dz > 0.f)) {\n\tconst float deadzone_end = deadzone_start + _params.ekf2_gnd_eff_dz;\n\tif (aid_src.innovation <= -deadzone_end) { aid_src.innovation += deadzone_end; } else { aid_src.innovation = -deadzone_start; }",
  "formula": "if gnd_effect: innov ← innov + D if innov ≤ −D; innov ← 0 if −D < innov < 0 (D = EKF2_GND_EFF_DZ, start = 0)",
  "variables": {
   "D": "EKF2_GND_EFF_DZ deadzone width [m]"
  },
  "defaults": [
   {
    "name": "EKF2_GND_EFF_DZ",
    "value": "4.0",
    "unit": "m",
    "where": "src/modules/ekf2/params_barometer.yaml"
   },
   {
    "name": "EKF2_GND_MAX_HGT",
    "value": "0.5",
    "unit": "m",
    "where": "src/modules/ekf2/params_barometer.yaml"
   }
  ],
  "assumptions": "Near the ground rotor wash raises static pressure, making baro read low (negative innovation); modeled as a one-sided deadzone, not a pressure model.",
  "notes": "Empirical deadzone."
 },
 {
  "id": "px4ap-054",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Control-allocator rotor effectiveness: thrust and moment per unit actuator output",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp",
  "lines": "156-203",
  "code": "// Compute thrust generated by this rotor\nmatrix::Vector3f thrust = ct * axis;\n// Compute moment generated by this rotor\nmatrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;\neffectiveness(j, i + actuator_start_index) = moment(j);\neffectiveness(j + 3, i + actuator_start_index) = thrust(j);",
  "formula": "B_i = [ m_i ; t_i ],  t_i = c_t·a_i ,  m_i = c_t·(r_i × a_i) − c_t·k_m·a_i  (rows: roll,pitch,yaw,Fx,Fy,Fz in body FRD)",
  "variables": {
   "a_i": "unit thrust axis of rotor i (normalized; default (0,0,−1) = up) [-]",
   "r_i": "rotor position relative to CG, CA_ROTORn_PX/PY/PZ [m]",
   "c_t": "CA_ROTORn_CT thrust coefficient [normalized thrust per unit output]",
   "k_m": "CA_ROTORn_KM moment ratio, torque = km·thrust; + for CCW rotor, − for CW [m]"
  },
  "defaults": [
   {
    "name": "CA_ROTORn_CT",
    "value": "6.5",
    "unit": "-",
    "where": "src/modules/control_allocator/module.yaml:220"
   },
   {
    "name": "CA_ROTORn_KM",
    "value": "0.05",
    "unit": "m (torque = KM·thrust)",
    "where": "module.yaml:236"
   },
   {
    "name": "CA_ROTORn_AZ",
    "value": "-1.0",
    "unit": "-",
    "where": "module.yaml:205"
   }
  ],
  "assumptions": "Thrust ∝ actuator output (linear in the allocation; nonlinearity is handled by THR_MDL_FAC, px4ap-066); thrust along rotor axis; reaction torque opposite spin direction; no rotor-flapping, gyroscopic, or induced inflow effects. Rotors with |c_t|<ε or invalid axis are skipped.",
  "notes": "CONTRADICTION: module.yaml:211 documents 'Thrust = CT * u^2' but the matrix is linear in u. The 'normalized' scale is set by later column-normalization (px4ap-058). Yaw sign: with axis (0,0,−1), yaw moment = +c_t·k_m."
 },
 {
  "id": "px4ap-055",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Fixed rotor axes and propeller-torque suppression for fixed-wing / VTOL",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp",
  "lines": "Rotors.cpp:104-110, 177-187; FixedWing.cpp:53; StandardVTOL.cpp:53",
  "code": "case AxisConfiguration::FixedForward:\n\taxis = Vector3f(1.f, 0.f, 0.f);\ncase AxisConfiguration::FixedUpwards:\n\taxis = Vector3f(0.f, 0.f, -1.f);\nif (geometry.propeller_torque_disabled) { km = 0.f; }\nbool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;",
  "formula": "FW pusher axis a = (1,0,0), k_m = 0; Standard VTOL: k_m = 0 for non-upward rotors (a_z ≥ −0.5 or |a_x|,|a_y| ≥ 0.1)",
  "variables": {
   "a": "rotor thrust axis, body FRD [-]",
   "k_m": "torque ratio [-]"
  },
  "defaults": [],
  "assumptions": "Fixed-wing propeller reaction torque is neglected in the allocation (yaw handled by control surfaces); VTOL pusher torque neglected.",
  "notes": "VTOL standard: pusher motors in matrix 0 have km zeroed; fixed-wing: _rotors.enablePropellerTorque(false). Quantity used by VTOL and fixed-wing configurations. Also in: src/modules/control_allocator/VehicleActuatorEffectiveness/FixedWing.cpp; src/modules/control_allocator/VehicleActuatorEffectiveness/StandardVTOL.cpp."
 },
 {
  "id": "px4ap-056",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Tiltrotor / tilt-servo rotor axis from tilt angle",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp",
  "lines": "243-255",
  "code": "const float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (collective_tilt_control + 1.f) / 2.f);\nconst float tilt_direction = math::radians((float)tilt.tilt_direction);\n_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);\n// tiltedAxis: Vector3f axis{0.f, 0.f, -1.f}; return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;",
  "formula": "θ = θ_min + (θ_max − θ_min)(u_c + 1)/2 ;  a = R_euler(0, −θ, ψ_dir)·[0,0,−1]ᵀ",
  "variables": {
   "θ": "tilt angle (0 = up) [rad]",
   "ψ_dir": "tilt direction (CA_SV_TL*_TD) [rad]",
   "u_c": "collective tilt control in [−1,1]"
  },
  "defaults": [],
  "assumptions": "Instantaneous servo position equals command; rotor thrust axis follows tilt angle without servo dynamics.",
  "notes": "VTOL tiltrotor (MCTilt effectiveness file not opened). For tiltrotors, 3D thrust disabled in the matrix: thrust x,y zeroed and F_z = −c_t (ActuatorEffectivenessRotors.cpp:216-218), used with collective tilt as separate control."
 },
 {
  "id": "px4ap-057",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Pseudo-inverse control allocation",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp",
  "lines": "PseudoInverse.cpp:69, 192; ControlAllocation.cpp:61-63",
  "code": "matrix::geninv(_effectiveness, _mix);\n_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);\n// ControlAllocation.cpp:\n_actuator_trim = actuator_trim + linearization_point_clipped;\n_control_trim = _effectiveness * linearization_point_clipped;",
  "formula": "u = u_trim + M·(c − c_trim),  M = pinv(B)·diag(1/scale) ;  c_trim = B·u_lin ;  c = [τ_x τ_y τ_z F_x F_y F_z]ᵀ",
  "variables": {
   "B": "effectiveness matrix 6×N (px4ap-054)",
   "M": "mixing matrix (normalized pseudo-inverse)",
   "u": "actuator setpoint [normalized]",
   "c": "control setpoint (torque, thrust) [normalized]"
  },
  "defaults": [],
  "assumptions": "Linear actuator model; least-squares minimum-norm solution; setpoint clipped to actuator limits (clipActuatorSetpoint) afterwards; allocation matrix per airframe is static unless tilt/flight phase updates.",
  "notes": "geninv = generalized (Moore-Penrose) inverse from matrix library. Desaturation is performed by ControlAllocationSequentialDesaturation (px4ap-060) for the multirotor method. Also in: src/lib/control_allocation/control_allocation/ControlAllocation.cpp."
 },
 {
  "id": "px4ap-058",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Mixing-matrix normalization scales (torque and thrust axes)",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp",
  "lines": "88-151",
  "code": "roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));\n_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);\n_control_allocation_scale(2) = _mix.col(2).max();\n_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;",
  "formula": "s_roll = s_pitch = max( √(‖M₀‖²/(n₀/2)), √(‖M₁‖²/(n₁/2)) );  s_yaw = max(M₂);  s_thrust,k = mean of |M_ik| over actuators with nonzero thrust effect",
  "variables": {
   "M_j": "column j of the pseudo-inverse mix",
   "n₀, n₁": "number of actuators with |M|>1e-3 for roll, pitch"
  },
  "defaults": [],
  "assumptions": "Output: unit torque setpoint ≈ unit actuator authority; the normalization only rescales columns, so rate-controller gains are airframe-independent.",
  "notes": "Entries |M|<1e-3 are set to 0 (lines 172-180). In this commit, dropDependentAxes (lines 195-253) zeroes lower-priority axes (priority THRUST_Z, ROLL, PITCH, THRUST_X, THRUST_Y, YAW) whose rows are linearly dependent; kMinAxisIndependence = 1e-2 (hpp:89)."
 },
 {
  "id": "px4ap-059",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Actuator slew-rate limiting",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/control_allocation/control_allocation/ControlAllocation.cpp",
  "lines": "143-153",
  "code": "const float delta_sp_max = dt * (_actuator_max(i) - _actuator_min(i)) / _actuator_slew_rate_limit(i);\nconst float delta_sp = input - previous;\nif (delta_sp > delta_sp_max) { output = previous + delta_sp_max; }\nelse if (delta_sp < -delta_sp_max) { output = previous - delta_sp_max; }",
  "formula": "|Δu| ≤ dt·(u_max − u_min)/T_slew  (T_slew from CA_Rn_SLEW / CA_SVn_SLEW: time for full range)",
  "variables": {
   "T_slew": "slew parameter as used in the formula (seconds for full range; unit not stated in code) [s]"
  },
  "defaults": [],
  "assumptions": "Models a rate limit on the command, not motor dynamics; NaN (motor off) treated as zero then restored.",
  "notes": "This is the only first-order-like actuator dynamics in the scanned allocator; no motor lag model found."
 },
 {
  "id": "px4ap-060",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Sequential desaturation (airmode) of actuator setpoints",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp",
  "lines": "67-120, 131-139",
  "code": "float gain = computeDesaturationGain(desaturation_vector, actuator_sp);\nactuator_sp(i) += gain * desaturation_vector(i);\ngain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);\nactuator_sp(i) += gain * desaturation_vector(i);\n... k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);\nreturn k_min + k_max;",
  "formula": "u ← u + k·d (then u ← u + 0.5·k'·d),  k = k_min + k_max,  k_i = (u_lim,i − u_i)/d_i for violated actuators with |d_i| ≥ 0.2;  d = thrust-axis (Z) or yaw column of the mix",
  "variables": {
   "d": "desaturation vector (thrust_z or yaw column of M)",
   "u_lim": "violated bound"
  },
  "defaults": [],
  "assumptions": "Preserves roll/pitch torque by shifting collective thrust (airmode); yaw desaturated last or prioritized below roll/pitch.",
  "notes": "Heuristic mixing logic, kept because it determines the realized torque/thrust."
 },
 {
  "id": "px4ap-061",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Control surface effectiveness: torque per normalized deflection (linear)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp",
  "lines": "130-136, 216-221",
  "code": "for (int n = 0; n < 3; ++n) { param_get(_param_handles[i].torque[n], &torque(n)); }\nint actuator_idx = configuration.addActuator(ActuatorType::SERVOS, _params[i].torque, Vector3f{});\nconfiguration.trim[configuration.selected_matrix](actuator_idx) = _params[i].trim;",
  "formula": "B_surface = [ (τ_r, τ_p, τ_y) ; (0,0,0) ] ;  roll/pitch/yaw torque_cmd = Σ_j B_j δ_j + trim",
  "variables": {
   "τ": "CA_SV_CSn_TRQ_R/P/Y: normalized torque per unit deflection [-]",
   "δ": "normalized servo command in [−1,1]",
   "trim": "CA_SV_CSn_TRIM [-]"
  },
  "defaults": [
   {
    "name": "CA_SV_CSn_TRQ_R/P/Y",
    "value": "0.0",
    "unit": "-",
    "where": "src/modules/control_allocator/module.yaml:296-322"
   }
  ],
  "assumptions": "Surface moment linear in deflection and independent of airspeed inside the allocator (airspeed scaling is applied upstream, px4ap-069 and px4ap-070). Flaps, spoilers, airbrakes, steering wheel have torque forced to 0 (lines 183-208).",
  "notes": "For standard VTOL and tiltrotor surfaces are in allocation matrix 1 (fixed-wing mode)."
 },
 {
  "id": "px4ap-062",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Flap and spoiler deflection mapping",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp",
  "lines": "227-258",
  "code": "// map [0, 1] to [-1, 1] for pure flaps\nactuator_sp(i + first_actuator_idx) += (_flaps_setpoint_with_slewrate.getState() * 2.f - 1.f) * _params[i].scale_flap;\n// map [0, 1] to [0, 1] for flaperons (ailerons deflected down)\nactuator_sp(i + first_actuator_idx) += _flaps_setpoint_with_slewrate.getState() * _params[i].scale_flap;",
  "formula": "δ_flap += (2f − 1)·s_flap (pure flaps) or f·s_flap (flaperons); δ_spoil similarly with s_spoil",
  "variables": {
   "f": "flap setpoint [0..1]",
   "s_flap, s_spoil": "CA_SVn_FLAP, CA_SVn_SPOIL scales [-]"
  },
  "defaults": [],
  "assumptions": "Deflection linear in flap/spoiler command; slew-rate limited via CA_FLAP_SLEW.",
  "notes": "Aerodynamic effect of flaps on lift/drag is not modeled in the allocator (airspeed scaling: FW_AIRSPD_FLP_SC in performance model, px4ap-078)."
 },
 {
  "id": "px4ap-063",
  "group": 9,
  "group_name": "rotation",
  "quantity": "Helicopter swashplate mixing and collective/throttle curves",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.cpp",
  "lines": "158-184, 207-211",
  "code": "const float throttle = (math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.throttle_curve) + rpm_control_output) * spoolup_progress;\nconst float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);\nfloat roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;\nfloat pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;\nactuator_sp(...) = collective_pitch + control_sp(ControlAxis::PITCH) * pitch_coeff - control_sp(ControlAxis::ROLL) * roll_coeff + trim;\nreturn _geometry.inverse_max_servo_throw * asinf(servo_height);",
  "formula": "servo_i = c + τ_p·cos α_i·L_i − τ_r·sin α_i·L_i + trim_i ;  throttle = interp(thrust_z; curve)·spoolup ;  servo linearization: u = asin(h_max·x)/θ_max",
  "variables": {
   "c": "collective pitch from pitch curve [-]",
   "α_i": "swashplate servo angle [rad]",
   "L_i": "arm length [-]",
   "x": "normalized servo command"
  },
  "defaults": [],
  "assumptions": "Swashplate servo deflection ∝ cyclic·arm geometry; collective and throttle are interpolated tables (CA_HELI_* parameters), spool-up progress = time since arming / COM_SPOOLUP_TIME (lines 235-236).",
  "notes": "Helicopter isn't multirotor strictly, listed under 'multirotor' for lack of a category. Yaw: actuator_sp(1) = yaw·sign + |c − offset|·scale + throttle·scale (lines 166-168). Coaxial variant not opened. Quote condensed from several code lines."
 },
 {
  "id": "px4ap-064",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Tilt-servo yaw/pitch torque sign from rotor geometry",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTilts.cpp",
  "lines": "114-132",
  "code": "Vector3f rotated_pos = Dcmf{Eulerf{0.f, 0.f, -tilt_direction}} * geometry.rotors[i].position;\nif (rotated_pos(1) < -0.01f) { _torque[tilt_index](2) = 1.f; }\nelse if (rotated_pos(1) > 0.01f) { _torque[tilt_index](2) = -1.f; }\n_torque[tilt_index](1) = tilting_forwards ? -1.f : 1.f;",
  "formula": "yaw effectiveness sign = −sign(y of rotor position rotated by −tilt direction); pitch sign = −1 if tilting forward (direction <90° or >270°)",
  "variables": {
   "tilt_direction": "tilt direction angle [deg]"
  },
  "defaults": [],
  "assumptions": "Tilting a rotor sideways produces yaw by differential lateral thrust: ±1 normalized effectiveness (sign only, magnitude not physical).",
  "notes": "VTOL tiltrotor; magnitude of tilt-servo effectiveness is a unit placeholder; pitch disabled in tiltrotor (ActuatorEffectivenessTiltrotorVTOL.cpp:97). Quote condensed from several code lines."
 },
 {
  "id": "px4ap-065",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Reversible-motor lower limit in actuator range (motor failure recovery)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator/ControlAllocator.cpp",
  "lines": "527-537",
  "code": "if (_runtime_reversible_bitmask & (1u << actuator_type_idx)) {\n\t// Failure recovery: a forward propeller driven in reverse makes only a fraction of its forward thrust\n\tminimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -_param_ca_rev_thr_frac.get();\n} else if (_param_r_rev.get() & (1u << actuator_type_idx)) {\n\tminimum[...] = -1.f;",
  "formula": "u_min = −f_rev (runtime-reversed prop; f_rev = CA_REV_THR_FRAC), −1 (symmetric reversible), 0 otherwise",
  "variables": {
   "f_rev": "fraction of forward thrust a forward prop produces in reverse [-]"
  },
  "defaults": [],
  "assumptions": "Reverse thrust is a constant fraction of forward thrust; default value of CA_REV_THR_FRAC not read.",
  "notes": "Default for CA_REV_THR_FRAC not found in the files scanned."
 },
 {
  "id": "px4ap-066",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Motor thrust curve model THR_MDL_FAC (relative thrust vs relative signal)",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/mixer_module/motor_params.yaml",
  "lines": "5-24",
  "code": "The model is:\n\nrel_thrust = factor * rel_signal^2 + (1-factor) * rel_signal\n\n, where rel_thrust is the normalized thrust between 0 and 1, and\nrel_signal is the relative motor control signal between 0 and 1.",
  "formula": "T_rel = f·x² + (1 − f)·x,  f = THR_MDL_FAC ∈ [0,1]",
  "variables": {
   "T_rel": "normalized static thrust [0..1]",
   "x": "relative motor control signal (e.g. PWM) [0..1]",
   "f": "THR_MDL_FAC [-]"
  },
  "defaults": [
   {
    "name": "THR_MDL_FAC",
    "value": "0.0",
    "unit": "-",
    "where": "src/lib/mixer_module/motor_params.yaml:20"
   }
  ],
  "assumptions": "Static (not dynamic) thrust curve; f = 0 is linear thrust vs signal; f = 1 is thrust ∝ signal² (standard propeller thrust ∝ ω²).",
  "notes": "The model is in the parameter documentation; the inverse is implemented in FunctionMotors.hpp (px4ap-067)."
 },
 {
  "id": "px4ap-067",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Inverse thrust curve: relative thrust command → motor signal",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/mixer_module/functions/FunctionMotors.hpp",
  "lines": "81-119",
  "code": "// rel_thrust = factor * x^2 + (1-factor) * x,\nconst float a = thrust_factor;\nconst float b = (1.f - thrust_factor);\nconst float tmp1 = b / (2.f * a);\nconst float tmp2 = b * b / (4.f * a * a);\nvalues[i] = -tmp1 + sqrtf(tmp2 + (control / a));\nvalues[i] =  tmp1 - sqrtf(tmp2 - (control / a));   // negative (reversible)\nvalues[i] = values[i] * 2.f - 1.f;                 // [0,1] to [-1,1]",
  "formula": "x = −b/(2a) + √(b²/(4a²) + T/a)  (T>0);  x = b/(2a) − √(b²/(4a²) − T/a)  (T<0, reversible);  a = f, b = 1 − f;  then non-reversible output = 2x − 1",
  "variables": {
   "T": "relative thrust command from allocator [0..1]",
   "x": "relative motor signal [0..1]",
   "f": "THR_MDL_FAC [-]"
  },
  "defaults": [],
  "assumptions": "Applied only when 0 < f ≤ 1; static curve; remap [0,1]→[−1,1] for non-reversible outputs (negative input → NaN = motor off).",
  "notes": "This is where the allocator's 'linear thrust' becomes the actual signal. Used also in actuator test (mixer_module.cpp:449)."
 },
 {
  "id": "px4ap-068",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Normalized output → PWM mapping",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/mixer_module/mixer_module.cpp",
  "lines": "560-566",
  "code": "output = math::interpolateNXY(value, {-1.f, 0.f, 1.f}, {(float)_min_value[i], (float)_center_value[i], (float)_max_value[i]});\n// Everything except servos, or if center is not set\noutput = math::interpolate(value, -1.f, 1.f, static_cast<float>(_min_value[i]), static_cast<float>(_max_value[i]));",
  "formula": "pwm = pwm_min + (value + 1)/2·(pwm_max − pwm_min)  (servos with center set: piecewise linear through center)",
  "variables": {
   "value": "normalized output in [−1,1]",
   "pwm_min/max/center": "channel min/max/center values [μs]"
  },
  "defaults": [],
  "assumptions": "Linear signal-to-PWM mapping; ESC response is not modeled.",
  "notes": "Defaults of PWM ranges not read (params.yaml not opened)."
 },
 {
  "id": "px4ap-069",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Fixed-wing control-surface airspeed scaling factor",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/fw_rate_control/FixedwingRateControl.cpp",
  "lines": "160-208",
  "code": "airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);\nconst float min_airspeed = math::max(_param_fw_airspd_stall.get(), 0.1f);\nconst float airspeed_constrained = math::max(airspeed, min_airspeed);\n_airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;",
  "formula": "s = V_trim / max(V_CAS, V_stall, 0.1)  (1.0 if FW_ARSP_SCALE_EN = 0)",
  "variables": {
   "V_CAS": "filtered calibrated airspeed [m/s]",
   "V_trim": "FW_AIRSPD_TRIM [m/s]",
   "V_stall": "FW_AIRSPD_STALL [m/s]",
   "s": "airspeed scaling [-]"
  },
  "defaults": [
   {
    "name": "FW_ARSP_SCALE_EN",
    "value": "1",
    "unit": "-",
    "where": "src/modules/fw_rate_control/fw_rate_control_params.yaml:219"
   },
   {
    "name": "FW_USE_AIRSPD",
    "value": "1",
    "unit": "-",
    "where": "fw_rate_control_params.yaml:15"
   },
   {
    "name": "FW_AIRSPD_TRIM",
    "value": "15.0",
    "unit": "m/s",
    "where": "src/lib/fw_performance_model/performance_model_params.yaml:127"
   },
   {
    "name": "FW_AIRSPD_STALL",
    "value": "7.0",
    "unit": "m/s",
    "where": "performance_model_params.yaml:140"
   }
  ],
  "assumptions": "Control-surface moment ∝ q·δ ∝ V²·δ, so the same deflection at lower V gives less moment; compensate by 1/V² in angular-acceleration-to-deflection (px4ap-070). Without airspeed the trim airspeed is used (scaling 1); VTOL in hover without airspeed uses stall airspeed.",
  "notes": "Uses CAS, not TAS; the V² dependence is implemented in px4ap-070."
 },
 {
  "id": "px4ap-070",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Fixed-wing rate control: angular acceleration → deflection (s² scaling), feed-forward, trim",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/fw_rate_control/FixedwingRateControl.cpp",
  "lines": "353-374, 388-402",
  "code": "trim *= _airspeed_scaling * _airspeed_scaling;\nscaled_gain_ff(i) = (gain_ff(i) >= 0.f) ? gain_ff(i) / _airspeed_scaling : gain_ff(i);\nVector3f control_u = _gain_compression.getGains().emult(angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling);\ntrim(0) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_r_vmin.get(), 0.0f);",
  "formula": "u = clamp( K_comp ⊙ (α_sp·s²) + trim·s² + Δtrim(V) , −1, 1) ;  K_ff,eff = K_ff/s (K_ff ≥ 0);  Δtrim(V): linear from FW_DTRIM_*_VMIN (at V_min) to 0 (at V_trim) to FW_DTRIM_*_VMAX (at V_max)",
  "variables": {
   "α_sp": "angular acceleration setpoint from rate controller [rad/s^2]",
   "s": "px4ap-069",
   "K_ff": "FW_RR_FF / FW_PR_FF / FW_YR_FF"
  },
  "defaults": [
   {
    "name": "FW_RR_FF",
    "value": "0.5",
    "unit": "-",
    "where": "fw_rate_control_params.yaml:142"
   },
   {
    "name": "FW_PR_FF",
    "value": "0.5",
    "unit": "-",
    "where": "fw_rate_control_params.yaml:156"
   },
   {
    "name": "FW_YR_FF",
    "value": "0.3",
    "unit": "-",
    "where": "fw_rate_control_params.yaml:170"
   }
  ],
  "assumptions": "Control authority ∝ V² (dynamic pressure) so the deflection for a given angular acceleration scales with s²; positive feedforward ∝ aerodynamic damping which scales linearly with airspeed. Manual mode: torque = stick·scale + trim·s² (lines 135-140).",
  "notes": "Comment (line 388): 'Positive feedforward compensates aerodynamic damping and is scaled linearly with airspeed.' Δtrim table interpolation for ± airspeed (lines 352-374); defaults for FW_DTRIM_* not read."
 },
 {
  "id": "px4ap-071",
  "group": 9,
  "group_name": "rotation",
  "quantity": "Fixed-wing attitude control: coordinated-turn rate feed-forward",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/fw_att_control/FixedwingAttitudeControl.cpp",
  "lines": "284-296",
  "code": "const float V = math::max(get_true_airspeed_constrained(), 0.1f);\nconst float q1 = 2.f * (q_current(0) * q_current(1) + q_current(2) * q_current(3)); // equivalent to 2.f * sin(roll) * cos(pitch)\nconst float yawrate_ff = CONSTANTS_ONE_G * q1 / V;\nconst float pitchrate_ff = q1 * yawrate_ff / (1.f - 2.f * q_current(1) * q_current(1) - 2.f * q_current(2) * q_current(2));\nconst float ff_scale = math::interpolate(tilt, radians(70.f), radians(75.f), 1.f, 0.f);",
  "formula": "r_ff = g·sinφ·cosθ / V ;  q_ff = q₁·r_ff / (cosφ·cosθ)  (cos tilt = 1 − 2(q_x²+q_y²));  scale: 1 for tilt ≤ 70°, linear to 0 at 75°",
  "variables": {
   "V": "true airspeed, constrained to [V_stall,V_max] CAS converted to TAS [m/s]",
   "q1": "2(q_w q_x + q_y q_z) = 2 sinφ cosθ [-]",
   "φ, θ": "roll, pitch [rad]"
  },
  "defaults": [],
  "assumptions": "Steady coordinated level-ish turn; body rates add to the proportional attitude loop output.",
  "notes": "Hamilton quaternion q = (w,x,y,z). Gravity constant 9.80665."
 },
 {
  "id": "px4ap-072",
  "group": 7,
  "group_name": "wind & airspeed",
  "quantity": "Fixed-wing attitude control: TAS from constrained CAS",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/fw_att_control/FixedwingAttitudeControl.cpp",
  "lines": "181-183",
  "code": "// limits are calibrated airspeeds, so convert after constraining\nreturn calc_true_from_calibrated_airspeed(\n\t       math::constrain(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_max.get()), _air_density);",
  "formula": "V_TAS = clamp(V_CAS, V_stall, V_max)·√(ρ₀/ρ)",
  "variables": {
   "ρ": "_air_density [kg/m^3]"
  },
  "defaults": [
   {
    "name": "FW_AIRSPD_MAX",
    "value": "20.0",
    "unit": "m/s",
    "where": "performance_model_params.yaml:167"
   }
  ],
  "assumptions": "CAS≈EAS.",
  "notes": ""
 },
 {
  "id": "px4ap-073",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "TECS specific energy rates (kinetic + potential)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/tecs/TECS.cpp",
  "lines": "337-350",
  "code": "specific_energy_rates.spe_rate.setpoint = control_setpoint.altitude_rate_setpoint * CONSTANTS_ONE_G; // potential energy rate of change\n// d/dt(0.5 V^2) = V * dV/dt: use measured TAS, not the TAS setpoint, to keep the airspeed loop linear\nspecific_energy_rates.ske_rate.setpoint = input.tas * control_setpoint.tas_rate_setpoint;\nspecific_energy_rates.spe_rate.estimate = input.altitude_rate * CONSTANTS_ONE_G;\nspecific_energy_rates.ske_rate.estimate = input.tas * input.tas_rate;",
  "formula": "ĖP = g·ḣ ;  ĖK = V·V̇ ;  Ė_total = g·ḣ + V·V̇ (specific, [m²/s³])",
  "variables": {
   "ḣ": "altitude rate [m/s]",
   "V": "true airspeed [m/s]",
   "V̇": "TAS rate [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Per unit mass; specific potential energy gh, specific kinetic ½V².",
  "notes": ""
 },
 {
  "id": "px4ap-074",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "TECS total-energy-rate limits and load-factor drag correction",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/tecs/TECS.cpp",
  "lines": "296-297, 598-603",
  "code": "limit.STE_rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;\nlimit.STE_rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;\n// Assume induced drag scales linearly with normal load factor.\n// The additional normal load factor is given by (1/cos(bank angle) - 1)\nste_rate.setpoint += param.load_factor_correction * (param.load_factor - 1.f);",
  "formula": "Ė_max = g·ḣ_climb,max ;  Ė_min = −g·ḣ_sink,min ;  Ė_sp ← Ė_sp + k_n·(n − 1),  n = 1/cos(bank)",
  "variables": {
   "k_n": "param.load_factor_correction (TECS parameter; default not read)",
   "n": "normal load factor [-]"
  },
  "defaults": [],
  "assumptions": "Induced drag power rise in turns is linear in load factor (as stated in source comment); extra energy rate required to hold speed in a turn.",
  "notes": "max_climb_rate / min_sink_rate come from FW_T_CLMB_MAX (5.0 m/s) and FW_T_SINK_MIN (2.0 m/s) via the performance model (px4ap-079)."
 },
 {
  "id": "px4ap-075",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "TECS energy-balance rate, climb-angle conversion and pitch setpoint",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/tecs/TECS.cpp",
  "lines": "471-476, 527-546",
  "code": "seb_rate.setpoint = specific_energy_rates.spe_rate.setpoint * weight.spe_weighting - specific_energy_rates.ske_rate.setpoint * weight.ske_weighting;\nreturn max(input.tas, param.tas_min, FLT_EPSILON) * CONSTANTS_ONE_G;\nfloat pitch_setpoint_unc = SEB_rate_correction / _climbAngleToSebRate(input, param) + _pitch_integ_state + param.level_flight_pitch;",
  "formula": "Ė_B = w_p·ĖP − w_k·ĖK ;  γ_correction = Ė_B,corr/(g·max(V, V_min)) ;  θ_sp = γ_correction + integ + θ_level",
  "variables": {
   "w_p, w_k": "spe_weighting = 2 − w_k, ske_weighting ∈[0,2] (px4ap pitch_speed_weight, lines 369-390)",
   "V_min": "param.tas_min [m/s]",
   "θ_level": "level_flight_pitch [rad]"
  },
  "defaults": [],
  "assumptions": "Climb angle γ ≈ ḣ/V; Ė_B/(gV) ≈ γ. Pitch angle follows climb angle (comment: angle of attack const, small-lag). Pitch-rate limit from vert_accel_limit: pitch_increment = dt·a_lim/V (line 446).",
  "notes": "Standard TECS energy-balance formulation."
 },
 {
  "id": "px4ap-076",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "TECS throttle from total energy rate (linear map between trim/min/max)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/tecs/TECS.cpp",
  "lines": "646-673; TECS.hpp:430",
  "code": "// assume airspeed and density-independent delta_throttle to sink/climb rate mapping\n// TODO: include air density for thrust mappings\nconst float throttle_above_trim_per_ste_rate = (param.throttle_max - param.throttle_trim) / limit.STE_rate_max;\nconst float throttle_below_trim_per_ste_rate = (param.throttle_trim - param.throttle_min) / limit.STE_rate_min;\nthrottle_predicted = param.throttle_trim + ste_rate.setpoint * throttle_above_trim_per_ste_rate;",
  "formula": "thr_pred = thr_trim + Ė_sp·(thr_max − thr_trim)/Ė_max  (Ė_sp ≥ 0);  thr_trim − Ė_sp·(thr_trim − thr_min)/Ė_min (Ė_sp < 0)",
  "variables": {
   "Ė_sp": "total energy rate setpoint [m²/s³]",
   "thr_trim,min,max": "throttle values [0..1]"
  },
  "defaults": [],
  "assumptions": "Thrust change ∝ energy rate and independent of airspeed and density (source TODO); STE-rate-to-throttle gain 1/(Ė_max − Ė_min).",
  "notes": "Note: for Ė_sp<0 code uses '−Ė_sp·(thr_trim−thr_min)/Ė_min' (Ė_min negative, so result lowers throttle)."
 },
 {
  "id": "px4ap-077",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "TECS constant-rate airspeed filter and EAS→TAS conversion",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/tecs/TECS.cpp",
  "lines": "TECS.cpp:99-106, 705-708, 732; TECS.hpp:131-134",
  "code": "new_state_predicted(0) = _airspeed_state.speed + dt * _airspeed_state.speed_rate;\nnew_state(0) = new_state_predicted(0) + dt * (kKg00 * innovation(0) + kKg01 * innovation(1));\n_control_param.tas_min = input.eas_to_tas * _equivalent_airspeed_min;\nconst float true_airspeed = input.eas_to_tas * _airspeed_filter.getState().speed;",
  "formula": "V ← V + dt·V̇ + dt(K₀₀ ε₀ + K₀₁ ε₁),  K₀₀=1.9194, K₀₁=0.2593, K₁₀=2.1164, K₁₁=0.6718 ;  V_TAS = eas_to_tas · V_EAS",
  "variables": {
   "eas_to_tas": "conversion factor input [-]",
   "ε": "innovation vector (speed, speed rate)"
  },
  "defaults": [],
  "assumptions": "Constant airspeed-rate model, steady-state Kalman gains (continuous gains approximated by dt·K, source comment lines 92-95).",
  "notes": "TECS operates on EAS limits and converts to TAS using eas_to_tas. Also in: src/lib/tecs/TECS.hpp."
 },
 {
  "id": "px4ap-078",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Performance model: weight ratio and weight-scaled stall/min/trim airspeeds (load factor and flaps)",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/fw_performance_model/PerformanceModel.cpp",
  "lines": "85-101, 172-189",
  "code": "weight -= (1.f - _fuel_fraction_remaining) * _param_weight_fuel.get();\nweight_factor = math::constrain(weight / _param_weight_base.get(), kMinWeightRatio, kMaxWeightRatio);\nreturn math::constrain(_param_fw_airspd_trim.get() * sqrtf(getWeightRatio()), _param_fw_airspd_min.get(), _param_fw_airspd_max.get());\nconst float airspeed_flap_factor = math::lerp(1.f, _param_fw_airspd_flp_sc.get(), flaps_setpoint);\nreturn (_param_fw_airspd_stall.get() * airspeed_flap_factor) * sqrtf(getWeightRatio() * load_factor);",
  "formula": "W_r = clamp((W_gross − (1−f_fuel)·W_fuel)/W_base, 0.5, 2.0) ;  V_trim = clamp(V_trim,0·√W_r, V_min, V_max) ;  V_stall(n, flaps) = V_stall,0·lerp(1, K_flap, flaps)·√(W_r·n)",
  "variables": {
   "W_r": "weight ratio [-]",
   "n": "load factor [-]",
   "f_fuel": "fuel fraction remaining [0..1]",
   "K_flap": "FW_AIRSPD_FLP_SC [-]"
  },
  "defaults": [
   {
    "name": "WEIGHT_BASE / WEIGHT_GROSS / WEIGHT_FUEL",
    "value": "−1.0 / −1.0 / −1.0 (disabled)",
    "unit": "kg",
    "where": "performance_model_params.yaml:56-88"
   },
   {
    "name": "FW_AIRSPD_FLP_SC",
    "value": "1.0",
    "unit": "-",
    "where": "performance_model_params.yaml:172-178"
   },
   {
    "name": "FW_AIRSPD_MIN",
    "value": "10.0",
    "unit": "m/s",
    "where": "performance_model_params.yaml:145-157"
   }
  ],
  "assumptions": "Stall speed V_s ∝ √(W·n/(ρ S C_Lmax)) at constant density (calibrated airspeed removes ρ); weight ratio clamped [0.5, 2.0].",
  "notes": "Source file is lib/fw_performance_model (not originally in the list but used by TECS/airspeed selector)."
 },
 {
  "id": "px4ap-079",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Performance model: density- and weight-corrected max climb rate and min sink rate",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/fw_performance_model/PerformanceModel.cpp",
  "lines": "102-122, 167-171",
  "code": "const float climbrate_gradient = math::max((_param_fw_t_clmb_max.get() - kClimbRateAtServiceCeiling) / (kAirDensitySeaLevelStandardAtmos - ceiling_density), 0.0f);\nconst float delta_rho = air_density - kAirDensitySeaLevelStandardAtmos;\nclimbrate_max = math::constrain(_param_fw_t_clmb_max.get() + climbrate_gradient * delta_rho, kClimbRateAtServiceCeiling, _param_fw_t_clmb_max.get());\nreturn climbrate_max / getWeightRatio();\nreturn _param_fw_t_sink_min.get() * sqrtf(getWeightRatio() * kAirDensitySeaLevelStandardAtmos / air_density);",
  "formula": "ḣ_max(ρ) = clamp(ḣ₀ + (ḣ₀ − 0.5)/(ρ₀ − ρ_ceil)·(ρ − ρ₀), 0.5, ḣ₀)/W_r ;  ḣ_sink,min = ḣ_s,0·√(W_r·ρ₀/ρ)",
  "variables": {
   "ḣ₀": "FW_T_CLMB_MAX [m/s]",
   "ρ_ceil": "density at service ceiling (ISA via getPressureFromAltitude, getStandardTemperatureAtAltitude) [kg/m^3]",
   "ḣ_s,0": "FW_T_SINK_MIN [m/s]",
   "0.5": "kClimbRateAtServiceCeiling [m/s]"
  },
  "defaults": [
   {
    "name": "FW_T_CLMB_MAX",
    "value": "5.0",
    "unit": "m/s",
    "where": "performance_model_params.yaml:101"
   },
   {
    "name": "FW_T_SINK_MIN",
    "value": "2.0",
    "unit": "m/s",
    "where": "performance_model_params.yaml:114"
   },
   {
    "name": "FW_SERVICE_CEIL",
    "value": "−1.0 (disabled)",
    "unit": "m",
    "where": "performance_model_params.yaml:43-51"
   }
  ],
  "assumptions": "Max climb rate decreases linearly with density (reaching 0.5 m/s at the service ceiling); sink rate at min-sink airspeed ∝ √(W/ρ). Density floor kAirDensityStandardAtmos11000Amsl = 0.3639 (atmosphere.h:51).",
  "notes": ""
 },
 {
  "id": "px4ap-080",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Performance model: trim throttle vs airspeed and air-density scaling",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/fw_performance_model/PerformanceModel.cpp",
  "lines": "123-166",
  "code": "const float throttle_trim = getTrimThrottleForCalibratedAirspeed(airspeed_sp) * getAirDensityThrottleScale(air_density);\n// scale throttle as a function of sqrt(rho0/rho)\nconst float eas2tas = sqrtf(kAirDensitySeaLevelStandardAtmos / air_density);\nair_density_throttle_scale = math::constrain(eas2tas, 1.f, eas2tas_at_11km_amsl);\nthrottle_trim = _param_fw_thr_trim.get() - slope_below_trim * (_param_fw_airspd_trim.get() - calibrated_airspeed_sp);",
  "formula": "thr_trim(V,ρ) = clamp( thr(V)·clamp(√(ρ₀/ρ), 1, √(ρ₀/0.3639)), thr_min, thr_max ) ;  thr(V) piecewise linear: slope_below = (thr_trim − thr_min,V)/(V_trim − V_min), slope_above = (thr_max,V − thr_trim)/(V_max − V_trim)",
  "variables": {
   "thr_trim, thr_aspd_min/max": "FW_THR_TRIM, FW_THR_ASPD_MIN, FW_THR_ASPD_MAX [-]",
   "V": "calibrated airspeed setpoint [m/s]"
  },
  "defaults": [
   {
    "name": "FW_THR_TRIM",
    "value": "0.6",
    "unit": "-",
    "where": "performance_model_params.yaml:11"
   },
   {
    "name": "FW_THR_ASPD_MIN / MAX",
    "value": "0.0 / 0.0",
    "unit": "-",
    "where": "performance_model_params.yaml:25, 38"
   }
  ],
  "assumptions": "Parasite drag mapping airspeed→throttle taken as linear with different gradients above/below trim (source comment); thrust needed at altitude scales as √(ρ₀/ρ) (EAS→TAS ratio).",
  "notes": ""
 },
 {
  "id": "px4ap-081",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL",
  "quantity": "Fixed-wing performance model: sanity check of minimum airspeed at max bank",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/fw_performance_model/PerformanceModel.cpp",
  "lines": "251-262",
  "code": "const float max_bank_loadfactor = 1 / cosf(math::radians(_param_fw_r_lim.get()));\nconst float min_airspd_at_max_bank = getMinimumCalibratedAirspeed(max_bank_loadfactor, /*flaps_setpoint = */0.0f);\n//      FW_AIRSPD_MAX >= FW_AIRSPD_MIN * sqrt(WEIGHT_GROSS/WEIGHT_BASE) * sqrt(1/cos(FW_R_LIM))",
  "formula": "n_max = 1/cos(φ_lim) ;  V_min,bank = V_min·√(W_r·n_max) ≤ V_max",
  "variables": {
   "φ_lim": "FW_R_LIM roll limit [deg]"
  },
  "defaults": [],
  "assumptions": "Level coordinated turn: n = 1/cos φ.",
  "notes": ""
 },
 {
  "id": "px4ap-082",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Not found: propeller thrust/torque as T = c_t·ω² with explicit RPM, motor time constant, battery-voltage compensation in autopilot code scanned",
  "simulator": "PX4 autopilot",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/control_allocator, src/lib/mixer_module",
  "lines": "-",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "Within the scanned autopilot files the only thrust curve is THR_MDL_FAC (static, px4ap-066/067) and the allocator is linear in normalized output; no motor lag model; slew rate limit only (px4ap-059). Battery-voltage thrust compensation (e.g. MC_THR... / battery scaling) was not searched in this task.",
  "notes": "Not found: motor/propeller dynamics (first-order lag), propeller thrust ∝ RPM² with explicit c_T, battery-voltage scaling of thrust, in the control_allocator and mixer_module files read."
 },
 {
  "id": "px4ap-083",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "Not found: fixed-wing aerodynamic force model (lift/drag coefficients) in autopilot",
  "simulator": "PX4 autopilot",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/modules/ekf2, src/lib/fw_performance_model, src/lib/tecs",
  "lines": "-",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "EKF2 drag fusion is multirotor-only (body X/Y); fixed-wing uses only airspeed/sideslip fusion. The autopilot has no C_L/C_D model: lift and drag appear only as load-factor/weight scalings (px4ap-040, -074, -078) and empirical throttle maps (px4ap-080).",
  "notes": "Not found: lift/drag/pitching-moment coefficients."
 },
 {
  "id": "px4ap-084",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Not physics (table lookup): world magnetic model / geo",
  "simulator": "PX4 autopilot",
  "vehicle": "both",
  "repo": "PX4/PX4-Autopilot",
  "commit": "d28a642",
  "file": "src/lib/world_magnetic_model/geo_mag_declination.cpp; src/lib/geo/geo.cpp",
  "lines": "geo_mag_declination.cpp:56-86; geo.cpp:62-101",
  "code": "",
  "formula": "not used as a physical model: declination/inclination/strength by bilinear interpolation in a lookup table (full-degree resolution); geo.cpp holds azimuthal-equidistant map projection with CONSTANTS_RADIUS_OF_EARTH = 6371000 m (geo.h:55)",
  "variables": {},
  "defaults": [],
  "assumptions": "Only grep'd for formula content; not scanned line by line (scope says include only if used as physics).",
  "notes": "EKF2 in this commit uses LatLonAlt (WGS-84 ellipsoid, px4ap-014..017) instead of the spherical geo.cpp projection for its prediction."
 }
];
