window.SIM_FORMULAS_AIRSIM = [
 {
  "id": "PA-1-01",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Gravity vector in the world (NED) frame",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/environment.hpp",
  "lines": "44-45",
  "code": "env_info.gravity =\n    Vector3(0, 0, EarthUtils::GetGravity(env_info.geo_point.altitude));",
  "formula": "g_vec = (0, 0, +g(h))  [NED: +z is down, so gravity is +z]",
  "variables": {
   "g(h)": "scalar gravity from EarthUtils::GetGravity [m/s^2]",
   "h": "geodetic altitude of the robot = home_altitude - z_ned [m]"
  },
  "defaults": [
   {
    "name": "kGravity",
    "value": "9.80665",
    "unit": "m/s^2",
    "where": "core_sim/include/core_sim/earth_utils.hpp:72-73"
   }
  ],
  "assumptions": "NED world frame. Gravity direction is always +z (no latitude or geoid dependence). It is stored as an acceleration, not a force, and is added directly to the linear acceleration (see PA-8-01).",
  "notes": "Recomputed every tick by Robot::UpdateEnvironment -> Environment::SetPosition (robot.cpp:1517-1520)."
 },
 {
  "id": "PA-1-02",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Gravity magnitude versus altitude",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "206-218",
  "code": "if (altitude < 10000 && altitude > -10000) {\n  return kGravity;\n} else if (altitude < 100000 && altitude > -100000) {\n  return kGravity * (1.0f - 2.0f * altitude / kEarthRadius);\n} else {\n  float factor = 1.0f + altitude / kEarthRadius;",
  "formula": "g(h) = g0 for |h| < 10 km;  g0 (1 - 2h/R) for 10 km <= |h| < 100 km;  g0 / (1 + h/R)^2 otherwise (last branch: return kGravity / factor / factor)",
  "variables": {
   "g0": "standard gravity [m/s^2]",
   "R": "Earth equatorial radius [m]",
   "h": "altitude [m]"
  },
  "defaults": [
   {
    "name": "kGravity",
    "value": "9.80665",
    "unit": "m/s^2",
    "where": "earth_utils.hpp:72-73"
   },
   {
    "name": "kEarthRadius",
    "value": "6378137",
    "unit": "m",
    "where": "earth_utils.hpp:76-77"
   }
  ],
  "assumptions": "Spherical Earth, point mass; first-order binomial expansion between 10 and 100 km. Every normal drone flight is below 10 km, so g is exactly 9.80665 there.",
  "notes": "Comment cites citycollegiate.com for the derivation."
 },
 {
  "id": "PA-1-03",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Geopotential height",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "127-130",
  "code": "static float GetGeopotential(float altitude_km) {\n  static constexpr float radius_km = kEarthRadius / 1000.0f;\n  return radius_km * altitude_km / (radius_km + altitude_km);\n}",
  "formula": "H = R_km h_km / (R_km + h_km)",
  "variables": {
   "H": "geopotential height [km]",
   "h_km": "geometric altitude [km]",
   "R_km": "6378.137 [km]"
  },
  "defaults": [],
  "assumptions": "Environment::SetPosition passes geo_point.altitude/1000 (environment.hpp:47-48).",
  "notes": "All atmosphere tables below take H in km."
 },
 {
  "id": "PA-1-04",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Standard atmosphere temperature (piecewise)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "132-160",
  "code": "if (geopot_height <= 11) {  // troposphere\n  return kSeaLevelTemperature - (6.5f * geopot_height);\n} else if (geopot_height <= 20) {  // Staroshere starts\n  return 216.65f;\n} else if (geopot_height <= 32) {\n  return 196.65f + geopot_height;",
  "formula": "T(H) [K], H in km: H<=11: 288.15 - 6.5H;  11<H<=20: 216.65;  20<H<=32: 196.65 + H;  32<H<=47: 228.65 + 2.8(H-32);  47<H<=51: 270.65;  51<H<=71: 270.65 - 2.8(H-51);  71<H<=84.85: 214.65 - 2(H-71);  H>84.85: 3",
  "variables": {
   "T": "standard temperature [K]",
   "H": "geopotential height [km]"
  },
  "defaults": [
   {
    "name": "kSeaLevelTemperature",
    "value": "288.15",
    "unit": "K",
    "where": "earth_utils.hpp:80-81"
   }
  ],
  "assumptions": "US-standard-atmosphere layers; no weather, no humidity, no temperature offset input.",
  "notes": "Comment cites Stull, Practical Meteorology p.12 below 51 km and braeunig.us above. The final branch returns 3 K (a placeholder, not physical)."
 },
 {
  "id": "PA-1-05",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Standard atmosphere pressure (piecewise)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "162-192",
  "code": "if (geopot_height <= 11) {\n  return kSeaLevelPressure *\n         std::pow(kSeaLevelTemperature / std_temperature, -5.255877f);\n} else if (geopot_height <= 20) {\n  return 22632.06f * std::exp(-0.1577f * (geopot_height - 11));\n} else if (geopot_height <= 32) {",
  "formula": "p(H) [Pa]: H<=11: p0 (T0/T)^(-5.255877) = p0 (T/T0)^5.255877;  11<H<=20: 22632.06 exp(-0.1577 (H-11));  20<H<=32: 5474.889 (216.65/T)^34.16319;  32<H<=47: 868.0187 (228.65/T)^12.2011;  47<H<=51: 110.9063 exp(-0.1262 (H-47));  51<H<=71: 66.93887 (270.65/T)^(-12.2011);  71<H<=84.85: 3.956420 (214.65/T)^(-17.0816);  above: 1e-3",
  "variables": {
   "p": "static pressure [Pa]",
   "T": "standard temperature from PA-1-04 [K]",
   "H": "geopotential height [km]",
   "p0, T0": "sea-level pressure and temperature"
  },
  "defaults": [
   {
    "name": "kSeaLevelPressure",
    "value": "101325",
    "unit": "Pa",
    "where": "earth_utils.hpp:78-79"
   }
  ],
  "assumptions": "US-standard atmosphere. Code has a TODO that negative altitude is not handled (pressure keeps growing with the same exponent).",
  "notes": "The overload GetStandardPressure(altitude) at lines 194-199 chains geopotential -> temperature -> pressure from altitude in metres."
 },
 {
  "id": "PA-1-06",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Air density (ideal gas law)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "201-204",
  "code": "static float GetAirDensity(float std_pressure, float std_temperature) {\n  // http://www.braeunig.us/space/atmmodel.htm\n  return std_pressure / 287.053f / std_temperature;  // kg/m^3\n}",
  "formula": "rho = p / (R_air T),  R_air = 287.053 J/(kg K)",
  "variables": {
   "rho": "air density [kg/m^3]",
   "p": "standard pressure [Pa]",
   "T": "standard temperature [K]",
   "R_air": "specific gas constant of dry air [J/(kg K)]"
  },
  "defaults": [
   {
    "name": "kSeaLevelAirDensity",
    "value": "1.224999",
    "unit": "kg/m^3",
    "where": "earth_utils.hpp:82-84 (comment: SL pressure 101325 / 287.053 / SL temp 288.15)"
   }
  ],
  "assumptions": "Dry air, standard atmosphere; density depends on altitude only (no temperature or pressure disturbance).",
  "notes": "Rotor thrust uses rho/kSeaLevelAirDensity (PA-4-01); drag and lift-drag use rho directly (PA-5-03, PA-6-04)."
 },
 {
  "id": "PA-1-07",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Pressure altitude (altimeter formula)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "220-227",
  "code": "float pressure_altitude =\n    (1 - pow(pressure / kSeaLevelPressure, 0.190284f)) * 145366.45f *\n    0.3048f;  // Feet to Meters",
  "formula": "h_p = (1 - (p/p0)^0.190284) * 145366.45 ft * 0.3048 m/ft",
  "variables": {
   "h_p": "pressure altitude [m]",
   "p": "measured static pressure [Pa]",
   "p0": "101325 [Pa]"
  },
  "defaults": [],
  "assumptions": "Troposphere altimeter formula (Wikipedia Pressure_altitude), used by the barometer sensor (PA-14-06).",
  "notes": "Valid only in the troposphere; it is used regardless of altitude."
 },
 {
  "id": "PA-1-08",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Standard constants (sea level, speed of sound, Earth radius, speed of light)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "72-86",
  "code": "static constexpr float kGravity = 9.80665f;\nstatic constexpr float kSpeedOfLight = 299792458.0f;\nstatic constexpr float kEarthRadius = 6378137.0f;\nstatic constexpr float kSeaLevelPressure = 101325.0f;\nstatic constexpr float kSeaLevelTemperature = 288.15f;\nstatic constexpr float kSeaLevelAirDensity = 1.224999f;",
  "formula": "constants only",
  "variables": {
   "kSeaLevelSpeedOfSound": "340.29 [m/s], line 85-86, used by the airspeed sensor",
   "kSpeedOfLight": "299792458 [m/s], used as a velocity clip"
  },
  "defaults": [
   {
    "name": "kGravity",
    "value": "9.80665",
    "unit": "m/s^2",
    "where": "earth_utils.hpp:72"
   },
   {
    "name": "kEarthRadius",
    "value": "6378137",
    "unit": "m",
    "where": "earth_utils.hpp:76"
   },
   {
    "name": "kSeaLevelPressure",
    "value": "101325",
    "unit": "Pa",
    "where": "earth_utils.hpp:78"
   },
   {
    "name": "kSeaLevelTemperature",
    "value": "288.15",
    "unit": "K",
    "where": "earth_utils.hpp:80"
   },
   {
    "name": "kSeaLevelAirDensity",
    "value": "1.224999",
    "unit": "kg/m^3",
    "where": "earth_utils.hpp:83"
   },
   {
    "name": "kSeaLevelSpeedOfSound",
    "value": "340.29",
    "unit": "m/s",
    "where": "earth_utils.hpp:85"
   }
  ],
  "assumptions": "Constants are float, not double.",
  "notes": "The rotor model uses its own base density kBaseAirDensity = 1.225 (rotor.hpp:84), which differs from kSeaLevelAirDensity = 1.224999 in the 6th digit."
 },
 {
  "id": "PA-1-09",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Local NED position to geodetic latitude, longitude, altitude",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "100-125",
  "code": "double x_rad = pos_ned.x() / kEarthRadius;\ndouble y_rad = pos_ned.y() / kEarthRadius;\ndouble c = std::sqrt(x_rad * x_rad + y_rad * y_rad);\nlat_rad = std::asin(cos_c * home_geo_point.sin_lat +\n                    (x_rad * sin_c * home_geo_point.cos_lat) / c);\nhome_geo_point.geo_point.altitude - pos_ned.z()",
  "formula": "c = sqrt((N/R)^2 + (E/R)^2);  lat = asin(cos c sin lat0 + (N/R) sin c cos lat0 / c);  lon = lon0 + atan2((E/R) sin c, c cos lat0 cos c - (N/R) sin lat0 sin c);  alt = alt0 - D",
  "variables": {
   "N,E,D": "NED position relative to the world origin [m]",
   "lat0,lon0,alt0": "home geo point [rad, rad, m]",
   "R": "6378137 [m]"
  },
  "defaults": [],
  "assumptions": "Spherical Earth, azimuthal-equidistant (gnomonic-free) inverse projection around the home point; when c is approximately 0 it returns the home lat/lon.",
  "notes": "Altitude = home altitude minus the NED z. This is the h used by gravity, temperature and pressure."
 },
 {
  "id": "PA-1-10",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Environment state chain (position to gravity, temperature, pressure, density)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/environment.hpp",
  "lines": "38-57",
  "code": "float geo_potential =\n    EarthUtils::GetGeopotential(env_info.geo_point.altitude / 1000.0f);\nenv_info.temperature = EarthUtils::GetStandardTemperature(geo_potential);\nenv_info.air_pressure =\n    EarthUtils::GetStandardPressure(geo_potential, env_info.temperature);\nenv_info.air_density =\n    EarthUtils::GetAirDensity(env_info.air_pressure, env_info.temperature);",
  "formula": "h -> H (PA-1-03) -> T (PA-1-04) -> p (PA-1-05) -> rho (PA-1-06); g = (0,0,g(h)) (PA-1-01)",
  "variables": {
   "env_info": "per-robot Environment::EnvInfo: gravity [m/s^2], temperature [K], air_pressure [Pa], air_density [kg/m^3]"
  },
  "defaults": [],
  "assumptions": "Evaluated after each physics step from the new position (scene.cpp:1099-1101). Atmosphere and gravity are therefore one step behind the kinematics they feed.",
  "notes": "EnvInfo defaults (environment.hpp:16-22) are all zero until SetPosition is first called."
 },
 {
  "id": "PA-2-01",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor maximum speed squared and maximum thrust",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/actuators/rotor.hpp",
  "lines": "85-89",
  "code": "float revs_per_second = max_rpm / 60.0f;\nmax_speed_square = (float)std::pow(revs_per_second * 2 * M_PI, 2);\nmax_thrust =\n    (float)(coeff_of_thrust * kBaseAirDensity * revs_per_second *\n            revs_per_second * std::pow(propeller_diameter, 4));",
  "formula": "n_max = RPM_max/60;  omega_max^2 = (2 pi n_max)^2;  T_max = C_T rho0 n_max^2 D^4",
  "variables": {
   "n_max": "maximum revolutions per second [rev/s]",
   "omega_max": "maximum angular speed [rad/s]",
   "C_T": "thrust coefficient [-] (coeff_of_thrust, from UIUC propeller database)",
   "rho0": "base air density 1.225 [kg/m^3] (kBaseAirDensity)",
   "D": "propeller diameter [m]",
   "T_max": "thrust at full command, sea-level density [N]"
  },
  "defaults": [
   {
    "name": "coeff_of_thrust (struct default)",
    "value": "0.0",
    "unit": "-",
    "where": "rotor.hpp:52"
   },
   {
    "name": "max_rpm (struct default)",
    "value": "0.0",
    "unit": "rpm",
    "where": "rotor.hpp:54"
   },
   {
    "name": "propeller_diameter (struct default)",
    "value": "0.0",
    "unit": "m",
    "where": "rotor.hpp:55"
   },
   {
    "name": "coeff-of-thrust (example robot config)",
    "value": "0.109919",
    "unit": "-",
    "where": "client/python/example_user_scripts/sim_config/robot_quadrotor_fastphysics.jsonc:235"
   },
   {
    "name": "max-rpm (example)",
    "value": "6396.667",
    "unit": "rpm",
    "where": "robot_quadrotor_fastphysics.jsonc:237"
   },
   {
    "name": "propeller-diameter (example)",
    "value": "0.2286",
    "unit": "m",
    "where": "robot_quadrotor_fastphysics.jsonc:238"
   }
  ],
  "assumptions": "Static thrust law: T proportional to n^2 (no advance-ratio or forward-flight correction). Same as legacy AirSim. The struct defaults are zero, so the config must supply C_T, RPM and D.",
  "notes": "Source comment at line 90 says 'defaults to 4.179446268f' (the GWS 9x5 value with the example numbers above); this is stale because the struct defaults are 0. Lines 71-82 cite physics.stackexchange.com/a/32013 and the UIUC propDB."
 },
 {
  "id": "PA-2-02",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor maximum torque (propeller drag torque)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/actuators/rotor.hpp",
  "lines": "91-93",
  "code": "max_torque = (float)(coeff_of_torque * kBaseAirDensity * revs_per_second *\n                     revs_per_second * std::pow(propeller_diameter, 5) /\n                     (2 * M_PI));  // defaults to 0.055562f",
  "formula": "Q_max = C_P rho0 n_max^2 D^5 / (2 pi)",
  "variables": {
   "C_P": "power coefficient used as the torque coefficient [-] (coeff_of_torque)",
   "Q_max": "reaction torque at full command [N m]"
  },
  "defaults": [
   {
    "name": "coeff_of_torque (struct default)",
    "value": "0.0",
    "unit": "-",
    "where": "rotor.hpp:53"
   },
   {
    "name": "coeff-of-torque (example)",
    "value": "0.040164",
    "unit": "-",
    "where": "robot_quadrotor_fastphysics.jsonc:236"
   }
  ],
  "assumptions": "Q = P/omega with P = C_P rho n^3 D^5, folded into the n^2 form. Source comment 'defaults to 0.055562f' holds only for the example config values.",
  "notes": "The member is named coeff_of_torque but the doc and the legacy code call it C_P (power coefficient); torque = C_P rho n^2 D^5/(2 pi)."
 },
 {
  "id": "PA-2-03",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor thrust and torque from the filtered command",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "383-385",
  "code": "thrust_scalar_ = control_signal_filtered * rotor_settings_.max_thrust;\ntorque_scalar_ = control_signal_filtered * rotor_settings_.max_torque *\n                 static_cast<int>(rotor_settings_.turning_direction);",
  "formula": "T = u_f T_max;  Q = sigma u_f Q_max   (sigma = +1 clockwise, -1 counter-clockwise)",
  "variables": {
   "u_f": "filtered, clamped command in [0,1] [-]",
   "T": "thrust [N]",
   "Q": "signed reaction torque [N m]",
   "sigma": "RotorTurningDirection: CW = +1, CCW = -1"
  },
  "defaults": [],
  "assumptions": "Thrust is LINEAR in the command (u_f), and speed is sqrt(u_f) (PA-3-02): the command is a thrust fraction, not an rpm fraction. The thrust-to-torque ratio is constant (Q/T = Q_max/T_max). In rotor.hpp:39 the comment says +ve torque gives clockwise rotation in NED.",
  "notes": "Zeroed when a fault is injected (rotor.cpp:387-392). Same equations as legacy AirSim (RotorActuator.hpp:124-126)."
 },
 {
  "id": "PA-2-04",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor force and torque vectors in the rotor frame (density-scaled, tilted)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "398-409",
  "code": "Vector3 wrench_force_rotor_frame =\n    rotor_settings_.normal_vector * thrust_scalar_ * air_density_ratio_;\nVector3 wrench_torque_rotor_frame =\n    rotor_settings_.normal_vector * torque_scalar_ * air_density_ratio_;\nwrench_point_.wrench.force =\n    quat_tilt_._transformVector(wrench_force_rotor_frame);",
  "formula": "F_rotor = n_hat T (rho/rho0);  tau_rotor = n_hat Q (rho/rho0);  F = q_tilt * F_rotor;  tau = q_tilt * tau_rotor",
  "variables": {
   "n_hat": "rotor axis in the rotor frame, default (0,0,-1) = up in NED [-]",
   "rho/rho0": "air_density_ratio_ = env air density / 1.224999 [-]",
   "q_tilt": "quaternion from the tilt actuator, identity if none [-]"
  },
  "defaults": [
   {
    "name": "normal_vector",
    "value": "(0, 0, -1)",
    "unit": "-",
    "where": "rotor.hpp:48"
   }
  ],
  "assumptions": "Thrust acts along -z body (up). Docs (config_robot.md:623) state only normal-vector '0 0 -1' is supported. The torque vector points along the same axis as the thrust, so a CW rotor (sigma=+1) gives torque along -z = counter-clockwise reaction on the body, as expected.",
  "notes": "The wrench is stored in wrench_point_ (position = zero here; the lever arm comes from the rotor origin in PA-2-05)."
 },
 {
  "id": "PA-2-05",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Total body wrench from all rotor wrenches (including lever-arm torque r x F)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "113-143",
  "code": "wrench_pt.wrench.torque =\n    pose.orientation._transformVector(wrench_pt.wrench.torque);\nwrench_pt.wrench.force =\n    pose.orientation._transformVector(wrench_pt.wrench.force);\naggregate_wrench += wrench_pt.wrench;\naggregate_wrench.torque += wrench_pt.position.cross(wrench_pt.wrench.force);",
  "formula": "F_body = sum_i R_i F_i;  tau_body = sum_i (R_i tau_i + r_i x R_i F_i);  F_world = q F_body + F_ext",
  "variables": {
   "R_i": "rotation from rotor i frame to the body frame [-]",
   "r_i": "rotor i origin in the body frame (relative to the root-link CG) [m]",
   "F_ext": "external force set through the API (world frame) [N]"
  },
  "defaults": [],
  "assumptions": "Force is converted to world frame (line 137-138); torque stays in the body frame. The actuator wrench points include wheels. Wrench is computed at the end of each tick and held constant over the next physics step (scene.cpp:1137-1141, zero-order hold).",
  "notes": "Lines 142-143: external force wrench added as Wrench(ext_force_, 0), in world frame."
 },
 {
  "id": "PA-2-06",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor mechanical power",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "394",
  "code": "power_ = abs(torque_scalar_ * rotating_speed_raw);",
  "formula": "P = |Q omega_raw| = u_f Q_max sqrt(u_f) omega_max",
  "variables": {
   "P": "rotor power [W]",
   "Q": "signed torque scalar [N m]",
   "omega_raw": "unclipped rotor speed [rad/s]"
  },
  "defaults": [],
  "assumptions": "Not scaled by air density; uses the shaft power without efficiency; the fault-injected case gives rotating_speed_raw = 0.",
  "notes": "Summed over rotors in Robot::UpdateActuators (robot.cpp:1450) and fed to the battery (PA-13-01). Note abs() is applied to a float (may resolve to the int overload in C++; unverified)."
 },
 {
  "id": "PA-2-07",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor model defaults (GWS 9x5, DJI Phantom 2) - gap filled from legacy AirSim",
  "simulator": "AirSim (legacy)",
  "vehicle": "multirotor",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/vehicles/multirotor/RotorParams.hpp",
  "lines": "37-61",
  "code": "real_T C_T = 0.109919f;\nreal_T C_P = 0.040164f;\nreal_T air_density = 1.225f;\nreal_T max_rpm = 6396.667f;\nreal_T propeller_diameter = 0.2286f;\nreal_T control_signal_filter_tc = 0.005f;",
  "formula": "max_thrust = C_T rho n^2 D^4 = 4.179446268 N;  max_torque = C_P rho n^2 D^5/(2 pi) = 0.055562 N m (values in the header comments)",
  "variables": {
   "C_T, C_P": "UIUC propeller coefficients [-]",
   "rho": "1.225 [kg/m^3]",
   "D": "propeller diameter [m]"
  },
  "defaults": [
   {
    "name": "C_T",
    "value": "0.109919",
    "unit": "-",
    "where": "RotorParams.hpp:37"
   },
   {
    "name": "C_P",
    "value": "0.040164",
    "unit": "-",
    "where": "RotorParams.hpp:38"
   },
   {
    "name": "max_rpm",
    "value": "6396.667",
    "unit": "rpm",
    "where": "RotorParams.hpp:40"
   },
   {
    "name": "propeller_diameter",
    "value": "0.2286",
    "unit": "m",
    "where": "RotorParams.hpp:41"
   },
   {
    "name": "propeller_height",
    "value": "0.01",
    "unit": "m",
    "where": "RotorParams.hpp:42"
   },
   {
    "name": "control_signal_filter_tc",
    "value": "0.005",
    "unit": "s",
    "where": "RotorParams.hpp:43"
   },
   {
    "name": "max_thrust",
    "value": "4.179446268",
    "unit": "N",
    "where": "RotorParams.hpp:48"
   },
   {
    "name": "max_torque",
    "value": "0.055562",
    "unit": "N m",
    "where": "RotorParams.hpp:49"
   }
  ],
  "assumptions": "Legacy built-in defaults; Project AirSim has no built-in defaults (all zero) and takes the same numbers from its example robot config.",
  "notes": "Source for the 'defaults to 4.179446268f' comments that survive in Project AirSim rotor.hpp."
 },
 {
  "id": "PA-2-08",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Legacy rotor wrench (thrust and torque with air-density ratio)",
  "simulator": "AirSim (legacy)",
  "vehicle": "multirotor",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/vehicles/multirotor/RotorActuator.hpp",
  "lines": "110-116, 124-126",
  "code": "wrench.force = normal * output_.thrust * air_density_ratio_;\nwrench.torque = normal * output_.torque_scaler * air_density_ratio_;\noutput.speed = sqrt(output.control_signal_filtered * params.max_speed_square);\noutput.thrust = output.control_signal_filtered * params.max_thrust;\noutput.torque_scaler = output.control_signal_filtered * params.max_torque * static_cast<int>(turning_direction);",
  "formula": "F = n T rho/rho_sl;  tau = n Q rho/rho_sl;  omega = sqrt(u_f omega_max^2);  T = u_f T_max;  Q = sigma u_f Q_max",
  "variables": {
   "rho_sl": "EarthUtils::getAirDensity(0) sea-level density [kg/m^3] (RotorActuator.hpp:52)"
  },
  "defaults": [],
  "assumptions": "Identical structure to Project AirSim (PA-2-03/04, PA-3-02).",
  "notes": "Project AirSim is a port of this; only the sea-level density reference constant differs (1.224999 constant vs computed)."
 },
 {
  "id": "PA-3-01",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "First-order low-pass filter on the rotor command (motor/ESC lag)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/first_order_filter.hpp",
  "lines": "60-73",
  "code": "if (dt_sec <= 0.0f) return;\nif (time_constant_ == 0) {\n  output_ = input_;\n  return;\n}\ndouble alpha = std::exp(-dt_sec / time_constant_);\noutput_ = static_cast<T>((output_ * alpha) + (input_ * (1.0 - alpha)));",
  "formula": "y[k+1] = e^(-dt/tau) y[k] + (1 - e^(-dt/tau)) u[k];  tau = 0 gives y = u  (exact zero-order-hold discretisation of tau y' = -y + u)",
  "variables": {
   "y": "filter output [same unit as input]",
   "u": "filter input [same unit]",
   "tau": "time constant (smoothing-tc) [s]",
   "dt": "simulation step [s]"
  },
  "defaults": [
   {
    "name": "smoothing_tc (rotor struct default)",
    "value": "0.0",
    "unit": "s",
    "where": "rotor.hpp:56"
   },
   {
    "name": "smoothing-tc (example quad robot)",
    "value": "0.005",
    "unit": "s",
    "where": "robot_quadrotor_fastphysics.jsonc:239"
   },
   {
    "name": "initial output",
    "value": "0",
    "unit": "-",
    "where": "rotor.cpp:257 Initialize(tc, 0.0, 0.0)"
   }
  ],
  "assumptions": "Used by rotor, tilt, lift-drag control surface, wheel, GPS dilution; the sampling step dt is the scene tick (step-ns, default 20 ms per docs/config_scene.md:79, example configs use 3 ms).",
  "notes": "Header comment lines 16-27 describe the discretisation. It mentions different acceleration and deceleration time constants, but the implementation has only one time constant."
 },
 {
  "id": "PA-3-02",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Rotor speed from the filtered command",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "357-381",
  "code": "first_order_filter_.SetInput(std::clamp(control_signal, 0.0f, 1.0f));\nfirst_order_filter_.UpdateOutput(dt_sec);  // do filtering\ncontrol_signal_filtered = first_order_filter_.GetOutput();\nfloat rotating_speed_raw =\n    std::sqrt(control_signal_filtered * rotor_settings_.max_speed_square);",
  "formula": "u_c = clamp(u,0,1);  u_f = LPF(u_c) (PA-3-01);  omega = sqrt(u_f omega_max^2) = omega_max sqrt(u_f); omega set to 0 if |omega| <= float epsilon",
  "variables": {
   "u": "rotor control signal [-]",
   "u_f": "filtered command [-]",
   "omega": "rotor angular speed [rad/s]",
   "omega_max": "2 pi max_rpm/60 [rad/s]"
  },
  "defaults": [],
  "assumptions": "Rotor speed has no inertia of its own: the lag is the single first-order filter on the command (thrust fraction). Spin-up/spin-down are symmetric. The comment says u_f is proportional to thrust and thrust to speed squared.",
  "notes": "The speed is only used for the visual spin, power, and RotorInfo messages; thrust does NOT read omega."
 },
 {
  "id": "PA-3-03",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Rotor shaft angle (visual spin)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "413-418",
  "code": "auto dangle = rotating_speed_ * dt_sec *\n              static_cast<int>(rotor_settings_.turning_direction);\nangle_cur_ += dangle;\nwhile (angle_cur_ < 0.0f) angle_cur_ += static_cast<float>(2.0 * M_PI);\nwhile (angle_cur_ > static_cast<float>(2.0 * M_PI))\n  angle_cur_ -= static_cast<float>(2.0 * M_PI);",
  "formula": "theta[k+1] = wrap_{0..2 pi}( theta[k] + sigma omega dt )",
  "variables": {
   "theta": "rotor shaft angle [rad]",
   "sigma": "+1 CW, -1 CCW"
  },
  "defaults": [],
  "assumptions": "Cosmetic only (applied to the child-link mesh transform); has no effect on forces.",
  "notes": "Explicit Euler of the shaft angle."
 },
 {
  "id": "PA-3-04",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Rotor command via JSBSim bridge (ESC state read back from JSBSim)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "335-355",
  "code": "const float clamped_cmd =\n    std::clamp(control_signal, 0.0f, 1.0f);\njsbsim_model_->SetPropertyValue(\n    rotor_settings_.jsbsim_cmd,\n    static_cast<double>(clamped_cmd));\nfloat state = GetJSBSimState();",
  "formula": "u_f = JSBSim property state (default to clamp(u,0,1) if state < 0); the first-order filter is bypassed",
  "variables": {
   "u_f": "filtered command used for thrust, speed and torque [-]"
  },
  "defaults": [],
  "assumptions": "Only when jsbsim-cmd and jsbsim-state are configured and a JSBSim model is attached; motor dynamics are then whatever the JSBSim actuator model does (not in this repo).",
  "notes": "The same pattern exists for the control surface (lift_drag_control_surface.cpp:199-208), tilt (tilt.cpp:220-228) and wheel (wheel.cpp:346-359). The TODO at line 332 notes a missing check against using a JSBSim controller."
 },
 {
  "id": "PA-3-05",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Rotor fault injection",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "387-392",
  "code": "if (is_fault_injected_) {\n  rotating_speed_raw = 0.0f;\n  rotating_speed_ = 0.0f;\n  thrust_scalar_ = 0.0f;\n  torque_scalar_ = 0.0f;\n}",
  "formula": "if fault: omega = 0, T = 0, Q = 0",
  "variables": {},
  "defaults": [],
  "assumptions": "A fault zeroes thrust and torque instantly (no ramp-down).",
  "notes": "The flag is set through the actuator service methods (ActuatorImpl, not read here)."
 },
 {
  "id": "PA-3-06",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Tilt actuator angle (tilting rotor pod, VTOL)",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/tilt.cpp",
  "lines": "215-237",
  "code": "auto control_signal_mapped = control_mapper_(control_signal);\nfirst_order_filter_.SetInput(control_signal_mapped);\nfirst_order_filter_.UpdateOutput(dt_sec);\ncontrol_signal_filtered = first_order_filter_.GetOutput();\nradians = settings_.radians_min +\n          (settings_.dradians) * control_signal_filtered;\nquat = AngleAxis(radians, settings_.vec3_axis);",
  "formula": "s = ControlMapper(u) (PA-12-01);  s_f = LPF(s);  angle = angle_min + (angle_max - angle_min) s_f;  q_tilt = AngleAxis(angle, axis)",
  "variables": {
   "angle_min, angle_max": "tilt limits at s_f = 0 and 1 [rad]",
   "axis": "unit rotation axis (normalised on load, tilt.cpp:283)",
   "q_tilt": "quaternion given to the target rotor via SetTilt (robot.cpp:1404-1407)"
  },
  "defaults": [
   {
    "name": "radians_min",
    "value": "0.0",
    "unit": "rad",
    "where": "core_sim/include/core_sim/actuators/tilt.hpp:39"
   },
   {
    "name": "dradians",
    "value": "pi",
    "unit": "rad",
    "where": "tilt.hpp:40"
   },
   {
    "name": "smoothing_tc",
    "value": "0.0",
    "unit": "s",
    "where": "tilt.hpp:41"
   },
   {
    "name": "smoothing-tc (UAM example)",
    "value": "0.5",
    "unit": "s",
    "where": "client/python/example_user_scripts/autonomy/configs/robot_quadtiltrotor_fastphysics_uam.jsonc (tilt actuators, angle-min 0, angle-max 1.57, axis '0 -1 0')"
   }
  ],
  "assumptions": "VTOL tiltrotor: the tilt rotates the rotor thrust vector via q_tilt in PA-2-04. No tilt dynamics other than the first-order filter.",
  "notes": "JSBSim variant at tilt.cpp:220-228 clamps the JSBSim state to [0,1]."
 },
 {
  "id": "PA-4-01",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "Air-density scaling of rotor thrust and torque",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actor/robot.cpp",
  "lines": "1424-1427",
  "code": "float air_density_ratio =\n    environment_.env_info.air_density / EarthUtils::kSeaLevelAirDensity;\n\nrotor_ref.SetAirDensityRatio(air_density_ratio);",
  "formula": "air_density_ratio = rho(h) / 1.224999;  F and tau of every rotor are multiplied by this ratio (rotor.cpp:398-402)",
  "variables": {
   "rho(h)": "air density at the vehicle altitude [kg/m^3] (PA-1-06)",
   "ratio": "dimensionless scale on thrust and torque [-]"
  },
  "defaults": [
   {
    "name": "air_density_ratio_ (initial)",
    "value": "1.0",
    "unit": "-",
    "where": "rotor.cpp:238"
   }
  ],
  "assumptions": "Thrust and torque scale linearly with density (as T = C_T rho n^2 D^4 at fixed n). Rotor speed itself is not changed by density.",
  "notes": "The ratio uses env_info from the previous physics tick."
 },
 {
  "id": "PA-4-02",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "Inflow, advance ratio, ground effect, blade flapping, slipstream, rotor-wind interaction",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp; core_sim/src/actuators/rotor.cpp",
  "lines": "fast_physics.cpp:501-504; rotor.cpp:319-429",
  "code": "//   TODO Calculate air flow velocity at each link considering wind\n//   from the environment and possibly slipstream from rotor actuators",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "Thrust depends only on the filtered command and the density ratio; it does not depend on vehicle velocity, inflow, wind, or ground proximity.",
  "notes": "The only trace is the TODO at fast_physics.cpp:501-504 about slipstream. No C_T(J), no induced velocity, no ground effect, no hub forces, no motor back-EMF or voltage model (grep of rotor.cpp, fast_physics.cpp, core_sim/src/actuators)."
 },
 {
  "id": "PA-5-01",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Drag-face geometry: six cuboid faces with area and drag factor",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "203-239",
  "code": "cross_section_areas += link.GetInertial().GetCrossSectionAreas();\nfloat drag_factor = links[0].GetInertial().GetDragCoefficient() / 2;\nconst Vector3& body_box = links[0].GetInertial().GetBodyBox();\ndrag_faces.emplace_back(Vector3(0, 0, -body_box.z() / 2), Vector3(0, 0, -1),\n                        cross_section_areas.z(), drag_factor);",
  "formula": "A_total = sum over all links of (A_x, A_y, A_z);  k = Cd_root / 2;  six faces at +-box/2 with normals +-x, +-y, +-z, face area A_total.x / .y / .z, drag factor k",
  "variables": {
   "A_total": "summed cross-section areas of the root link and every other link (e.g. propeller cylinders) [m^2]",
   "Cd_root": "drag coefficient of the ROOT link only (links[0]) [-]",
   "box": "body_box_xyz from the root link [m]"
  },
  "defaults": [
   {
    "name": "drag-coefficient (example quad frame and props)",
    "value": "0.325",
    "unit": "-",
    "where": "robot_quadrotor_fastphysics.jsonc:17, 49, 82, 115, 148"
   },
   {
    "name": "drag-coefficient (UAM example frame)",
    "value": "0.04",
    "unit": "-",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:17"
   }
  ],
  "assumptions": "Comment at lines 192-199 assumes a drone: root link is a cuboid, other links are cylinders with normal along z; NED with -z up. Only the root link's Cd is used for all faces; other links' drag-coefficient is ignored.",
  "notes": "Box areas: (y z, x z, x y) (inertial.cpp:343-345); cylinder areas: (2 r L, 2 r L, pi r^2) (inertial.cpp:355-356). The six faces have fixed positions, but positions are unused (no drag torque)."
 },
 {
  "id": "PA-5-02",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Cross-section areas from geometry (box and cylinder)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/link/inertial.cpp",
  "lines": "343-345, 355-356",
  "code": "cross_section_areas = {body_box.y() * body_box.z(),\n                       body_box.x() * body_box.z(),\n                       body_box.x() * body_box.y()};\ncross_section_areas = {2 * radius * length, 2 * radius * length,\n                       static_cast<float>(M_PI) * radius * radius};",
  "formula": "box: (A_x, A_y, A_z) = (b_y b_z, b_x b_z, b_x b_y);  cylinder: (2 r L, 2 r L, pi r^2)",
  "variables": {
   "b_x,b_y,b_z": "box size along body axes [m]",
   "r, L": "cylinder radius and length [m]",
   "A_x": "area seen when moving along body x [m^2]"
  },
  "defaults": [
   {
    "name": "propeller cylinder (example)",
    "value": "r = 0.1143 m, L = 0.01 m",
    "unit": "m",
    "where": "robot_quadrotor_fastphysics.jsonc:52-55"
   },
   {
    "name": "frame box (example)",
    "value": "0.180 x 0.110 x 0.040",
    "unit": "m",
    "where": "robot_quadrotor_fastphysics.jsonc:11-13"
   }
  ],
  "assumptions": "The cylinder axis is the body z axis. 'cross-section-areas-xyz' type sets the areas directly (inertial.cpp:361-365); 'lift-drag' type leaves them zero (inertial.cpp:366-372).",
  "notes": "drag-coefficient is read at inertial.cpp:437-438 with no default (the GetNumber call without a default)."
 },
 {
  "id": "PA-5-03",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Quadratic drag force on the body (face-by-face)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "711-743",
  "code": "auto net_ave_vel_lin_body = ave_vel_lin - wind_velocity;\nconst Vector3 ave_vel_lin_body = PhysicsUtils::TransformVectorToBodyFrame(\n    net_ave_vel_lin_body, orientation);\nconst float vel_normal = drag_face.normal.dot(ave_vel_lin_body);\nif (vel_normal > kDragMinVelocity) {\n  const Vector3 drag_force = ave_vel_lin_body.stableNormalized() *\n                             (-drag_face.drag_factor * drag_face.area *",
  "formula": "v_rel = R^T (v_avg - w);  for each face i with unit normal n_i: v_n = n_i . v_rel;  if v_n > 0.1 m/s:  F_i = -(Cd/2) A_i rho v_n^2 * v_rel/|v_rel|;  F_drag_world = R sum_i F_i;  no torque",
  "variables": {
   "v_avg": "average world velocity over the step = v + a dt/2 [m/s]",
   "w": "wind velocity, world NED [m/s]",
   "R": "body-to-world rotation from the pose quaternion [-]",
   "n_i": "face normal in the body frame [-]",
   "A_i": "face area [m^2]",
   "Cd": "root-link drag coefficient [-]",
   "rho": "air density [kg/m^3]",
   "v_n": "component of relative velocity along the face normal [m/s]"
  },
  "defaults": [
   {
    "name": "kDragMinVelocity",
    "value": "0.1",
    "unit": "m/s",
    "where": "physics/include/fast_physics.hpp:134"
   }
  ],
  "assumptions": "The magnitude uses only the normal component of the relative velocity squared; the DIRECTION is the full relative-velocity direction (not the face normal). Only faces facing into the flow (v_n > 0) generate drag. The linear term b v is neglected (comment cites Taylor, Classical Mechanics p.44). Angular velocity of the faces is ignored (comment at lines 717-719).",
  "notes": "Output wrench torque is zero (comment lines 733-736). Legacy AirSim applied the force along the FACE NORMAL (legacy FastPhysicsEngine.hpp:288), so the two differ for oblique flow."
 },
 {
  "id": "PA-5-04",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Angular (rotational) drag",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/constant.hpp; physics/src/fast_physics.cpp",
  "lines": "constant.hpp:418-421; fast_physics.cpp:733-736",
  "code": "static constexpr const char* linear_drag_coefficient =\n    \"linear-drag-coefficient\";\nstatic constexpr const char* angular_drag_coefficient =\n    \"angular-drag-coefficient\";",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "The two JSON key strings are declared in constant.hpp but a repo-wide code search finds no other use; no config loader reads them and no physics code uses them. fast_physics.cpp:733-736 states that no drag torque is applied.",
  "notes": "A pitch/roll/yaw-rate damping term does not exist in Project AirSim FastPhysics. The legacy engine has one (PA-5-06)."
 },
 {
  "id": "PA-5-05",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Legacy drag-face builder: frame box plus propeller areas, linear_drag_coefficient/2",
  "simulator": "AirSim (legacy)",
  "vehicle": "multirotor",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/vehicles/multirotor/MultiRotorPhysicsBody.hpp",
  "lines": "197-216",
  "code": "real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;\nreal_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;\nVector3r drag_factor_unit = Vector3r(\n    front_back_area + rotors_.size() * propeller_xsection,\n    left_right_area + rotors_.size() * propeller_xsection,\n    top_bottom_area + rotors_.size() * propeller_area) * params.linear_drag_coefficient / 2;",
  "formula": "A_prop = pi D^2 (sic: pi D^2, not pi D^2/4);  A_xsec = pi D h;  drag_factor = (A_face + N_rotors A_prop_part) Cd/2, per axis",
  "variables": {
   "D": "propeller diameter [m]",
   "h": "propeller height 0.01 [m]",
   "N_rotors": "rotor count",
   "Cd": "linear_drag_coefficient [-]"
  },
  "defaults": [
   {
    "name": "linear_drag_coefficient",
    "value": "1.3/4 = 0.325",
    "unit": "-",
    "where": "AirLib/include/vehicles/multirotor/MultiRotorParams.hpp:47"
   },
   {
    "name": "angular_drag_coefficient",
    "value": "= linear_drag_coefficient",
    "unit": "-",
    "where": "MultiRotorParams.hpp:52"
   }
  ],
  "assumptions": "Note the propeller area used is pi D^2 (diameter squared times pi), four times a disc area; the code comment at 195-196 says it approximates the exposed area.",
  "notes": "Comment at MultiRotorParams.hpp:48-51: 1.3 from klsin.bpmsg.com divided by 4 for a streamlined frame. angular_drag_coefficient is declared (MultiRotorParams.hpp:52) but createDragVertices uses only linear_drag_coefficient; the rotational effect comes from the vertex-velocity term in PA-5-06."
 },
 {
  "id": "PA-5-06",
  "group": 5,
  "group_name": "body aerodynamics (drag faces, angular drag)",
  "quantity": "Legacy drag wrench including rotation-induced drag and drag torque",
  "simulator": "AirSim (legacy)",
  "vehicle": "multirotor",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/physics/FastPhysicsEngine.hpp",
  "lines": "279-293",
  "code": "const Vector3r relative_vel = linear_vel - wind_world;\nconst Vector3r linear_vel_body = VectorMath::transformToBodyFrame(relative_vel, orientation);\nconst Vector3r vel_vertex = linear_vel_body + angular_vel_body.cross(vertex.getPosition());\nconst real_T vel_comp = vertex.getNormal().dot(vel_vertex);\nconst Vector3r drag_force = vertex.getNormal() * (-vertex.getDragFactor() * air_density * vel_comp * vel_comp);\nconst Vector3r drag_torque = vertex.getPosition().cross(drag_force);",
  "formula": "v_vertex = R^T (v - w) + omega x r_vertex;  v_n = n . v_vertex;  if v_n > 0.1: F = -n k rho v_n^2,  tau = r_vertex x F",
  "variables": {
   "omega": "body angular velocity [rad/s]",
   "r_vertex": "drag face position in the body frame [m]",
   "k": "drag factor = Cd/2 times the area terms of PA-5-05 [m^2]",
   "n": "face unit normal [-]"
  },
  "defaults": [
   {
    "name": "kDragMinVelocity",
    "value": "0.1",
    "unit": "m/s",
    "where": "FastPhysicsEngine.hpp:456"
   }
  ],
  "assumptions": "Force is along the face normal (opposes the normal velocity component). Faces are 0.02 to 0.2 m from the CG, so the torque is a (small) rotational damping.",
  "notes": "This is the missing 'angular drag' of Project AirSim (PA-5-04). Wind sign: relative_vel = v - wind, the same as Project AirSim."
 },
 {
  "id": "PA-6-01",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Lift-drag link: relative air velocity and skip conditions",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "773-797",
  "code": "const Vector3& cp_vel_linear_world = body_velocity - wind_velocity;\nif (cp_vel_linear_world.stableNorm() <= kDragMinVelocity) continue;\nconst Vector3 forward_world = PhysicsUtils::TransformVectorToWorldFrame(\n    lift_drag_params.forward_xyz, link_orientation_world);\nif (forward_world.dot(cp_vel_linear_world) <= 0.0f) continue;",
  "formula": "v_cp = v_avg - w (world);  skip the link if |v_cp| <= 0.1 m/s or f . v_cp <= 0 (flow from behind)",
  "variables": {
   "v_cp": "velocity of the centre of pressure relative to the air, world [m/s]",
   "f": "link forward unit vector in world [-]",
   "w": "wind velocity [m/s]"
  },
  "defaults": [
   {
    "name": "kDragMinVelocity",
    "value": "0.1",
    "unit": "m/s",
    "where": "fast_physics.hpp:134"
   }
  ],
  "assumptions": "Centre-of-pressure velocity is taken equal to the body CG velocity (TODO at 775-777: link angular velocity not included). No lift or drag if the flow comes from behind the link, i.e. no reverse-flow aerodynamics. VTOL tiltrotor and tailsitter use this same code.",
  "notes": "v_avg = v + a dt/2 comes from CalcNextKinematicsNoCollision (line 495-496), passed as body_velocity."
 },
 {
  "id": "PA-6-02",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Lift-drag plane and in-plane velocity (sweep correction)",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "800-815",
  "code": "const Vector3 lift_drag_plane_normal =\n    forward_world.cross(upward_world).stableNormalized();\nconst Vector3 vel_normal_to_lift_drag_plane =\n    cp_vel_linear_world.dot(lift_drag_plane_normal) *\n    lift_drag_plane_normal;\nconst Vector3 vel_in_lift_drag_plane =\n    cp_vel_linear_world - vel_normal_to_lift_drag_plane;",
  "formula": "n = (f x u)/|f x u|;  v_p = v_cp - (v_cp . n) n",
  "variables": {
   "f": "forward unit vector, default (1,0,0) [-]",
   "u": "upward (lift) unit vector, default (0,0,-1) [-]",
   "n": "normal to the lift-drag plane = pitching-moment axis [-]",
   "v_p": "relative air velocity projected onto the lift-drag plane [m/s]"
  },
  "defaults": [
   {
    "name": "forward_xyz",
    "value": "(1, 0, 0)",
    "unit": "-",
    "where": "physics_common_types.hpp:210"
   },
   {
    "name": "upward_xyz",
    "value": "(0, 0, -1)",
    "unit": "-",
    "where": "physics_common_types.hpp:211"
   }
  ],
  "assumptions": "Spanwise flow is removed (strip-theory with sweep). With f=(1,0,0), u=(0,0,-1): n = x cross (-z) = +y (body right), so a positive pitching moment is nose-up.",
  "notes": "The link frame orientation comes from the transform tree (lines 786-791)."
 },
 {
  "id": "PA-6-03",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Angle of attack and force directions",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "820-838",
  "code": "const Vector3 lift_direction =\n    lift_drag_plane_normal.cross(vel_in_lift_drag_plane).stableNormalized();\nconst Vector3 drag_direction = (-vel_in_lift_drag_plane).stableNormalized();\nconst float cos_alpha =\n    std::clamp(lift_direction.dot(upward_world), -1.0f, 1.0f);\nfloat alpha = (lift_direction.dot(forward_world) >= 0.0f)\n                  ? lift_drag_params.alpha_0 + std::acos(cos_alpha)\n                  : lift_drag_params.alpha_0 - std::acos(cos_alpha);",
  "formula": "l_hat = (n x v_p)/|n x v_p|;  d_hat = -v_p/|v_p|;  alpha = alpha_0 + sign(l_hat . f) acos(clamp(l_hat . u));  then alpha wrapped into [-pi/2, pi/2] by adding or subtracting pi (lines 836-838)",
  "variables": {
   "alpha": "angle of attack, including the alpha_0 offset [rad]",
   "alpha_0": "AOA shift so the coefficient curves cross zero at the zero-lift AOA [rad]",
   "l_hat": "lift direction (perpendicular to flow, in plane) [-]",
   "d_hat": "drag direction (against flow) [-]"
  },
  "defaults": [],
  "assumptions": "The sign of alpha is taken from l_hat . f. Wrapping gives the usual flat-plate symmetry (alpha and alpha + pi equivalent). The sign convention of alpha_0 follows the code ('+alpha_0'): the header (physics_common_types.hpp:170-172) describes alpha_0 as the shift moving the curves right.",
  "notes": "No Mach, Reynolds or sideslip dependence."
 },
 {
  "id": "PA-6-04",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Dynamic pressure of the in-plane flow",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "843-846",
  "code": "const float spd_in_lift_drag_plane = vel_in_lift_drag_plane.stableNorm();\n\nconst float q =\n    0.5f * air_density * spd_in_lift_drag_plane * spd_in_lift_drag_plane;",
  "formula": "q = 1/2 rho |v_p|^2",
  "variables": {
   "q": "dynamic pressure [Pa]",
   "rho": "air density [kg/m^3]",
   "v_p": "in-plane relative velocity [m/s]"
  },
  "defaults": [],
  "assumptions": "Uses local air density (altitude dependent), unlike the rotor which uses a ratio.",
  "notes": ""
 },
 {
  "id": "PA-6-05",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Lift coefficient (linear, then post-stall slope) and lift force",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "850-871",
  "code": "if (alpha > alpha_stall) {\n  cl = cla * alpha_stall + cla_stall * (alpha - alpha_stall);\n  cl = std::max(cl, 0.0);\n} else if (alpha < -alpha_stall) {\n  cl = -cla * alpha_stall + cla_stall * (alpha + alpha_stall);\n  cl = std::min(cl, 0.0);",
  "formula": "CL = C_La alpha for |alpha| <= alpha_s;  CL = C_La alpha_s + C_La,s (alpha - alpha_s), floored at 0, for alpha > alpha_s;  CL = -C_La alpha_s + C_La,s (alpha + alpha_s), capped at 0, for alpha < -alpha_s;  CL += (dCL/ddelta) delta;  F_lift = CL q S l_hat",
  "variables": {
   "C_La": "c_lift_alpha, lift slope below stall [1/rad]",
   "C_La,s": "c_lift_alpha_stall, post-stall slope (negative = falling) [1/rad]",
   "alpha_s": "alpha_stall, includes the alpha_0 shift [rad]",
   "delta": "control-surface angle [rad]",
   "dCL/ddelta": "control_surface_cl_per_rad [1/rad]",
   "S": "lift-drag area [m^2]"
  },
  "defaults": [
   {
    "name": "c_lift_alpha, c_lift_alpha_stall, alpha_0, alpha_stall, area (struct defaults)",
    "value": "all 0.0",
    "unit": "-",
    "where": "core_sim/include/core_sim/physics_common_types.hpp:172-192"
   },
   {
    "name": "UAM wing example",
    "value": "alpha-0 0.06, alpha-stall 0.64, c-lift-alpha 0.5, c-lift-alpha-stall -0.8, area 5.0, control-surface-cl-per-rad -0.5",
    "unit": "rad, 1/rad, m^2",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:331-340"
   }
  ],
  "assumptions": "Lift is piecewise-linear in alpha with a linear post-stall branch; the coefficient value is the one at the stall end (not C_La alpha). Control surface adds to CL linearly with its angle (line 868).",
  "notes": "The control-surface add is applied after the stall guard, so it can drive CL negative post-stall."
 },
 {
  "id": "PA-6-06",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Drag coefficient and drag force",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "875-899",
  "code": "cd = cda * alpha;\ncd = std::abs(cd);\ncd += std::abs(lift_drag_params.control_surface_cd_per_rad * control_angle);\nconst Vector3 drag_force = cd * q * lift_drag_params.area * drag_direction;",
  "formula": "CD0 = C_Da alpha (|alpha| <= alpha_s); beyond alpha_s: sign * C_Da alpha_s + C_Da,s (alpha -/+ alpha_s), floored/capped at 0;  CD = |CD0| + |dCD/ddelta delta|;  F_drag = CD q S d_hat",
  "variables": {
   "C_Da": "c_drag_alpha [1/rad]",
   "C_Da,s": "c_drag_alpha_stall [1/rad]",
   "dCD/ddelta": "control_surface_cd_per_rad [1/rad]"
  },
  "defaults": [
   {
    "name": "c_drag_alpha, c_drag_alpha_stall (struct)",
    "value": "0.0",
    "unit": "1/rad",
    "where": "physics_common_types.hpp:184-185"
   },
   {
    "name": "UAM wing example",
    "value": "c-drag-alpha 0.6, c-drag-alpha-stall -0.9, control-surface-cd-per-rad 0.0",
    "unit": "1/rad",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:335-341"
   }
  ],
  "assumptions": "There is NO separate constant zero-lift (parasitic) drag term: CD = |C_Da alpha| with alpha including the alpha_0 offset (so drag vanishes only where alpha = 0, i.e. at geometric AOA = -alpha_0). Drag magnitude is forced positive in the direction against the flow.",
  "notes": "Wing-level drag is separate from body drag-face drag (PA-5-03): for a link with 'lift-drag' aerodynamics, the cross-section areas are zero so no face drag is generated from that link."
 },
 {
  "id": "PA-6-07",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Pitching moment coefficient and moment",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "903-927",
  "code": "cm = cma * alpha;\ncm +=lift_drag_params.control_surface_cm_per_rad * control_angle;\nconst Vector3 moment_world =\n    cm * q * lift_drag_params.area * moment_direction;\nconst Vector3 moment_body = PhysicsUtils::TransformVectorToBodyFrame(\n    moment_world, body_orientation);",
  "formula": "CM as for CL with C_Ma, C_Ma,s, plus (dCM/ddelta) delta;  M = CM q S n (n = plane normal, PA-6-02), rotated to the body frame",
  "variables": {
   "C_Ma": "c_moment_alpha [1/rad]",
   "dCM/ddelta": "control_surface_cm_per_rad [1/rad]",
   "M": "aerodynamic moment about the centre of pressure [N m]"
  },
  "defaults": [
   {
    "name": "c_moment_alpha, c_moment_alpha_stall",
    "value": "0.0",
    "unit": "1/rad",
    "where": "physics_common_types.hpp:188-189"
   },
   {
    "name": "UAM elevator example",
    "value": "control-surface-cm-per-rad 2.0, area 0.5",
    "unit": "1/rad, m^2",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:421-437"
   }
  ],
  "assumptions": "The moment is not multiplied by a chord length (CM q S has units of force, so CM here is effectively in metres: CM = C_m * chord). Direction is the lift-drag plane normal.",
  "notes": "Units: moment = cm * q * area * unit vector, so cm carries length units [m] as written."
 },
 {
  "id": "PA-6-08",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Lift-drag wrench assembly: force in world, torque about the CG",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "932-957",
  "code": "const Vector3 total_force_world = lift_force + drag_force;\nconst Vector3 cg_to_cp_translation =\n    lift_drag_link.GetInertial().GetOrigin().translation_ +\n    lift_drag_params.center_pressure_xyz;\nconst Vector3 torque_from_force_body =\n    cg_to_cp_translation.cross(total_force_body);\nlift_drag_wrench.torque += moment_body;",
  "formula": "F = F_lift + F_drag (world);  r = r_link_origin + r_cp;  tau = r x (R^T F) + R^T M;  summed over all lift-drag links",
  "variables": {
   "r_link_origin": "origin translation of the link's inertial frame [m]",
   "r_cp": "center_pressure_xyz offset in the link frame [m]",
   "R": "body-to-world rotation from the body orientation [-]"
  },
  "defaults": [
   {
    "name": "center_pressure_xyz",
    "value": "(0,0,0)",
    "unit": "m",
    "where": "physics_common_types.hpp:204"
   }
  ],
  "assumptions": "The lever arm is the link's own origin translation (from its inertial origin) plus the CP offset, i.e. it assumes the link origin is expressed relative to the CG/root frame. Link orientation is not applied to r_cp.",
  "notes": "Wrench added to external + drag in CalcNextKinematicsNoCollision (line 521)."
 },
 {
  "id": "PA-6-09",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Control-surface actuator: command to surface angle",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/lift_drag_control_surface.cpp",
  "lines": "192-216",
  "code": "auto control_signal = control_signals[0];\nfirst_order_filter_.SetInput(control_signal);\nfirst_order_filter_.UpdateOutput(SimClock::Get()->NanosToSec(sim_dt_nanos));\ncontrol_signal_effective = first_order_filter_.GetOutput();\ncontrol_angle_ = settings_.rotation_rate * control_signal_effective;",
  "formula": "delta = rotation_rate * LPF(u),  u in [-1, 1] (no clamp in this file)",
  "variables": {
   "delta": "control surface angle fed to PA-6-05..07 [rad]",
   "rotation_rate": "rotation_rate: surface angle per 1.0 of command [rad]",
   "u": "control signal [-]"
  },
  "defaults": [
   {
    "name": "rotation_rate (struct default)",
    "value": "0.0",
    "unit": "rad",
    "where": "core_sim/include/core_sim/actuators/lift_drag_control_surface.hpp:38"
   },
   {
    "name": "smoothing_tc (struct default)",
    "value": "0.0",
    "unit": "s",
    "where": "lift_drag_control_surface.hpp:39"
   },
   {
    "name": "rotation-rate (UAM example)",
    "value": "0.524",
    "unit": "rad (30 deg)",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc (aileron and elevator actuators)"
   },
   {
    "name": "smoothing-tc (UAM example)",
    "value": "0.005",
    "unit": "s",
    "where": "same file"
   }
  ],
  "assumptions": "Servo model: a first-order lag, no rate limit, no angle limit, no deadband. Surface effect is added to the wing coefficients at the surface's PARENT link (the wing it moves), via lift_drag_control_angles_ (fast_physics.cpp:59-68).",
  "notes": "JSBSim path (lines 199-208) takes the JSBSim state instead of the filter."
 },
 {
  "id": "PA-6-10",
  "group": 6,
  "group_name": "wings / fixed-wing / VTOL aero (lift_drag_control_surface)",
  "quantity": "Lift-drag parameter structure and example wing values",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/physics_common_types.hpp",
  "lines": "164-212",
  "code": "float alpha_0 = 0.0f;\nfloat alpha_stall = 0.0f;\nfloat c_lift_alpha = 0.0f;\nfloat c_drag_alpha = 0.0f;\nfloat c_moment_alpha = 0.0f;\nVector3 upward_xyz = {0.0f, 0.0f, -1.0f};  // local NED frame unit vector",
  "formula": "parameter set (alpha_0, alpha_stall, C_La, C_La,s, C_Da, C_Da,s, C_Ma, C_Ma,s, S, dCL/ddelta, dCD/ddelta, dCM/ddelta, r_cp, f, u)",
  "variables": {
   "alpha_stall": "AOA at which the slope changes; includes the alpha_0 shift [rad]",
   "area": "surface area generating lift-drag [m^2]",
   "control_surface_c*_per_rad": "coefficient added per radian of surface deflection [1/rad]",
   "center_pressure_xyz": "offset from link origin where force acts, local NED [m]"
  },
  "defaults": [
   {
    "name": "all coefficients",
    "value": "0.0",
    "unit": "-",
    "where": "physics_common_types.hpp:172-200"
   },
   {
    "name": "UAM wing (two wings)",
    "value": "alpha-0 0.06; alpha-stall 0.64; c-lift-alpha 0.5; c-lift-alpha-stall -0.8; c-drag-alpha 0.6; c-drag-alpha-stall -0.9; c-moment-alpha 0; area 5.0; cl/cd/cm per rad -0.5/0/-0.1",
    "unit": "-",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:329-345, 375-391"
   },
   {
    "name": "UAM elevator",
    "value": "alpha-0 0; alpha-stall 0.3; all alpha coefficients 0; area 0.5; control-surface-cm-per-rad 2.0",
    "unit": "-",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:421-437"
   }
  ],
  "assumptions": "A link enables lift-drag when its aerodynamics type is 'lift-drag' (inertial.cpp:366-372). Parameters are read by the loader at inertial.cpp:374-431.",
  "notes": "Only one JSBSim config (x8_skywalker, c310) and one UAM config ship as fixed-wing examples; no plain fast-physics fixed-wing example exists in the tree searched."
 },
 {
  "id": "PA-7-01",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind velocity state and units",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/environment.hpp",
  "lines": "environment.hpp:24; scene.cpp:905-911, 1393-1412",
  "code": "inline static Vector3 wind_velocity = Vector3::Zero();\nbool Scene::Impl::SetWindVelocity(float v_x, float v_y, float v_z) {\n  Environment::wind_velocity = Vector3(v_x, v_y, v_z);",
  "formula": "w = (w_N, w_E, w_D), constant until changed by the scene config 'wind: velocity: \"x y z\"' or the SetWindVelocity API",
  "variables": {
   "w": "velocity of the AIR MASS in the world NED frame [m/s]"
  },
  "defaults": [
   {
    "name": "wind_velocity",
    "value": "(0,0,0)",
    "unit": "m/s",
    "where": "environment.hpp:24; scene.cpp:1402"
   },
   {
    "name": "example config",
    "value": "\"velocity\": \"0.0 10.0 0.0\" (10 m/s along +y East)",
    "unit": "m/s",
    "where": "client/python/example_user_scripts/sim_config/scene_drone_wind.jsonc:25-27"
   }
  ],
  "assumptions": "Wind is a STATIC (inline static) member, so it is global to every robot in the process. Uniform in space and constant in time unless the API is called. It is stated as a velocity vector, not a force. Sign convention: positive w_N means the air moves toward north; every consumer computes relative velocity as v - w (PA-5-03, PA-6-01, PA-7-04, legacy PA-5-06), so a drone at rest in a +N wind is pushed toward +N.",
  "notes": "Config key 'wind' -> 'velocity' is read by scene.cpp:1393-1412 and shown in scene_drone_wind.jsonc (docs/config_scene.md has no wind section); transition_from_airsim.md:111-113 says 'Not configurable. Use world.SetWindVelocity()', and line 421 says 'Wind isn't implemented in the physics yet, but the API exists' - contradicted by PA-7-02/03. Also in: core_sim/src/scene.cpp. Quote condensed from several code lines."
 },
 {
  "id": "PA-7-02",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind in body drag (and the contradiction with the 'not supported' comment)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "491-493 (comment), 711 (use)",
  "code": "//   Step 1a - Calculate air flow velocity at each link\n//   Currently, wind is not supported so assume all air flow velocity comes\n//   from the body motion velocity.\nauto net_ave_vel_lin_body = ave_vel_lin - wind_velocity;",
  "formula": "v_rel = v_avg - w   (world frame), then rotated to the body frame: v_rel,b = R^T v_rel (lines 711-713)",
  "variables": {
   "v_avg": "average velocity over the step [m/s]",
   "w": "wind [m/s]",
   "v_rel,b": "air-relative velocity in body frame [m/s]"
  },
  "defaults": [],
  "assumptions": "CONTRADICTION: the comment at 491-493 says wind is unsupported and the TODO at 501-504 says wind is to be considered later, yet line 711 subtracts env_wind_velocity_ (passed in at 509-511) from the velocity used for body drag, and line 778 does the same for lift-drag. So wind IS applied to drag and lift-drag; the comment is stale. The docs (transition_from_airsim.md:421) are also stale. The variable is named net_ave_vel_lin_body but is in the WORLD frame at line 711 (it is rotated at 712-713).",
  "notes": "The wind vector is assumed to be in the world NED frame. The wind effect is observed by the example wind.py: the drone 'tilts proportionally to wind speed & direction' because the simple-flight position hold fights the drag force."
 },
 {
  "id": "PA-7-03",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind in wing lift and drag",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "778",
  "code": "const Vector3& cp_vel_linear_world = body_velocity - wind_velocity;",
  "formula": "v_cp = v_avg - w (world); then PA-6-01..08",
  "variables": {
   "w": "wind velocity, world NED [m/s]"
  },
  "defaults": [],
  "assumptions": "Same sign convention as PA-7-02. Wind changes angle of attack, dynamic pressure and the skip conditions of every lift-drag link.",
  "notes": "The velocity passed in is v_avg (line 495-496), the same average velocity used for body drag."
 },
 {
  "id": "PA-7-04",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind in the airspeed sensor",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/airspeed.cpp",
  "lines": "236-238",
  "code": "velocity_in_air_true =\n    (position - position_last_) / dtSec -\n    ground_truth_.environment->wind_velocity;  // Get true air velocity",
  "formula": "v_air = (p[k] - p[k-1])/dt - w",
  "variables": {
   "p": "world position [m]",
   "w": "wind [m/s]"
  },
  "defaults": [],
  "assumptions": "Ground velocity from finite-differenced position; same sign convention (relative velocity = ground velocity - wind).",
  "notes": "A TODO at 234-235 suggests using twist.linear instead of differencing the position."
 },
 {
  "id": "PA-7-05",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind initial condition for JSBSim",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/jsbsim_physics.cpp",
  "lines": "164-166",
  "code": "auto wind_velocity = sim_robot_.GetEnvironment().wind_velocity;\ninit.SetWindNEDFpsIC(wind_velocity.x() * MathUtils::meters_to_feets, wind_velocity.y() * MathUtils::meters_to_feets,\n                     wind_velocity.z() * MathUtils::meters_to_feets);",
  "formula": "w_fps = w_mps / 0.3048  (NED components)",
  "variables": {
   "w": "wind, NED [m/s -> ft/s]"
  },
  "defaults": [
   {
    "name": "feets_to_meters",
    "value": "0.3048",
    "unit": "m/ft",
    "where": "core_sim/include/core_sim/math_utils.hpp:130"
   }
  ],
  "assumptions": "Wind is applied to JSBSim only as an initial condition at body creation (line 164 sits in the init function); later API wind changes are not forwarded in the code read (env_wind_velocity_ is stored at jsbsim_physics.cpp:276 but a search of that file finds no further use).",
  "notes": "JSBSim itself (external library) applies wind inside its own aerodynamics."
 },
 {
  "id": "PA-7-06",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Gusts, turbulence (Dryden/von Karman), wind shear, wind in rotor model",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/scene.cpp; core_sim/include/core_sim/environment.hpp",
  "lines": "n/a",
  "code": "inline static Vector3 wind_velocity = Vector3::Zero();",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "Wind is a single constant vector. A repo-wide code search for 'wind' lists only the files already cited above; there is no stochastic gust model, no altitude profile, no turbulence filter, and wind does not enter the rotor model or the IMU/GPS/baro.",
  "notes": "The example wind.py changes the wind with API calls to emulate gusts, and uses set_external_force for an alternative force-based wind (docstring lines 6-7)."
 },
 {
  "id": "PA-7-07",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind in legacy AirSim drag (sign check)",
  "simulator": "AirSim (legacy)",
  "vehicle": "multirotor",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/physics/FastPhysicsEngine.hpp",
  "lines": "278-280",
  "code": "// Use relative velocity of the body wrt wind\nconst Vector3r relative_vel = linear_vel - wind_world;\nconst Vector3r linear_vel_body = VectorMath::transformToBodyFrame(relative_vel, orientation);",
  "formula": "v_rel = v - w_world",
  "variables": {
   "w_world": "wind, world frame [m/s]"
  },
  "defaults": [
   {
    "name": "wind (constructor default)",
    "value": "Zero",
    "unit": "m/s",
    "where": "FastPhysicsEngine.hpp:25"
   }
  ],
  "assumptions": "Same sign as Project AirSim: the comment names it 'relative velocity of the body wrt wind'.",
  "notes": "Legacy wind is set via setWind (line 68-71). No stale 'unsupported' comment exists in the legacy file."
 },
 {
  "id": "PA-8-01",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Linear acceleration from total wrench and gravity",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "521-525",
  "code": "const Wrench total_wrench = external_wrench + drag_wrench + lift_drag_wrench;\n\nkin_no_collision.accels.linear = total_wrench.force * mass_inv + env_gravity;",
  "formula": "a = (F_rotors + F_ext + F_drag + F_liftdrag)/m + g_vec   (world frame; g_vec = (0,0,+g))",
  "variables": {
   "a": "world-frame linear acceleration [m/s^2]",
   "F": "force components, world frame [N]",
   "m": "body mass [kg]",
   "g_vec": "gravity vector [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Newton's second law with the mass of the ROOT link only (PA-8-02) and constant mass. Gravity is added as an acceleration; there is no buoyancy, no added mass, no Coriolis/centrifugal term, no Earth rotation.",
  "notes": "Result is stored as accels.linear and later read by the IMU as a - g (PA-14-01)."
 },
 {
  "id": "PA-8-02",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Body mass and its inverse",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "fast_physics.cpp:71; base_physics.hpp:48-55, 79",
  "code": "SetMass(root_link.GetInertial().GetMass());  // TODO accumulate link masses?\nif (mass < kMinMass) {\n  mass = kMinMass;\nmass_ = mass;\nmass_inv_ = 1.0f / mass;",
  "formula": "m = max(m_root, 1e-6);  1/m",
  "variables": {
   "m_root": "mass of link 0 (root link) [kg]",
   "kMinMass": "1e-6 [kg]"
  },
  "defaults": [
   {
    "name": "mass (example quad frame)",
    "value": "1.0",
    "unit": "kg",
    "where": "robot_quadrotor_fastphysics.jsonc:7"
   },
   {
    "name": "mass (each example propeller link)",
    "value": "0.055",
    "unit": "kg (ignored for total mass)",
    "where": "robot_quadrotor_fastphysics.jsonc:44"
   },
   {
    "name": "mass (UAM example frame)",
    "value": "10.0",
    "unit": "kg",
    "where": "robot_quadtiltrotor_fastphysics_uam.jsonc:7"
   }
  ],
  "assumptions": "Only the root link mass is the body mass; the other link masses (e.g. 4 x 0.055 kg propellers) are NOT added to m (TODO in the code) but their inertia terms are (PA-8-03). So the example quad has m = 1.0 kg, not 1.22 kg. Legacy AirSim subtracted the motor masses from the box mass (MultiRotorParams.hpp:326: box_mass = mass - N 0.055).",
  "notes": "root_link = links.at(0) (TODO at line 37 allows choosing another root link). Also in: physics/include/base_physics.hpp."
 },
 {
  "id": "PA-8-03",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Aggregate inertia with the parallel-axis theorem (diagonal only)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "155-190",
  "code": "float inertia_dx =\n    link_inertia(0, 0) +\n    link_mass * (link_pos.y() * link_pos.y() + link_pos.z() * link_pos.z());\nfloat inertia_dy =\n    link_inertia(1, 1) +\n    link_mass * (link_pos.x() * link_pos.x() + link_pos.z() * link_pos.z());",
  "formula": "I_xx = I_root,xx + sum_k [ I_k,xx + m_k (y_k^2 + z_k^2) ];  I_yy: (x_k^2 + z_k^2);  I_zz: (x_k^2 + y_k^2); off-diagonal terms are not touched",
  "variables": {
   "I_root": "root link inertia matrix, assumed at the CG [kg m^2]",
   "m_k, r_k": "mass and position of link k relative to the body frame [kg, m]"
  },
  "defaults": [],
  "assumptions": "Diagonal inertia; the root link is assumed to be at the CG; off-diagonal products of inertia are ignored; the sum includes every link i >= 1 (e.g. propellers) so the propeller mass enters inertia but not mass.",
  "notes": "The inertia is computed once at initialisation. Inverse = full 3x3 inverse (base_physics.hpp:57-60)."
 },
 {
  "id": "PA-8-04",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Link inertia from a solid cuboid or from a matrix",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/link/inertial.cpp",
  "lines": "290-297, 303-308",
  "code": "inertia(0, 0) = impl_.mass_ / 12.0f *\n                (box_xyz.y() * box_xyz.y() + box_xyz.z() * box_xyz.z());\ninertia(1, 1) = impl_.mass_ / 12.0f *\n                (box_xyz.x() * box_xyz.x() + box_xyz.z() * box_xyz.z());\ninertia(2, 2) = impl_.mass_ / 12.0f *\n                (box_xyz.x() * box_xyz.x() + box_xyz.y() * box_xyz.y());",
  "formula": "I_xx = m (b_y^2 + b_z^2)/12;  I_yy = m (b_x^2 + b_z^2)/12;  I_zz = m (b_x^2 + b_y^2)/12;  type 'matrix' reads ixx, iyy, izz; 'point-mass' or other = zero matrix",
  "variables": {
   "b": "box size [m]",
   "m": "link mass [kg]"
  },
  "defaults": [
   {
    "name": "inertia (point-mass or unknown type)",
    "value": "0 matrix",
    "unit": "kg m^2",
    "where": "inertial.cpp:277, 309-311"
   },
   {
    "name": "example quad frame",
    "value": "mass 1.0 kg, box 0.180 x 0.110 x 0.040 m",
    "unit": "-",
    "where": "robot_quadrotor_fastphysics.jsonc:7-13"
   }
  ],
  "assumptions": "Solid uniform cuboid. Wikipedia list-of-moments-of-inertia reference.",
  "notes": "Same formula as legacy MultiRotorParams.hpp:296-298."
 },
 {
  "id": "PA-8-05",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Unreal physics: wrench aggregation without frame conversion",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/unreal_physics.cpp",
  "lines": "47-67",
  "code": "aggregate_wrench += wrench_pt->wrench;\naggregate_wrench.torque +=\n    wrench_pt->position.cross(wrench_pt->wrench.force);\naggregate_wrench.force = PhysicsUtils::TransformVectorToWorldFrame(\n    aggregate_wrench.force, kinematics_.pose.orientation);",
  "formula": "F = q sum_i F_i;  tau = sum_i (tau_i + r_i x F_i); the wrench is handed to Unreal's engine through a callback; Unreal integrates the motion",
  "variables": {},
  "defaults": [],
  "assumptions": "Differs from FastPhysics: no rotor-frame-to-body rotation of each wrench (PA-2-05 does it); no drag, no lift-drag, no gravity computed here. Integration (physics_world.cpp:192-195 'No-op, Unreal steps physics body directly') is done by the Unreal Chaos engine, not in this repo.",
  "notes": "Unreal solver parameters stored in the link inertial (inertia-tensor-scale default (1,1,1), stabilization threshold multiplier default 1, velocity solver iterations 1, position solver iterations 8): inertial.cpp:461-484."
 },
 {
  "id": "PA-8-06",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "MATLAB physics and non-physics (kinematic-only) modes",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/matlab_physics.cpp; docs/physics/no_physics.md",
  "lines": "matlab_physics.cpp:38-43, 161-212; no_physics.md:3",
  "code": "// TODO Copy robot's mass, inertia, drag, etc and send to Matlab to intialize\n// No need to do anything here, all wrenches are sent to Matlab for processing",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "MATLAB physics forwards each actuator wrench (rotated to body), gravity and collision info to an external MATLAB/Simulink model; the dynamics live outside this repo. Non-physics mode ('physics-type': 'non-physics') has no dynamics: the robot moves only when the client sets its pose.",
  "notes": "Scanned matlab_physics.cpp fully: only frame rotations and message packing (lines 168-212), no physics arithmetic."
 },
 {
  "id": "PA-8-07",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "JSBSim: linear acceleration reconstructed for the IMU, and unit conversion",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/jsbsim_physics.cpp",
  "lines": "430-436, 453-465",
  "code": "double n = propagate->GetVel(1) * MathUtils::feets_to_meters;\nconst Vector3 specific_force_body(\n    accel->GetBodyAccel(1) * MathUtils::feets_to_meters,\n    accel->GetBodyAccel(2) * MathUtils::feets_to_meters,\n    accel->GetBodyAccel(3) * MathUtils::feets_to_meters);\nreturn PhysicsUtils::TransformVectorToWorldFrame(specific_force_body, orientation) + env_gravity_;",
  "formula": "v[m/s] = v[ft/s] 0.3048;  a_world = R f_body + g,  f_body = JSBSim GetBodyAccel (force/mass in body axes, no gravity) x 0.3048",
  "variables": {
   "f_body": "JSBSim body specific force [m/s^2]",
   "R": "body-to-world rotation [-]",
   "g": "gravity vector [m/s^2]"
  },
  "defaults": [
   {
    "name": "feets_to_meters",
    "value": "0.3048",
    "unit": "m/ft",
    "where": "math_utils.hpp:130-131"
   }
  ],
  "assumptions": "JSBSim (external library, not in this repo) integrates the 6-DOF equations in feet-pound-second units; Project AirSim reads state back and publishes it as Kinematics (SI). GetUVWdot is deliberately NOT used because it carries rotating-frame terms (comment 447-452).",
  "notes": "The JSBSim path is the intended fixed-wing physics engine (examples cessna_310, x8_skywalker)."
 },
 {
  "id": "PA-9-01",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Euler's rotation equation (angular acceleration)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "527-534",
  "code": "const Vector3 angular_momentum = inertia * ave_vel_ang;\nconst Vector3 angular_momentum_rate =\n    total_wrench.torque - ave_vel_ang.cross(angular_momentum);\n\nkin_no_collision.accels.angular = inertia_inv * angular_momentum_rate;",
  "formula": "alpha = I^-1 (tau - omega_avg x (I omega_avg)),  omega_avg = omega + alpha_prev dt/2",
  "variables": {
   "alpha": "body-frame angular acceleration [rad/s^2]",
   "I": "inertia matrix about the CG, body frame [kg m^2]",
   "omega_avg": "average angular velocity over the step [rad/s]",
   "tau": "total torque, body frame [N m]"
  },
  "defaults": [],
  "assumptions": "Includes the gyroscopic term omega x I omega. Torque is the sum of rotor torques (PA-2-05), lift-drag moments and torques (PA-6-08); drag torque is zero (PA-5-03). Linear and angular dynamics are fully decoupled except through the pose rotation R.",
  "notes": "The omega used is the AVERAGE of the current angular velocity and its half-step extrapolation, as in the legacy engine (FastPhysicsEngine.hpp:381-384)."
 },
 {
  "id": "PA-9-02",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Angular velocity update",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "545-548",
  "code": "kin_no_collision.twist.angular =\n    cur_kin.twist.angular +\n    (cur_kin.accels.angular + kin_no_collision.accels.angular) * 0.5f *\n        dt_sec;",
  "formula": "omega[k+1] = omega[k] + (alpha[k] + alpha[k+1]) dt/2  (trapezoidal / velocity-Verlet)",
  "variables": {
   "alpha[k]": "previous stored angular acceleration [rad/s^2]",
   "alpha[k+1]": "new angular acceleration from PA-9-01 [rad/s^2]"
  },
  "defaults": [],
  "assumptions": "The comment calls it Verlet integration; it is the trapezoidal rule on acceleration. Angular velocity in body frame.",
  "notes": "If |omega|^2 > c^2 the vector is scaled down to the speed of light (PA-10-04)."
 },
 {
  "id": "PA-9-03",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Quaternion orientation update by angle-axis",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "574-594",
  "code": "const auto angle_per_unit = ave_vel_ang.norm();\nconst AngleAxis angle_dt_aa =\n    AngleAxis(angle_per_unit * dt_sec, ave_vel_ang / angle_per_unit);\nconst Quaternion angle_dt_q = Quaternion(angle_dt_aa);\nkin_no_collision.pose.orientation = cur_kin.pose.orientation * angle_dt_q;\nkin_no_collision.pose.orientation.normalize();",
  "formula": "q[k+1] = normalize( q[k] (x) AngleAxis(|omega_avg| dt, omega_avg/|omega_avg|) )  (body-frame rotation increment; q stays unchanged if |omega_avg| = 0)",
  "variables": {
   "q": "unit quaternion body-to-world, Hamilton convention [-]",
   "omega_avg": "average body angular velocity [rad/s]"
  },
  "defaults": [],
  "assumptions": "Exact rotation for constant omega_avg over the step; re-normalised every step; throws on NaN.",
  "notes": "The legacy derivation of q0 q1 is in FastPhysicsEngine.hpp:424-438."
 },
 {
  "id": "PA-9-04",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Euler angles from a quaternion",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/transforms/transform_utils.cpp",
  "lines": "59-78",
  "code": "float t0 = +2.0f * (quaternion.w() * quaternion.x() +\n                    quaternion.y() * quaternion.z());\nfloat t1 = +1.0f - 2.0f * (quaternion.x() * quaternion.x() + ysqr);\nauto roll_rad = std::atan2(t0, t1);\nfloat t2 = +2.0f * (quaternion.w() * quaternion.y() -\n                    quaternion.z() * quaternion.x());",
  "formula": "roll = atan2(2(wx + yz), 1 - 2(x^2 + y^2));  pitch = asin(clamp(2(wy - zx), -1, 1));  yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))  (z-y-x Tait-Bryan)",
  "variables": {
   "w,x,y,z": "quaternion components [-]"
  },
  "defaults": [],
  "assumptions": "Used by the grounded-state handling (fast_physics.cpp:416-417) and the simple-flight estimator (PA-15-01). Gimbal lock at pitch = +-90 deg.",
  "notes": ""
 },
 {
  "id": "PA-10-01",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Per-tick order: physics step then sensors then control then actuators then wrenches",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/scene.cpp",
  "lines": "1068-1141",
  "code": "// 2. Step physics:\nstep_physics_func(sim_dt_nanos);\n// 3. Environment and sensors:\nrobot.UpdateEnvironment();\nrobot.UpdateSensors(sim_time_, sim_dt_nanos);\n// 4. Control and actuators:",
  "formula": "order each tick: kinematics(t+dt) = f(wrench held from t, dt) -> environment(t+dt) -> sensors(t+dt) -> controller -> actuators (new wrench for the NEXT dt) -> set wrenches",
  "variables": {
   "dt": "sim_dt_nanos, the clock step [ns]"
  },
  "defaults": [
   {
    "name": "step-ns (clock)",
    "value": "20000000 (20 ms) if omitted; the docs and examples use 3000000 (3 ms)",
    "unit": "ns",
    "where": "docs/config_scene.md:68-69, 79"
   },
   {
    "name": "real-time-update-rate",
    "value": "3000000",
    "unit": "ns",
    "where": "docs/config_scene.md:80"
   }
  ],
  "assumptions": "Zero-order hold of the actuator wrench over each step; one full tick of control latency between sensors at t and the wrench they cause. The steppable clock is deterministic.",
  "notes": "In Unreal, engine-substepping (default false) can cap the physics substep to step-ns (config_scene.md:83)."
 },
 {
  "id": "PA-10-02",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Half-step average velocity used for forces and position",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "495-499, 570",
  "code": "const Vector3 ave_vel_lin =\n    cur_kin.twist.linear + cur_kin.accels.linear * (0.5f * dt_sec);\nconst Vector3 ave_vel_ang =\n    cur_kin.twist.angular + cur_kin.accels.angular * (0.5f * dt_sec);\nkin_no_collision.pose.position = cur_kin.pose.position + ave_vel_lin * dt_sec;",
  "formula": "v_avg = v + a dt/2;  omega_avg = omega + alpha dt/2;  p[k+1] = p[k] + v_avg dt",
  "variables": {
   "v,a": "current stored velocity and acceleration [m/s, m/s^2]",
   "dt": "step [s]"
  },
  "defaults": [],
  "assumptions": "Position advance uses the half-step velocity extrapolated from the PREVIOUS acceleration (second-order exact for constant acceleration); the new acceleration enters only the velocity update. TODO at 567-569 asks whether the new velocity should be used instead.",
  "notes": "Forces (drag, lift-drag) are evaluated at the half-step velocity v_avg."
 },
 {
  "id": "PA-10-03",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Linear velocity update (trapezoidal / 'Verlet')",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "541-543",
  "code": "kin_no_collision.twist.linear =\n    cur_kin.twist.linear +\n    (cur_kin.accels.linear + kin_no_collision.accels.linear) * 0.5f * dt_sec;",
  "formula": "v[k+1] = v[k] + (a[k] + a[k+1]) dt/2",
  "variables": {
   "a[k]": "stored acceleration from the previous step [m/s^2]",
   "a[k+1]": "new acceleration from PA-8-01 [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Trapezoidal rule (the comment links a Verlet note). Second-order accurate; the force at step k+1 is evaluated with the half-step velocity, so the scheme is explicit.",
  "notes": "Same equations as the legacy engine (FastPhysicsEngine.hpp:388)."
 },
 {
  "id": "PA-10-04",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Speed-of-light safety clip",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "551-564",
  "code": "const auto c_light = EarthUtils::kSpeedOfLight;\nif (kin_no_collision.twist.linear.squaredNorm() > c_light * c_light) {\n  kin_no_collision.twist.linear /=\n      (kin_no_collision.twist.linear.norm() / c_light);\n  kin_no_collision.accels.linear = Vector3::Zero();\n}",
  "formula": "if |v| > c: v <- c v/|v|, a <- 0  (same for omega, with c in rad/s)",
  "variables": {
   "c": "299792458 [m/s or rad/s]"
  },
  "defaults": [
   {
    "name": "kSpeedOfLight",
    "value": "299792458",
    "unit": "m/s",
    "where": "earth_utils.hpp:74-75"
   }
  ],
  "assumptions": "Numerical guard against runaway controllers, not physics. The same threshold is applied to angular velocity in rad/s.",
  "notes": ""
 },
 {
  "id": "PA-10-05",
  "group": 10,
  "group_name": "time integration",
  "quantity": "JSBSim sub-stepping",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/jsbsim_physics.cpp",
  "lines": "497-506",
  "code": "constexpr double kTimeEpsilonSec = 1e-12;\ndouble remaining_time_sec = dt_sec;\nwhile (remaining_time_sec > kTimeEpsilonSec) {\n  const double integration_step_sec =\n      std::min(remaining_time_sec, fp_body->jsbsim_dt_sec_);\n  fp_body->model_->Setdt(integration_step_sec);",
  "formula": "repeat: dt_i = min(remaining, dt_jsbsim); JSBSim.Run() with that dt; remaining -= dt_i",
  "variables": {
   "dt_jsbsim": "configured JSBSim maximum integration step [s]"
  },
  "defaults": [
   {
    "name": "kDefaultJSBSimDtSec",
    "value": "0.008333333333333333 (1/120 s)",
    "unit": "s",
    "where": "core_sim/include/core_sim/physics_common_types.hpp:28"
   }
  ],
  "assumptions": "JSBSim's own integrator (not in this repo) does the integration; this wrapper only splits the Project AirSim tick into sub-steps no longer than dt_jsbsim.",
  "notes": ""
 },
 {
  "id": "PA-11-01",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Landed (grounded) latch: stays grounded until net force exceeds weight",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "271-286",
  "code": "const float normalized_force = external_wrench_.force.squaredNorm();\nconst Vector3 weight = mass_ * env_gravity_;\nconst float normalized_weight = weight.squaredNorm();\nif (is_grounded_ && normalized_force < normalized_weight &&\n    kinematics_.twist.linear.z() >= 0) {",
  "formula": "stay grounded while |F_ext|^2 < |m g|^2 and v_z >= 0 (v_z >= 0 is downward or zero in NED); otherwise take off",
  "variables": {
   "F_ext": "external wrench force (rotors + API force), world [N]",
   "m g": "weight [N]",
   "v_z": "vertical velocity, +z down [m/s]"
  },
  "defaults": [],
  "assumptions": "Drag and lift-drag forces are not part of the test (external_wrench_ only). Landed state can also be set at spawn by start-landed.",
  "notes": "When grounded without wheels the kinematics are set to zero velocity/acceleration with roll and pitch zeroed (PA-11-03)."
 },
 {
  "id": "PA-11-02",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Landing-collision test (ground normal and downward motion)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "288-311",
  "code": "const bool is_ground_normal = MathUtils::IsApproximatelyEqual(\n    normal_body.z(), -1.0f, kGroundCollisionAxisTol);\nconst float vel_z = kinematics_.twist.linear.z();\nconst bool is_moving_downward = (vel_z > vel_x_abs) && (vel_z > vel_y_abs);",
  "formula": "landing = (n_body,z approx -1 within rel. tol 0.01) AND (v_z > |v_x| AND v_z > |v_y|)",
  "variables": {
   "n_body": "collision normal in the body frame [-]",
   "v": "world linear velocity, NED [m/s]"
  },
  "defaults": [
   {
    "name": "kGroundCollisionAxisTol",
    "value": "0.01",
    "unit": "-",
    "where": "fast_physics.hpp:89"
   }
  ],
  "assumptions": "Assumes NED. A normal along body +z would not count (only -1). Collision geometry comes from Unreal (CollisionInfo), not from the physics engine.",
  "notes": "Legacy used |n_z| approx 1 within 0.25 (FastPhysicsEngine.hpp:151, kAxisTolerance 0.25)."
 },
 {
  "id": "PA-11-03",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Landed pose: offset along the contact normal and zero roll/pitch",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "381-385, 403-419",
  "code": "const Vector3 landed_position =\n    fp_body->collision_info_.position +\n    fp_body->collision_info_.normal *\n        (fp_body->collision_info_.penetration_depth +\n         FastPhysicsBody::kCollisionOffset);\nkin_grounded.pose.orientation = TransformUtils::ToQuaternion(0., 0., rpy[2]);",
  "formula": "p_landed = p_contact + n (d_pen + 0.001);  grounded kinematics: v = 0, omega = 0, a = 0, roll = pitch = 0, yaw kept",
  "variables": {
   "n": "collision normal [-]",
   "d_pen": "penetration depth [m]",
   "kCollisionOffset": "0.001 [m] (land with a 1 mm air gap)"
  },
  "defaults": [
   {
    "name": "kCollisionOffset",
    "value": "0.001",
    "unit": "m",
    "where": "fast_physics.hpp:90"
   }
  ],
  "assumptions": "A ground lock: the vehicle is held still and level on the contact point.",
  "notes": "Legacy had the same ground-lock (FastPhysicsEngine.hpp:223-242) with extra: restitution 0 and friction 1 for ground hits, the angular velocity damped by 0.9 (line 215), and a z-acceleration bump a = -0.5 m v (line 239)."
 },
 {
  "id": "PA-11-04",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Collision impulse: restitution (normal) impulse with Coulomb friction",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "626-660",
  "code": "const Vector3 contact_vel_body =\n    ave_vel_lin_body + ave_vel_ang.cross(r_contact);\nconst float restitution_impulse_denom =\n    mass_inv + (inertia_inv * r_contact.cross(normal_body))\n                   .cross(r_contact)\n                   .dot(normal_body);\nconst float restitution_impulse = -contact_vel_body.dot(normal_body) *\n                                  (1.0f + restitution) /\n                                  restitution_impulse_denom;",
  "formula": "v_c = v_b + omega x r;  j = -(1 + e)(v_c . n) / (1/m + ((I^-1 (r x n)) x r) . n);  v+ = v_avg + n j/m;  omega+ = omega_avg + (r x n) j  (the code adds no I^-1 here; see TODO line 658)",
  "variables": {
   "e": "restitution coefficient of the root link collision [-]",
   "r": "contact point vector impact_point - position [m]",
   "n": "collision normal [-]",
   "j": "impulse magnitude [N s]",
   "v_avg": "v + a dt (full-step, not half) (lines 617-621)"
  },
  "defaults": [
   {
    "name": "restitution (example)",
    "value": "0.1",
    "unit": "-",
    "where": "robot_quadrotor_fastphysics.jsonc:27"
   },
   {
    "name": "friction (example)",
    "value": "0.5",
    "unit": "-",
    "where": "robot_quadrotor_fastphysics.jsonc:28"
   }
  ],
  "assumptions": "Collision with a static fixed body; formula from Hecker (Physics Part 3) and Gaffer on Games; code comments say it does not produce realistic bounces (TODO 643) and the angular update should include inertia (TODO 658). Only active when moving into the surface (n . v_next < 0) and not a landing.",
  "notes": "Collision info (normal, impact point, penetration) is supplied by Unreal; this module only turns it into an impulse."
 },
 {
  "id": "PA-11-05",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Collision friction impulse and post-collision position",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "664-698",
  "code": "const float friction_mag =\n    -contact_tang_body.norm() * friction / friction_mag_denom;\nkin_with_collision.twist.linear += contact_tang_unit * friction_mag;\nkin_with_collision.pose.position =\n    collision_info.position +\n    (collision_info.normal * collision_info.penetration_depth) +\n    (kin_with_collision.twist.linear * dt_sec);",
  "formula": "v_t = v_c - n (n . v_c);  t = v_t/|v_t|;  j_t = -|v_t| mu / (1/m + ((I^-1 (r x t)) x r) . t);  v += t j_t;  omega += (r x t) j_t / m;  a = 0;  p = p_contact + n d_pen + v dt",
  "variables": {
   "mu": "friction coefficient (0 = no tangential reduction, 1 = full) [-]",
   "t": "unit tangential direction [-]"
  },
  "defaults": [],
  "assumptions": "Accelerations are zeroed during the response so they do not cancel the impulse (comment lines 687-688). The friction impulse is capped by no Coulomb cone: it is proportional to the tangential speed, not to the normal impulse.",
  "notes": "Same structure as the legacy code (FastPhysicsEngine.hpp:202-246), where the angular friction term has the mass inverse in the same place."
 },
 {
  "id": "PA-11-06",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "JSBSim collision clamps",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/jsbsim_physics.cpp",
  "lines": "520-523, 562-585",
  "code": "constexpr float kMinImpulseDenom = 1e-5f;\nconstexpr float kMaxPenetrationDepth = 1.0f;\nconstexpr float kMaxDeltaVCollision = 20.0f;\nconst float max_impulse = mass * kMaxDeltaVCollision;",
  "formula": "same impulse as PA-11-04 but j clamped to [-m 20, m 20] and the penetration depth clamped to [0, 1] m; degenerate denominator (|d| < 1e-5) leaves velocities at zero",
  "variables": {
   "kMaxDeltaVCollision": "max velocity change in one collision [m/s]"
  },
  "defaults": [
   {
    "name": "kMaxDeltaVCollision",
    "value": "20",
    "unit": "m/s",
    "where": "jsbsim_physics.cpp:523"
   },
   {
    "name": "kMaxPenetrationDepth",
    "value": "1",
    "unit": "m",
    "where": "jsbsim_physics.cpp:522"
   },
   {
    "name": "kMinImpulseDenom",
    "value": "1e-5",
    "unit": "-",
    "where": "jsbsim_physics.cpp:520"
   }
  ],
  "assumptions": "A copy-paste of the FastPhysics collision with guards (TO-DO at line 511 says to extract a common reaction). JSBSim also resolves ground contact itself through its ground callback.",
  "notes": ""
 },
 {
  "id": "PA-11-07",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Wheeled rover kinematic (bicycle) update",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "physics/src/fast_physics.cpp",
  "lines": "434-468",
  "code": "rover_speed = wheel->GetRadius() * rover_rotating_speed;\nauto dradians_yaw =\n    atan2(rover_speed * sin(rover_steering) * dt_sec, fp_body->rover_length_);\nauto dm_per_sec_x = rover_speed * cos(radians_yaw);\nauto dm_per_sec_y = rover_speed * sin(radians_yaw);\nkin.pose.position.x() += dm_per_sec_x * dt_sec;",
  "formula": "v = r omega_wheel;  d(yaw) = atan2(v sin(delta) dt, L);  x += v cos(yaw) dt;  y += v sin(yaw) dt;  z = z_landed;  yaw_rate = d(yaw)/dt",
  "variables": {
   "r": "wheel radius [m]",
   "omega_wheel": "engine-connected wheel speed [rad/s]",
   "delta": "steering angle [rad]",
   "L": "wheelbase = x-distance between the first two wheels [m]"
  },
  "defaults": [],
  "assumptions": "Rover: kinematic bicycle model on a flat ground; no tyre slip, no dynamics. The pitch and roll are zero. Wheelbase from the wheel origin translations (lines 91-100); if the first two wheels have equal x the third wheel is used.",
  "notes": "Applies when the robot HasWheels and is grounded."
 },
 {
  "id": "PA-11-08",
  "group": 11,
  "group_name": "ground contact / collision",
  "quantity": "Wheel actuator: speed, steering and brake model",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/wheel.cpp",
  "lines": "373-406",
  "code": "steering_value_ = steering_signal_filtered * (float)M_PI / 4;\nauto engine_torque = engine_signal_filtered * 100.0f * 0.5f;\nauto brake_torque = brake_signal_filtered * -GetRotatingSpeed();\nauto friction_torque =\n    wheel_settings_.coeff_of_friction * -GetRotatingSpeed();\nauto angular_accel = resulting_torque / 0.125;  // Hardcoded inertia moment.",
  "formula": "delta = pi/4 LPF(steer);  Q_eng = 50 LPF(engine);  Q_brake = -LPF(brake) omega;  Q_fric = -mu_w omega;  alpha = (Q_eng + Q_brake + Q_fric)/0.125;  omega += alpha dt;  F = Q/r",
  "variables": {
   "mu_w": "coeff_of_friction [N m s]",
   "omega": "wheel speed [rad/s]",
   "0.125": "hard-coded wheel moment of inertia [kg m^2]",
   "r": "wheel radius [m]"
  },
  "defaults": [
   {
    "name": "radius_",
    "value": "0.457",
    "unit": "m",
    "where": "core_sim/include/core_sim/actuators/wheel.hpp:46"
   },
   {
    "name": "coeff_of_friction",
    "value": "0.0",
    "unit": "N m s",
    "where": "wheel.hpp:43"
   },
   {
    "name": "engine torque scale",
    "value": "100 x 0.5 = 50 (hard-coded)",
    "unit": "N m",
    "where": "wheel.cpp:390"
   }
  ],
  "assumptions": "Steering is forced to 0 when coeff_of_friction == 0 (lines 361-363). Steering clamp is +-45 deg. The wheel hard-codes inertia and engine torque; force_scalar_ = resulting_torque/radius is stored but the rover motion in PA-11-07 uses only omega and the steering angle.",
  "notes": "CalcMaxTorqueAndForce (wheel.hpp:66-70): max_force = coeff_of_friction, max_torque = max_force radius, max_acceleration = max_force/1 kg."
 },
 {
  "id": "PA-12-01",
  "group": 12,
  "group_name": "control allocation / mixing (control_mapper)",
  "quantity": "ControlMapper: affine map of a control input into an output range",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/control_mapper.cpp",
  "lines": "94-105",
  "code": "if (fclamp_input_)\n  control_signal = std::clamp(control_signal, domain_min_, domain_max_);\noutput = (control_signal - domain_min_) / ddomain_ * drange_ + range_min_;\nif (fclamp_output_) output = std::clamp(output, range_min_, range_max_);",
  "formula": "y = (clamp(u, a, b) - a)/(b - a) (d - c) + c  with input range [a,b] = [input_min, input_max] and output range [c,d] = [output_min, output_max]",
  "variables": {
   "u": "control signal [-]",
   "a,b": "input domain limits, default -1..1",
   "c,d": "output range limits, default -1..1"
  },
  "defaults": [
   {
    "name": "domain",
    "value": "[-1, 1]",
    "unit": "-",
    "where": "control_mapper.cpp:22-23"
   },
   {
    "name": "range",
    "value": "[-1, 1]",
    "unit": "-",
    "where": "control_mapper.cpp:27-28"
   },
   {
    "name": "clamp input / output",
    "value": "true / true",
    "unit": "-",
    "where": "control_mapper.cpp:25-26"
   },
   {
    "name": "scale",
    "value": "1.0 (stored; not used by operator())",
    "unit": "-",
    "where": "control_mapper.cpp:29"
   }
  ],
  "assumptions": "Used only by the tilt actuator in the files read (tilt.cpp:216). The scale member is loaded and stored but operator() does not apply it.",
  "notes": "A division by ddomain_ = b - a: a zero-width input range would divide by zero (no guard)."
 },
 {
  "id": "PA-12-02",
  "group": 12,
  "group_name": "control allocation / mixing (control_mapper)",
  "quantity": "Simple-flight mixer: quad-X and hex-X mixing of roll, pitch, yaw, throttle into motor commands",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "vehicle_apis/multirotor_api/include/simple_flight/firmware/Mixer.hpp",
  "lines": "139-146, 190-197, 207-216",
  "code": "for (int iinput = 0; iinput < cinput_active; ++iinput)\n  output += controls[iinput] * rgr_coefficient[iinput];\n{-1.0f,  1.0f,  1.0f, 1.0f},  // FRONT_R\n{ 1.0f, -1.0f,  1.0f, 1.0f},  // REAR_L\n{ 1.0f,  1.0f, -1.0f, 1.0f},  // FRONT_L\n{-1.0f, -1.0f, -1.0f, 1.0f},  // REAR_R",
  "formula": "m_i = M[i][0] u_roll + M[i][1] u_pitch + M[i][2] u_yaw + M[i][3] u_thr;  quad-X rows (roll,pitch,yaw,thr): FR(-1,+1,+1,1) RL(+1,-1,+1,1) FL(+1,+1,-1,1) RR(-1,-1,-1,1);  hex-X rows: 0(0.5,1,-1,1) 1(-0.5,1,1,1) 2(-1,0,-1,1) 3(-0.5,-1,1,1) 4(0.5,-1,-1,1) 5(1,0,1,1)",
  "variables": {
   "m_i": "motor command i, typically in [0,1] before saturation [-]",
   "u": "controller outputs: roll, pitch, yaw torque commands and throttle [-]"
  },
  "defaults": [
   {
    "name": "motor_count",
    "value": "4",
    "unit": "-",
    "where": "firmware/Params.hpp:328"
   }
  ],
  "assumptions": "Mixer matrix elements are +-1 (unit-weight, no scaling by arm length or thrust constants); motor_count must be 4 or 6 or it throws. The actuator order in the robot config must match the rows (example: Prop_FR, Prop_RL, Prop_FL, Prop_RR, robot_quadrotor_fastphysics.jsonc:204-218).",
  "notes": "Rows are 'only change the sign' (comment line 189): a pure sign mixer, so the PID outputs carry the real gains."
 },
 {
  "id": "PA-12-03",
  "group": 12,
  "group_name": "control allocation / mixing (control_mapper)",
  "quantity": "Mixer saturation handling (shift up, scale down)",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "vehicle_apis/multirotor_api/include/simple_flight/firmware/Mixer.hpp",
  "lines": "153-178",
  "code": "if (output_min < params_->motor.min_motor_output) {\n  float undershoot = params_->motor.min_motor_output - output_min;\n  for (int irotor = 0; irotor < crotor; ++irotor)\n    motor_outputs[irotor] += undershoot;\n}\nfloat scale = output_max / params_->motor.max_motor_output;",
  "formula": "if min_i m_i < m_min: m_i += (m_min - min_j m_j) for all rotors;  scale = max_j m_j / m_max;  if scale > m_max: m_i /= scale for all rotors",
  "variables": {
   "m_min, m_max": "min and max motor output, defaults 0 and 1 [-]"
  },
  "defaults": [
   {
    "name": "min_motor_output",
    "value": "0",
    "unit": "-",
    "where": "Params.hpp:329"
   },
   {
    "name": "max_motor_output",
    "value": "1",
    "unit": "-",
    "where": "Params.hpp:330"
   },
   {
    "name": "min_control_output / max_control_output (non-rotor outputs)",
    "value": "-1 / 1",
    "unit": "-",
    "where": "Params.hpp:331-332"
   }
  ],
  "assumptions": "Preserves the differences between motors (attitude authority) by shifting the whole set up, then scales down uniformly. The scale test compares the ratio scale (output_max/m_max) with m_max (it is scale > max_motor_output, not scale > 1): with the default max of 1 the two are the same, but for another m_max it differs. The final clip loop (lines 180-185) runs from cmotor to cmotor, which is empty: non-rotor outputs are never clipped.",
  "notes": "This is the closest thing to a thrust-saturation / control-allocation rule in the repo."
 },
 {
  "id": "PA-12-04",
  "group": 12,
  "group_name": "control allocation / mixing (control_mapper)",
  "quantity": "VTOL mixers: tailsitter and tiltrotor (multicopter/fixed-wing blend by tilt)",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "vehicle_apis/multirotor_api/include/simple_flight/firmware/Mixer.hpp",
  "lines": "99-115, 247-262, 279-342",
  "code": "float prop1 = 1.0f - prop2;\n*pr = *prMixer1++ * prop1 + *prMixer2++ * prop2;\nauto tilt = std::clamp(controls[4], 0.0f, 1.0f);\nmixermatrixVTOLFWTiltrotor.Set(mixermatrixVTOLFWTiltrotorMC,\n                               mixermatrixVTOLFWTiltrotorFW,\n                               tilt);",
  "formula": "M(tilt) = (1 - tilt) M_MC + tilt M_FW;  tiltrotor outputs: 4 rotors (roll 0.5, pitch 0.5, yaw 0.5, thr 1.0 in MC mode; yaw -0.5/0.5 + thr 1 in FW mode), 4 tilt servos (tilt input), ailerons -+1 roll, elevator pitch 1, rudder -yaw; tailsitter: quad-X rows plus elevons (pitch -0.5, yaw +-0.5)",
  "variables": {
   "tilt": "transition fraction 0 = multicopter, 1 = fixed-wing [-]"
  },
  "defaults": [
   {
    "name": "tilt mixer update threshold",
    "value": "0.01",
    "unit": "-",
    "where": "Mixer.hpp:251"
   }
  ],
  "assumptions": "Mixers are linear matrices; the blend is recomputed only when the tilt changes by more than 0.01 or reaches 0 or 1.",
  "notes": "The matrices roughly match PX4's Generic Quadplane VTOL Tiltrotor (comment 304)."
 },
 {
  "id": "PA-13-01",
  "group": 13,
  "group_name": "battery",
  "quantity": "Battery energy drain from rotor power",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/battery.cpp",
  "lines": "226-233, 254-260",
  "code": "state_.battery_remaining =\n    std::max(0.0f, state_.battery_remaining -\n                       (dt * battery_settings_.battery_drain_rate));\nstate_.estimated_time_remaining =\n    state_.battery_remaining / battery_settings_.battery_drain_rate;\nbattery_settings_.battery_drain_rate = power * battery_settings_.power_coefficient;",
  "formula": "E[k+1] = max(0, E[k] - dt c P_rotors);  pct = 100 E/E_total;  t_remaining = E/(c P_rotors)  (rotor-power-discharge mode);  in simple-discharge mode the drain rate is the fixed value from the config instead of c P",
  "variables": {
   "E": "remaining energy [J]",
   "P_rotors": "sum of rotor |Q omega| from PA-2-06 [W]",
   "c": "rotor power coefficient [-]",
   "E_total": "total capacity [J]"
  },
  "defaults": [
   {
    "name": "rotor-power-coefficient",
    "value": "1.0",
    "unit": "-",
    "where": "battery.cpp:392 (loader)"
   },
   {
    "name": "total-battery-capacity (loader default)",
    "value": "100",
    "unit": "J",
    "where": "battery.cpp:381"
   },
   {
    "name": "battery-capacity-on-start (loader default)",
    "value": "100",
    "unit": "J",
    "where": "battery.cpp:383"
   },
   {
    "name": "battery-drain-rate-on-start (simple mode)",
    "value": "0.02",
    "unit": "capacity-units/s",
    "where": "battery.cpp:388"
   },
   {
    "name": "BatterySettings struct defaults",
    "value": "mode energy-consumption, capacity 150000, drain 0, coefficient 1",
    "unit": "-",
    "where": "core_sim/include/core_sim/sensors/battery.hpp:22-27"
   }
  ],
  "assumptions": "No voltage sag, internal resistance, discharge curve, temperature or C-rate model (docs/sensors/battery.md:3: 'not meant to help with battery modeling'). Docs state capacity in joules and an extra constant onboard power term ('Power Consumed = ... + constant', doc line 70); the code has no such constant. Divide-by-zero when the drain rate is 0 gives inf in estimated_time_remaining.",
  "notes": "The loader sets battery_pct_remaining = remaining/capacity WITHOUT the factor 100 (line 384-385), but the first Update recomputes it with 100. The charge-state test (lines 234-243) checks '< 20' before '< 5', so the CRITICAL branch is unreachable."
 },
 {
  "id": "PA-14-01",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU specific force and angular rate (ground truth before noise)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/imu.cpp",
  "lines": "185-192",
  "code": "output_.angular_velocity = ground_truth_.kinematics->twist.angular;\n\noutput_.linear_acceleration = PhysicsUtils::TransformVectorToBodyFrame(\n    ground_truth_.kinematics->accels.linear -\n        ground_truth_.environment->env_info.gravity,\n    ground_truth_.kinematics->pose.orientation);",
  "formula": "omega_meas = omega (body);  f_b = R^T (a_world - g_vec)  (specific force; at rest on the ground f_b = (0, 0, -g) in NED body)",
  "variables": {
   "f_b": "accelerometer reading [m/s^2]",
   "a_world": "stored world linear acceleration [m/s^2]",
   "g_vec": "gravity (0,0,+g) [m/s^2]",
   "R": "body-to-world rotation [-]"
  },
  "defaults": [],
  "assumptions": "Ideal sensor at the CG: no lever-arm, no mounting rotation, no scale factor or misalignment, no saturation, no quantisation. The accelerometer reads zero in free fall. The IMU orientation output is the true orientation.",
  "notes": "Gravity is subtracted so the result is the proper acceleration; note the sign: g_vec is +z (down), so f_b,z = -g at rest."
 },
 {
  "id": "PA-14-02",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU noise: velocity/angle random walk and bias random walk",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/imu.cpp",
  "lines": "209-235",
  "code": "auto sqrt_dt = static_cast<float>(\n    sqrt(std::max<TimeSec>(dt, imu_settings.min_sample_time)));\nRealT accelerometer_sigma_vel_rand_walk =\n    imu_settings.accelerometer.velocity_random_walk / sqrt_dt;\nimu_data.linear_acceleration +=\n    gaussian_distrib.next() * accelerometer_sigma_vel_rand_walk +\n    state_.accelerometer_bias;\nRealT accelerometer_sigma_bias = accelerometer_bias_stability_norm * sqrt_dt;",
  "formula": "s = sqrt(max(dt, 1e-3));  f_meas = f + N(0,1) VRW/s + b_a;  b_a += N(0,1) (BS_a/sqrt(tau_a)) s;  gyro: omega_meas = omega + N(0,1) ARW/s + b_g;  b_g += N(0,1) (BS_g/sqrt(tau_g)) s",
  "variables": {
   "VRW": "accelerometer velocity random walk [m/s^2 / sqrt(Hz) = m/s/sqrt(s)]",
   "ARW": "gyro angle random walk [rad/s / sqrt(Hz) = rad/sqrt(s)]",
   "b_a, b_g": "accelerometer / gyro bias state (3-vectors) [m/s^2, rad/s]",
   "BS": "bias stability [m/s^2, rad/s]",
   "tau": "bias correlation time [s]",
   "N(0,1)": "3 independent standard normal samples per call (fixed seeds 1, 2, 3, noise_model_utils.hpp:41-46; imu.cpp:88)"
  },
  "defaults": [
   {
    "name": "accelerometer.velocity_random_walk",
    "value": "0.24 g / 1000 = 2.3536e-3 (0.24 mg/sqrt(Hz) written as 0.24f*gravity/1.0E3f)",
    "unit": "m/s^2/sqrt(Hz)",
    "where": "core_sim/include/core_sim/sensors/imu.hpp:37"
   },
   {
    "name": "accelerometer.tau",
    "value": "800",
    "unit": "s",
    "where": "imu.hpp:38"
   },
   {
    "name": "accelerometer.bias_stability",
    "value": "36e-6 g = 3.53e-4",
    "unit": "m/s^2",
    "where": "imu.hpp:39"
   },
   {
    "name": "gyro.angle_random_walk",
    "value": "0.3/sqrt(3600) deg -> rad = 8.727e-5",
    "unit": "rad/s/sqrt(Hz)",
    "where": "imu.hpp:44-45"
   },
   {
    "name": "gyro.tau",
    "value": "500",
    "unit": "s",
    "where": "imu.hpp:46"
   },
   {
    "name": "gyro.bias_stability",
    "value": "4.6 deg/h -> 2.23e-5",
    "unit": "rad/s",
    "where": "imu.hpp:47"
   },
   {
    "name": "min_sample_time",
    "value": "0.001",
    "unit": "s",
    "where": "imu.hpp:51"
   },
   {
    "name": "turn_on_bias",
    "value": "(0,0,0)",
    "unit": "m/s^2, rad/s",
    "where": "imu.hpp:40, 48"
   }
  ],
  "assumptions": "The bias is added BEFORE it is advanced (so the first sample uses the previous bias). The bias walk uses sigma = BS/sqrt(tau) sqrt(dt): a pure random walk (not a Gauss-Markov decay); tau only sets the diffusion rate. The sample dt is the scene tick, floored at 1 ms. Fixed RNG seeds give the same noise sequence every run. Comment 'See docs/imu for the math' - that page was not found in the tree.",
  "notes": "The UAM example config repeats the struct defaults: velocity-random-walk 2.353e-3, bias-stability 3.53e-4 (accelerometer); angle-random-walk 8.72644e-5, bias-stability 2.23014e-5 (gyroscope) - robot_quadtiltrotor_fastphysics_uam.jsonc, IMU1 block. Arithmetic check of the struct expressions: 0.24 x 9.80665/1000 = 2.3536e-3; 36e-6 x 9.80665 = 3.53e-4; 0.3/60 deg x pi/180 = 8.727e-5; 4.6/3600 deg x pi/180 = 2.230e-5."
 },
 {
  "id": "PA-14-03",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU bias-walk normalisation constants at load",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/imu.cpp",
  "lines": "309-313, 331-334",
  "code": "impl.state_.accelerometer_bias = impl.imu_settings.accelerometer.turn_on_bias;\nimpl.accelerometer_bias_stability_norm =\n    impl.imu_settings.accelerometer.bias_stability /\n    sqrt(impl.imu_settings.accelerometer.tau);",
  "formula": "b(0) = turn_on_bias;  BS_norm = bias_stability/sqrt(tau)",
  "variables": {
   "BS_norm": "bias random-walk intensity [unit/sqrt(s)]"
  },
  "defaults": [],
  "assumptions": "Computed only inside LoadAccelerometerSettings / LoadGyroSettings, which run only when the 'accelerometer' / 'gyroscope' block exists and is non-empty (imu.cpp:274-289). The members accelerometer_bias_stability_norm and gyroscope_bias_stability_norm (imu.cpp:95) and state_ biases (imu.cpp:90-93) have no default initialiser, so an IMU config WITHOUT those blocks leaves them uninitialised (undefined values) while ApplyNoiseModel still uses them.",
  "notes": "Source observation, not tested at run time."
 },
 {
  "id": "PA-14-04",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gauss-Markov noise process (used by barometer and airspeed drift)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/sensors/noise_model_utils.hpp",
  "lines": "102-117",
  "code": "double alpha = exp(-dt / tau_);\noutput_ = static_cast<float>(alpha * output_ +\n                             (1 - alpha) * getNextRandom() * sigma_);",
  "formula": "x[k+1] = e^(-dt/tau) x[k] + (1 - e^(-dt/tau)) sigma w[k],  w ~ N(0,1)",
  "variables": {
   "x": "noise state [unit of the signal]",
   "tau": "correlation time [s]",
   "sigma": "driving standard deviation [same unit]",
   "w": "standard normal sample (float RNG, default seed 42)"
  },
  "defaults": [],
  "assumptions": "Steady-state variance is sigma^2 (1-alpha)/(1+alpha), not sigma^2. Comment cites Quinchia (GPS/INS MEMS error modelling, sec 3.2) and Wall 2007 eq 2.5.",
  "notes": "The initial output is 0 for the barometer."
 },
 {
  "id": "PA-14-05",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gaussian random generators (seeds and distributions)",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/sensors/noise_model_utils.hpp",
  "lines": "41-48, 50-72",
  "code": "typedef RandomGenerator<RealT, std::normal_distribution<double>, 1>\n    RandomGeneratorGaussianXT;\ntypedef RandomGenerator<RealT, std::normal_distribution<double>, 2>\n    RandomGeneratorGaussianYT;\ntypedef RandomGenerator<RealT, std::normal_distribution<double>, 3>\n    RandomGeneratorGaussianZT;",
  "formula": "x_i ~ N(mean, sigma_i) with std::mt19937 seeded 1, 2, 3 (x, y, z); scalar float generator seeded 42",
  "variables": {},
  "defaults": [
   {
    "name": "seeds",
    "value": "1, 2, 3 (vector), 42 (scalar)",
    "unit": "-",
    "where": "noise_model_utils.hpp:41-48"
   }
  ],
  "assumptions": "Deterministic noise: each sensor instance restarts the same sequence, so two IMUs (or two runs) see identical noise.",
  "notes": ""
 },
 {
  "id": "PA-14-06",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Barometer: pressure with drift and white noise, then altitude",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/barometer.cpp",
  "lines": "181-185, 215-233",
  "code": "pressure_factor_.Update(dt);\nbarometer_data.pressure +=\n    barometer_data.pressure * pressure_factor_.getOutput();\nbarometer_data.pressure += uncorrelated_noise_.next();\nbarometer_data.pressure = barometer_data.pressure -\n                          EarthUtils::kSeaLevelPressure +\n                          barometer_settings_.qnh * 100.0f;\nbarometer_data.altitude =\n    EarthUtils::GetPressureAltitude(barometer_data.pressure);",
  "formula": "p_true from the atmosphere model (PA-1-05);  g_p = GaussMarkov(tau_p, sigma_p);  p = p_true (1 + g_p) + N(0, sigma_u);  p = p - 101325 + 100 QNH;  h = (1 - (p/101325)^0.190284) 145366.45 (0.3048)",
  "variables": {
   "QNH": "sea-level pressure setting [hPa]",
   "g_p": "relative pressure drift factor [-]",
   "sigma_u": "uncorrelated noise sigma [Pa]"
  },
  "defaults": [
   {
    "name": "qnh",
    "value": "1013.25",
    "unit": "hPa",
    "where": "barometer.hpp:31"
   },
   {
    "name": "pressure_factor_sigma",
    "value": "0.0365/20 = 0.001825",
    "unit": "-",
    "where": "barometer.hpp:40"
   },
   {
    "name": "pressure_factor_tau",
    "value": "3600",
    "unit": "s",
    "where": "barometer.hpp:41"
   },
   {
    "name": "uncorrelated_noise_sigma",
    "value": "0.027 x 100 = 2.7",
    "unit": "Pa",
    "where": "barometer.hpp:46"
   },
   {
    "name": "update_latency / update_frequency / startup_delay",
    "value": "0 s / 50 Hz / 0 s (loaded but not referenced in Update)",
    "unit": "-",
    "where": "barometer.hpp:52-54; barometer.cpp:272-282"
   }
  ],
  "assumptions": "Pressure is the true static pressure from the standard atmosphere at the vehicle altitude (no wind or rotor-wash effect). The altitude output is pressure altitude from the QNH-shifted pressure, so with QNH = 1013.25 it equals standard-atmosphere altitude. The drift sigma 3.65%/20 and tau 1 h give roughly a 10 m/h altitude wander (comment lines 33-39). The sensor runs at the scene tick, not 50 Hz.",
  "notes": "Comment (line 222) 'about 0.2m sigma' for the 2.7 Pa noise (about 0.22 m near sea level)."
 },
 {
  "id": "PA-14-07",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Magnetic field: tilted-dipole Earth model",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/include/core_sim/earth_utils.hpp",
  "lines": "278-321",
  "code": "double mag_clat = acos(cos_lat * cos_MagPoleLat +\n                       sin_lat * sin_MagPoleLat * cos(lon - MagPoleLon));\ndouble field_mag = MeanMagField * pow(kEarthRadius / altitude, 3) *\n                   sqrt(1 + 3 * cos_mag_clat * cos_mag_clat);\ninclination = atan(2.0 / tan(mag_clat));  // do not use atan2 here",
  "formula": "cos(theta_m) = cos(lat) cos(lat_pole) + sin(lat) sin(lat_pole) cos(lon - lon_pole);  B = B0 (R/(R + h))^3 sqrt(1 + 3 cos^2 theta_m);  I = atan(2/tan(theta_m));  B_xy = B cos I;  B_ned = (B_xy cos D, B_xy sin D, B sin I)",
  "variables": {
   "theta_m": "magnetic colatitude [rad]",
   "B0": "mean equatorial field 3.12e-5 [T]",
   "I, D": "inclination and declination [rad]",
   "B_ned": "field in the geographic NED frame [T]"
  },
  "defaults": [
   {
    "name": "MagPoleLat / MagPoleLon",
    "value": "80.31 deg / -72.62 deg",
    "unit": "deg",
    "where": "earth_utils.hpp:88-89"
   },
   {
    "name": "MeanMagField",
    "value": "3.12e-5",
    "unit": "T",
    "where": "earth_utils.hpp:91"
   }
  ],
  "assumptions": "First-order centered-dipole model; the source comments say actual values may differ by up to 10x (example: dipole 2.50e-5, 3.4e-6, 3.66e-5 T vs WMM2015 1.84e-5, 5.2e-6, 5.0e-5 T). The model is only meant to give correct signs and changes with position.",
  "notes": "Declination formula from Tulane eens634 Hmwk6 eq (4a)."
 },
 {
  "id": "PA-14-08",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Magnetometer output: rotate to sensor frame, scale, add noise and bias",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/magnetometer.cpp",
  "lines": "184-207, 223-227",
  "code": "magnetic_field_true_ =\n    EarthUtils::GetMagneticField(\n        ground_truth_.environment->env_info.geo_point) *\n    1e4f;  // Convert units from Tesla to Gauss\nQuaternion sensor_quat = ground_truth_.kinematics->pose.orientation *\n                         magnetometer_settings_.origin_setting.rotation_;\nmagnetometer_data.magnetic_field_body += noise_vec_.next() + bias_vec_;",
  "formula": "B_world = 1e4 B_ned [Gauss];  B_sensor = k R_s^T B_world + n + b;  R_s = R_body R_mount;  n ~ N(0, sigma_i) per axis per call;  b drawn once at start from N(-bias, bias) per axis",
  "variables": {
   "k": "scale_factor [-]",
   "n": "white noise [Gauss]",
   "b": "constant bias vector [Gauss]",
   "R_s": "sensor-to-world rotation [-]"
  },
  "defaults": [
   {
    "name": "noise_sigma",
    "value": "(0.005, 0.005, 0.005)",
    "unit": "Gauss (5 mGauss, LSM303D spec)",
    "where": "magnetometer.hpp:34"
   },
   {
    "name": "scale_factor",
    "value": "1.0",
    "unit": "-",
    "where": "magnetometer.hpp:36"
   },
   {
    "name": "noise_bias",
    "value": "(0, 0, 0)",
    "unit": "Gauss",
    "where": "magnetometer.hpp:38"
   },
   {
    "name": "ref_source",
    "value": "dipole model (constant Seattle field (0.34252, 0.9805, 0.93438) Gauss is the alternative)",
    "unit": "-",
    "where": "magnetometer.hpp:42; magnetometer.cpp:188"
   }
  ],
  "assumptions": "No hard-iron or soft-iron effects, no motor-current interference, no temperature. The magnetic_field_covariance output is declared but never filled in the code read. The bias draw RandomVectorGaussianR(-noise_bias, noise_bias) uses mean = -bias and sigma = bias, so with the default zero bias it is exactly zero (and with a nonzero bias the sigma argument would be a negative or positive vector).",
  "notes": "The constant 'Seattle' field (0.34252, 0.9805, 0.93438) has |B| = sqrt(1.95) = 1.40 (in Gauss, per the code's unit conversion), about 2 to 3 times the dipole-model magnitude (the model's equatorial mean is 0.312 Gauss and about 0.6 Gauss at the poles). The Earth's real field in Seattle is about 0.55 Gauss; this constant is flagged as unverified/suspect."
 },
 {
  "id": "PA-14-09",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "GPS: position, velocity (noise-free) and dilution of precision filters",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/gps.cpp",
  "lines": "224-225, 241-252",
  "code": "output_.geo_point = ground_truth_.environment->env_info.geo_point;\noutput_.velocity = ground_truth_.kinematics->twist.linear;\nepv_filter.UpdateOutput(dt);\neph_filter.UpdateOutput(dt);\ngps_data.fix_type =\n    gps_data.eph <= gps_settings_.eph_min_3d   ? GnssFixType::GNSS_FIX_3D_FIX",
  "formula": "lat, lon, alt = ground truth;  v = world velocity (no noise);  eph[k+1] = LPF_tau(eph_final): eph from 100 toward 0.1 with tau = 0.9 s;  fix = 3D if eph <= 3.0, 2D if eph <= 4.0, else none",
  "variables": {
   "eph, epv": "horizontal / vertical position error estimate (HDOP/VDOP-like) [m]",
   "tau": "eph/epv time constant 0.9 [s]"
  },
  "defaults": [
   {
    "name": "eph_time_constant, epv_time_constant",
    "value": "0.9",
    "unit": "s",
    "where": "core_sim/include/core_sim/sensors/gps.hpp:27"
   },
   {
    "name": "eph_initial, epv_initial",
    "value": "100",
    "unit": "m",
    "where": "gps.hpp:28-29"
   },
   {
    "name": "eph_final, epv_final",
    "value": "0.1",
    "unit": "m",
    "where": "gps.hpp:30-31"
   },
   {
    "name": "eph_min_3d, eph_min_2d",
    "value": "3.0, 4.0",
    "unit": "m",
    "where": "gps.hpp:33"
   }
  ],
  "assumptions": "The reported eph/epv are only labels: the GPS position and velocity are the exact simulated values, with NO position noise, bias, multipath or latency. The GPS fix reaches 3D after about 3 s (0.9 ln((100-0.1)/(3-0.1)) = 3.2 s). Legacy AirSim GpsSimple has the same structure (GpsSimple.hpp:73-93).",
  "notes": "Reported altitude is geodetic altitude = home altitude - z."
 },
 {
  "id": "PA-14-10",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Airspeed sensor: differential pressure from true airspeed (compressible Pitot formula) and noise",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/airspeed.cpp",
  "lines": "236-271, 155-164",
  "code": "speed_in_air_true = velocity_in_air_true.dot(forward_world);\nspeed_in_air_true = std::max(speed_in_air_true, 0.0f);\nspeed_ratio = speed_in_air_true /\n              EarthUtils::kSeaLevelSpeedOfSound;\noutput_.pressure = ground_truth_.environment->env_info.air_pressure *\n                   (pow((EarthUtils::kSeaLevelTemperature / 5.0f /\n                         ground_truth_.environment->env_info.temperature) *",
  "formula": "V = max((v_ground - w) . f_world, 0);  M0 = V/340.29;  dp = p_static ((1 + (T0/(5T)) M0^2)^(7/2) - 1);  dp += dp GaussMarkov(tau, sigma) + N(0, sigma_u)",
  "variables": {
   "V": "true airspeed along the sensor forward axis [m/s]",
   "dp": "differential (Pitot minus static) pressure [Pa]",
   "T0, T": "sea-level and local temperature [K]",
   "p_static": "static pressure [Pa]",
   "f_world": "forward_xyz rotated by the body orientation [-]"
  },
  "defaults": [
   {
    "name": "forward_xyz",
    "value": "(1, 0, 0)",
    "unit": "-",
    "where": "airspeed.hpp:44"
   },
   {
    "name": "pressure_factor_sigma",
    "value": "0.0365/20",
    "unit": "-",
    "where": "airspeed.hpp:35"
   },
   {
    "name": "pressure_factor_tau",
    "value": "3600",
    "unit": "s",
    "where": "airspeed.hpp:36"
   },
   {
    "name": "uncorrelated_noise_sigma",
    "value": "1.052",
    "unit": "Pa (MS4525DO resolution)",
    "where": "airspeed.hpp:42"
   }
  ],
  "assumptions": "(T0/(5T)) M0^2 = 0.2 (V/a)^2 because a^2 is proportional to T (a0 for sea level, scaled by T0/T). Subsonic isentropic Pitot relation (gamma = 1.4, exponent 7/2). Ground velocity is the finite difference of position, not the twist. No blockage, no angle-of-attack sensitivity, no static-port error. The first update divides by dt with position_last_ initialised at Initialize.",
  "notes": "The output is the differential pressure in Pa, not speed; consumers invert it."
 },
 {
  "id": "PA-14-11",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Distance sensor noise and quantisation",
  "simulator": "Project AirSim",
  "vehicle": "both",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/sensors/distance_sensor.cpp; core_sim/include/core_sim/sensors/distance_sensor.hpp",
  "lines": "distance_sensor.hpp:30-51; distance_sensor.cpp:229-235",
  "code": "float min_distance = 0.2f;  // Minimum distance measured by the sensor\nfloat max_distance = 40.0f;  // Maximum distance measured by the sensor",
  "formula": "not found",
  "variables": {},
  "defaults": [
   {
    "name": "min_distance, max_distance",
    "value": "0.2, 40",
    "unit": "m",
    "where": "distance_sensor.hpp:34-37"
   }
  ],
  "assumptions": "The core_sim distance sensor holds only the range limits and the report rate; the measurement is a ray cast in the Unreal plugin (outside the scope searched), and no noise term appears in core_sim/src/sensors/distance_sensor.cpp (grep for noise/random/gauss gave no match).",
  "notes": "Legacy AirSim fills this gap (PA-14-12)."
 },
 {
  "id": "PA-14-12",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Legacy distance sensor: ray length plus Gaussian noise",
  "simulator": "AirSim (legacy)",
  "vehicle": "both",
  "repo": "microsoft/AirSim",
  "commit": "1ca93f6",
  "file": "AirLib/include/sensors/distance/DistanceSimple.hpp",
  "lines": "DistanceSimple.hpp:83-86; DistanceSimpleParams.hpp:17-18, 41",
  "code": "auto distance = getRayLength(params_.relative_pose + ground_truth.kinematics->pose);\n//add noise in distance (about 0.2m sigma)\ndistance += uncorrelated_noise_.next();\nreal_T uncorrelated_noise_sigma = 0.002f * 100;",
  "formula": "d = d_ray + N(0, 0.2 m)",
  "variables": {
   "d_ray": "true ray-cast range [m]"
  },
  "defaults": [
   {
    "name": "uncorrelated_noise_sigma",
    "value": "0.2",
    "unit": "m",
    "where": "DistanceSimpleParams.hpp:41"
   },
   {
    "name": "min_distance / max_distance",
    "value": "0.2 / 40",
    "unit": "m",
    "where": "DistanceSimpleParams.hpp:17-18"
   },
   {
    "name": "update_frequency",
    "value": "50",
    "unit": "Hz",
    "where": "DistanceSimpleParams.hpp:47"
   }
  ],
  "assumptions": "White noise only, not clipped to the [min, max] range in this code.",
  "notes": "The comment in the params says correlated-noise constants from Sabatini and Genovese were not used because they did not match experiments. Also in: AirLib/include/sensors/distance/DistanceSimpleParams.hpp."
 },
 {
  "id": "PA-15-01",
  "group": 15,
  "group_name": "estimators",
  "quantity": "Simple-flight state estimator: ground-truth pass-through",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "vehicle_apis/multirotor_api/include/simple_flight/AirSimSimpleFlightEstimator.hpp",
  "lines": "23-52, 103-123",
  "code": "// for now we don't do any state estimation and use ground truth (i.e. assume\n// perfect sensors)\nVector3 rpy = TransformUtils::ToRPY(kinematics_->pose.orientation);\nangles.Roll() = rpy[0];\nangles.Pitch() = rpy[1];\nangles.Yaw() = rpy[2];",
  "formula": "state estimate = ground-truth pose, twist and acceleration; angles = ToRPY(q) (PA-9-04); no filter, no sensor fusion",
  "variables": {},
  "defaults": [],
  "assumptions": "The built-in controller sees perfect state: there is no EKF, UKF or complementary filter in this repo. Sensor noise (IMU, GPS, baro) is published to clients but not consumed by the simple-flight estimator.",
  "notes": "For an actual estimator the course must look at PX4 EKF2 (outside this scope). A repo-wide search of the paths under physics/, core_sim/ and vehicle_apis/ finds no Kalman filter implementation."
 },
 {
  "id": "PA-16-01",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Rotor command chain: control signal to thrust, torque, speed and wrench",
  "simulator": "Project AirSim",
  "vehicle": "multirotor",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/rotor.cpp",
  "lines": "357-409",
  "code": "first_order_filter_.SetInput(std::clamp(control_signal, 0.0f, 1.0f));\nfirst_order_filter_.UpdateOutput(dt_sec);\nrotating_speed_raw = std::sqrt(control_signal_filtered * rotor_settings_.max_speed_square);\nthrust_scalar_ = control_signal_filtered * rotor_settings_.max_thrust;\nwrench_force_rotor_frame =\n    rotor_settings_.normal_vector * thrust_scalar_ * air_density_ratio_;",
  "formula": "u in [0,1] -> u_f = LPF(u) -> T = u_f T_max (rho/rho0) along n_hat;  Q = sigma u_f Q_max (rho/rho0);  omega = omega_max sqrt(u_f).  Hover command for weight m g with N identical rotors: u_hover = m g/(N T_max) (derived from T = u T_max, not in the code).",
  "variables": {
   "u": "controller output for this rotor [0,1]",
   "T_max, Q_max": "PA-2-01, PA-2-02 [N, N m]",
   "m": "body mass [kg]"
  },
  "defaults": [],
  "assumptions": "Command is a normalised THRUST (u maps linearly to T), not a PWM/voltage/rpm; there is no motor voltage, battery sag, or ESC curve. A real rotor with T proportional to omega^2 would have u proportional to omega^2: this is what the sqrt in the speed gives.",
  "notes": "Controller side: simple_flight outputs are the mixer's motor commands in [0,1] (PA-12-02, PA-12-03), passed to the rotor in the 'actuator-order' of the robot config."
 },
 {
  "id": "PA-16-02",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Lift-drag surface, tilt and wheel command conventions",
  "simulator": "Project AirSim",
  "vehicle": "fixed-wing",
  "repo": "iamaisim/ProjectAirSim",
  "commit": "c80cff5",
  "file": "core_sim/src/actuators/lift_drag_control_surface.cpp",
  "lines": "lift_drag_control_surface.cpp:194-216; tilt.cpp:213-237; wheel.cpp:361-406",
  "code": "// Convert -1.0 ~ +1.0 control signal to control surface angle (like a servo\n// motor but without any dynamics)\ncontrol_angle_ = settings_.rotation_rate * control_signal_effective;",
  "formula": "control surface: delta = rotation_rate LPF(u), u in [-1,1];  tilt: angle = angle_min + (angle_max - angle_min) LPF(map(u));  wheel: Q_eng = 50 LPF(clamp(u,-1,1))",
  "variables": {
   "u": "normalised command [-]"
  },
  "defaults": [],
  "assumptions": "The comment says 'without any dynamics' but a first-order filter with smoothing_tc is applied (default 0 = none).",
  "notes": "See PA-6-09, PA-3-06, PA-11-08. Also in: core_sim/src/actuators/tilt.cpp; core_sim/src/actuators/wheel.cpp."
 }
];
