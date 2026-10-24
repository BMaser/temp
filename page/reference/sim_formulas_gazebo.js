window.SIM_FORMULAS_GAZEBO = [
 {
  "id": "GZ-01",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Air density used by lift/drag plugins (constant, no altitude dependence)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "96-99, 169",
  "code": "public: double rho = 1.2041;\n...\nthis->rho = _sdf->Get<double>(\"air_density\", this->rho).first;",
  "formula": "ρ = constant (SDF <air_density>), same value at every altitude and temperature",
  "variables": {
   "ρ": "air density [kg/m^3]"
  },
  "defaults": [
   {
    "name": "rho (LiftDrag)",
    "value": "1.2041",
    "unit": "kg/m^3",
    "where": "LiftDrag.cc L99 (20 C, 101.325 kPa per the comment on L97-98)"
   },
   {
    "name": "rho (AdvancedLiftDrag)",
    "value": "1.2041",
    "unit": "kg/m^3",
    "where": "AdvancedLiftDrag.cc L250"
   },
   {
    "name": "air_density (rc_cessna, advanced_plane, standard_vtol SDF)",
    "value": "1.2041",
    "unit": "kg/m^3",
    "where": "PX4-gazebo-models model.sdf files"
   }
  ],
  "assumptions": "Density is a per-plugin constant; it is not coupled to the AirPressure or AirSpeed sensor atmosphere models (GZ-03, GZ-04).",
  "notes": "MulticopterMotorModel has no air density at all (thrust uses a lumped constant kf)."
 },
 {
  "id": "GZ-02",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Air density constant in the lighter-than-air plugin",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lighter_than_air_dynamics/LighterThanAirDynamics.cc",
  "lines": "251-254",
  "code": "if (_sdf->HasElement(\"air_density\"))\n{\n  this->dataPtr->airDensity = SdfParamDouble(_sdf, \"air_density\", 1.225);\n}",
  "formula": "ρ = SDF <air_density>, 1.225 only if the element is present with no value",
  "variables": {
   "ρ": "air density [kg/m^3]"
  },
  "defaults": [
   {
    "name": "airDensity member",
    "value": "uninitialized",
    "unit": "kg/m^3",
    "where": "LighterThanAirDynamics.cc L61 declares 'double airDensity;' with no initializer; 1.225 is applied only inside the HasElement branch"
   }
  ],
  "assumptions": "Airship plugin, not aircraft-relevant; no PX4 model uses it.",
  "notes": "If <air_density> is omitted, airDensity is never set (read of an uninitialized member). Noted only; plugin not used by x500, rc_cessna, advanced_plane or standard_vtol."
 },
 {
  "id": "GZ-03",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Barometric pressure vs altitude (ISA troposphere)",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/AirPressureSensor.cc",
  "lines": "43-52, 166-179",
  "code": "double geoHeight = kEarthRadiusMeters * height / (kEarthRadiusMeters + height);\ndouble tempAtHeight = kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * geoHeight;\nthis->dataPtr->pressure = kPressureOneAtmospherePascals * exp(kAirConstantDimensionless * log(kSeaLevelTempKelvin / tempAtHeight));",
  "formula": "h = h_ref + z;  h_g = R_E·h/(R_E+h);  T = T0 − L·h_g;  p = p0·(T0/T)^C with C = g·M/(R*·(−L)) = 9.80665·28.9644/(8314.32·(−0.0065)) = −5.2559",
  "variables": {
   "h": "height = referenceAltitude + sensor world z [m]",
   "h_g": "geopotential height [m]",
   "R_E": "Earth radius [m]",
   "T": "temperature at height [K]",
   "T0": "sea-level temperature [K]",
   "L": "lapse rate [K/m]",
   "p0": "sea-level pressure [Pa]",
   "C": "kAirConstantDimensionless [-]",
   "p": "static pressure [Pa]"
  },
  "defaults": [
   {
    "name": "kGasConstantNmPerKmolKelvin",
    "value": "8314.32",
    "unit": "N·m/(kmol·K)",
    "where": "AirPressureSensor.cc L43"
   },
   {
    "name": "kMeanMolecularAirWeightKgPerKmol",
    "value": "28.9644",
    "unit": "kg/kmol",
    "where": "L44"
   },
   {
    "name": "kGravityMagnitude",
    "value": "9.80665",
    "unit": "m/s^2",
    "where": "L45"
   },
   {
    "name": "kEarthRadiusMeters",
    "value": "6356766.0",
    "unit": "m",
    "where": "L46"
   },
   {
    "name": "kPressureOneAtmospherePascals",
    "value": "101325.0",
    "unit": "Pa",
    "where": "L47"
   },
   {
    "name": "kSeaLevelTempKelvin",
    "value": "288.15",
    "unit": "K",
    "where": "L48"
   },
   {
    "name": "kTempLapseKelvinPerMeter",
    "value": "0.0065",
    "unit": "K/m",
    "where": "L49"
   },
   {
    "name": "referenceAltitude",
    "value": "0.0",
    "unit": "m",
    "where": "L69 (set by gz-sim via SetReferenceAltitude)"
   }
  ],
  "assumptions": "ISA troposphere only (valid to 11 km); constants copied from RotorS per the comment on L41-42. No temperature offset or weather.",
  "notes": "The constant C evaluates to about -5.2559 (computed by hand from the three constants; the source defines it as an expression, not a literal)."
 },
 {
  "id": "GZ-04",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Local air temperature and density used by the airspeed sensor",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/AirSpeedSensor.cc",
  "lines": "41-51, 166-170",
  "code": "const float temperature_local = kTemperaturMsl - kLapseRate * alt_amsl;\nconst float density_ratio = powf(kTemperaturMsl / temperature_local , 4.256f);\nconst float air_density = kAirDensityMsl / density_ratio;",
  "formula": "T = T0 − L·h;  ρ = ρ0 / (T0/T)^4.256",
  "variables": {
   "h": "altitude AMSL = 0 + sensor world z [m]",
   "T0": "288.15 K",
   "L": "0.0065 K/m",
   "ρ0": "1.225 kg/m^3",
   "ρ": "local density [kg/m^3]"
  },
  "defaults": [
   {
    "name": "kDefaultHomeAltAmsl",
    "value": "0.0",
    "unit": "m",
    "where": "AirSpeedSensor.cc L42"
   },
   {
    "name": "kTemperaturMsl",
    "value": "288.15",
    "unit": "K",
    "where": "L45"
   },
   {
    "name": "kPressureMsl",
    "value": "101325.0",
    "unit": "Pa",
    "where": "L47 (declared, not used in the lines read)"
   },
   {
    "name": "kLapseRate",
    "value": "0.0065",
    "unit": "K/m",
    "where": "L49"
   },
   {
    "name": "kAirDensityMsl",
    "value": "1.225",
    "unit": "kg/m^3",
    "where": "L51"
   }
  ],
  "assumptions": "ISA troposphere; float precision.",
  "notes": "This density (1.225 at MSL) differs from the 1.2041 used by the lift/drag plugins (GZ-01)."
 },
 {
  "id": "GZ-05",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "World gravity vector handed to the physics engine",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/physics/Physics.cc",
  "lines": "1197-1199",
  "code": "sdf::World world;\nworld.SetName(_name->Data());\nworld.SetGravity(_gravity->Data());\nauto worldPtrPhys = this->engine->ConstructWorld(world);",
  "formula": "g (vector) = value of the world Gravity component, taken from the SDF <gravity> of the world file",
  "variables": {
   "g": "gravity acceleration vector in world frame [m/s^2]"
  },
  "defaults": [
   {
    "name": "gravity default",
    "value": "not found",
    "unit": "m/s^2",
    "where": "the default lives in sdformat, which was not read; PX4 world files were not in the repos requested"
   }
  ],
  "assumptions": "Gravity is uniform and constant; per-link enable flag set at L1575-1581 (link.SetEnableGravity).",
  "notes": "Gravity magnitude is NOT fixed in gz-sim. The AirPressureSensor uses a separate constant 9.80665 (GZ-03)."
 },
 {
  "id": "GZ-06",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Gravity set on the DART world",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "dartsim/src/WorldFeatures.cc",
  "lines": "98-102",
  "code": "void WorldFeatures::SetWorldGravity(const Identity &_id, const LinearVectorType &_gravity)\n{\n  ...\n  world->setGravity(_gravity);",
  "formula": "DART applies m·g to each body inside its dynamics; gz-physics only forwards the vector",
  "variables": {
   "g": "gravity vector [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Gravity forces are computed inside DART (external library, only World::step read; see GZ-57).",
  "notes": "bullet-featherstone does the same with worldInfo->world->setGravity (bullet-featherstone/src/WorldFeatures.cc L34)."
 },
 {
  "id": "GZ-07",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Buoyancy force (uniform fluid), usable with air density",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/buoyancy/Buoyancy.cc",
  "lines": "113, 617-637",
  "code": "buoyancy = -this->dataPtr->UniformFluidDensity(linkWorldPose) * _volume->Data() * gravity->Data();\nmath::Vector3d offsetWorld = linkWorldPose.Rot().RotateVector(_centerOfVolume->Data());\nmath::Vector3d torque = offsetWorld.Cross(buoyancy);\nlink.AddWorldWrench(_ecm, buoyancy, torque);",
  "formula": "F_b = −ρ_f · V · g ;  τ = r_cov × F_b  (r_cov = center of volume in world orientation)",
  "variables": {
   "ρ_f": "fluid density [kg/m^3]",
   "V": "link volume from collision geometry [m^3]",
   "g": "gravity vector [m/s^2]",
   "r_cov": "center-of-volume offset [m]"
  },
  "defaults": [
   {
    "name": "fluidDensity",
    "value": "1000",
    "unit": "kg/m^3",
    "where": "Buoyancy.cc L113 (water; for air set <uniform_fluid_density>)"
   }
  ],
  "assumptions": "Applies to every link (or a listed subset) that has a collision volume. Default fluid is water, so the plugin must NOT be loaded for an ordinary multicopter or plane.",
  "notes": "Not used by x500, rc_cessna, advanced_plane or standard_vtol. Relevant only to buoyant aircraft."
 },
 {
  "id": "GZ-08",
  "group": 1,
  "group_name": "atmosphere & gravity",
  "quantity": "Buoyancy with density layers (graded)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/buoyancy/Buoyancy.cc",
  "lines": "221, 247",
  "code": "auto forceMag =  - (vol - prevLayerVol) * _gravity * prevLayerFluidDensity;",
  "formula": "F_layer = −(V_below_plane − V_prev)·g·ρ_layer; one force per density layer, each at its own center of buoyancy",
  "variables": {
   "V": "submerged volume of the layer [m^3]",
   "ρ_layer": "density of the layer [kg/m^3]",
   "g": "gravity vector [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Layers are horizontal planes given by SDF <graded_buoyancy>.",
  "notes": "Gives a density-vs-height model; not used by any PX4 model."
 },
 {
  "id": "GZ-09",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor thrust magnitude",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "565-573",
  "code": "double realMotorVelocity = motorRotVel * this->rotorVelocitySlowdownSim;\nint realMotorVelocitySign = (realMotorVelocity > 0) - (realMotorVelocity < 0);\ndouble thrust = this->turningDirection * realMotorVelocitySign *\n                realMotorVelocity * realMotorVelocity * this->motorConstant;",
  "formula": "ω_real = ω_joint · s ;  T = d · sgn(ω_real) · ω_real² · k_f  (d = +1 ccw, −1 cw)",
  "variables": {
   "ω_joint": "measured joint velocity [rad/s]",
   "s": "rotorVelocitySlowdownSim [-]",
   "d": "turningDirection (ccw=+1, cw=−1) [-]",
   "k_f": "motorConstant [N/(rad/s)^2]",
   "T": "thrust along the link +z [N]"
  },
  "defaults": [
   {
    "name": "motorConstant",
    "value": "8.54858e-06",
    "unit": "N/(rad/s)^2",
    "where": "MulticopterMotorModel.cc L181"
   },
   {
    "name": "rotorVelocitySlowdownSim",
    "value": "10.0",
    "unit": "-",
    "where": "L200"
   }
  ],
  "assumptions": "T is quadratic in speed and independent of airspeed, air density, inflow, or ground. The joint is commanded with sign d (L680), so ω_real has sign d and d·sgn(ω_real) = +1: thrust is positive for both rotation directions.",
  "notes": "Unit comment on L179 says N/(rad/s)^2; the code uses no density term, so k_f lumps ρ, D and C_T."
 },
 {
  "id": "GZ-10",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Where and along which axis the thrust is applied",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "578-583",
  "code": "Link link(this->linkEntity);\nconst auto worldPose = link.WorldPose(_ecm);\nlink.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));",
  "formula": "F_world = R_link · [0, 0, T]ᵀ, applied to the rotor link",
  "variables": {
   "R_link": "rotor link world orientation [-]",
   "T": "thrust [N]"
  },
  "defaults": [],
  "assumptions": "Thrust is along the rotor LINK frame +z, not along the joint axis (joint axis is only used for the drag terms, L617-618). Force goes on the rotor link (a small 0.016 kg link in x500), so it reaches the body through the revolute joint.",
  "notes": "A tilted rotor needs the link z axis aligned with thrust."
 },
 {
  "id": "GZ-11",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Rotor reaction (drag) torque on the airframe",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "646-654",
  "code": "Pose poseDifference = (*parentWorldPose).Inverse() * (*worldPose);\nVector3 dragTorque(0, 0, -this->turningDirection * thrust * this->momentConstant);\nVector3 dragTorqueParentFrame = poseDifference.Rot().RotateVector(dragTorque);\nparentWorldTorque = parentWorldPose->Rot().RotateVector(dragTorqueParentFrame);",
  "formula": "τ_z = −d · T · k_m (rotor frame), rotated into the parent (body) link and applied as a torque on the parent link",
  "variables": {
   "d": "turning direction (+1 ccw, −1 cw) [-]",
   "T": "thrust [N]",
   "k_m": "momentConstant [m]"
  },
  "defaults": [
   {
    "name": "momentConstant",
    "value": "0.016",
    "unit": "m",
    "where": "MulticopterMotorModel.cc L177"
   }
  ],
  "assumptions": "Torque is proportional to thrust (so to ω²), with no separate propeller torque coefficient. Direction: a ccw rotor (d=+1) produces −z torque on the body.",
  "notes": "x500 uses 0.016; standard_vtol lift rotors 0.06 and pusher 0.01; rc_cessna 0.016 (see GZ-19, GZ-43, GZ-45). The torque is accumulated into ExternalWorldWrenchCmd of the parent link (L662-672). No rotor inertia / angular momentum term appears in this file."
 },
 {
  "id": "GZ-12",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Marine/propeller thrust law of the Thruster plugin",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/thruster/Thruster.cc",
  "lines": "593-599",
  "code": "return this->thrustCoefficient * pow(this->propellerDiameter, 4)\n  * abs(_angVel) * _angVel * this->fluidDensity;",
  "formula": "T = C_T · ρ · D⁴ · |n| · n   (n = propeller angular velocity [rad/s]; source comment cites Fossen p.246)",
  "variables": {
   "C_T": "thrustCoefficient [-]",
   "ρ": "fluidDensity [kg/m^3]",
   "D": "propellerDiameter [m]",
   "n": "angular velocity [rad/s]"
  },
  "defaults": [
   {
    "name": "thrustCoefficient",
    "value": "1",
    "unit": "-",
    "where": "Thruster.cc L123"
   },
   {
    "name": "fluidDensity",
    "value": "1000",
    "unit": "kg/m^3",
    "where": "L141 (water)"
   },
   {
    "name": "propellerDiameter",
    "value": "0.02",
    "unit": "m",
    "where": "L144"
   }
  ],
  "assumptions": "Uses n in rad/s with D⁴ (Fossen's T = ρ D⁴ K_T n² form with n in rad/s, not rev/s). Default density is water; set fluid_density=1.2 for air. Not used by any PX4 model.",
  "notes": "This is the only place in the stack with ρ·D⁴·C_T written out."
 },
 {
  "id": "GZ-13",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "Thruster force and torque application",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/thruster/Thruster.cc",
  "lines": "699-702, 755-760",
  "code": "auto jointWorldPose = linkWorldPose * this->dataPtr->jointPose;\nauto unitVector = jointWorldPose.Rot().RotateVector(this->dataPtr->jointAxis).Normalize();\nlink.AddWorldWrench(_ecm, unitVector * desiredThrust, unitVector * torque);",
  "formula": "F = T · û (û = joint axis in world frame);  τ = τ_PID · û",
  "variables": {
   "û": "unit joint axis in world frame [-]",
   "T": "desired thrust [N]",
   "τ_PID": "PID output torque [N·m]"
  },
  "defaults": [],
  "assumptions": "Thrust acts along the joint axis (unlike GZ-10). Torque is only the PID spin-up torque, not a reaction torque proportional to thrust.",
  "notes": "No battery effect on thrust except enable flag (PostUpdate L772)."
 },
 {
  "id": "GZ-14",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Rotor speed first-order lag with different up/down time constants",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "77-94, 673-676",
  "code": "if (_inputState > previousState) {\n  double alphaUp = exp(-_samplingTime / timeConstantUp);\n  outputState = alphaUp * previousState + (1 - alphaUp) * _inputState;\n} else {\n  double alphaDown = exp(-_samplingTime / timeConstantDown);\n  outputState = alphaDown * previousState + (1 - alphaDown) * _inputState;",
  "formula": "ω[k+1] = a·ω[k] + (1−a)·ω_cmd,  a = exp(−Δt/τ),  τ = τ_up if ω_cmd > ω[k] else τ_down  (exact zero-order-hold discretisation of ω̇ = (ω_cmd−ω)/τ)",
  "variables": {
   "ω_cmd": "commanded rotor speed refMotorInput [rad/s]",
   "ω": "filtered speed [rad/s]",
   "Δt": "simulation step [s]",
   "τ_up, τ_down": "time constants [s]"
  },
  "defaults": [
   {
    "name": "timeConstantUp",
    "value": "1/80 = 0.0125",
    "unit": "s",
    "where": "MulticopterMotorModel.cc L208"
   },
   {
    "name": "timeConstantDown",
    "value": "1/40 = 0.025",
    "unit": "s",
    "where": "L204"
   },
   {
    "name": "timeConstantUp, timeConstantDown (x500, rc_cessna, advanced_plane, standard_vtol SDF)",
    "value": "0.0125, 0.025",
    "unit": "s",
    "where": "PX4-gazebo-models model.sdf files"
   }
  ],
  "assumptions": "Filter output is written to the joint velocity command; the thrust at the same step uses the previous step's measured joint velocity (L558), adding about one step of extra delay.",
  "notes": "The filter state is the command-side speed; thrust is computed from the physics joint velocity, which follows JointVelocityCmd."
 },
 {
  "id": "GZ-15",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Motor command saturation",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "527-532",
  "code": "this->refMotorInput = std::min(\n    static_cast<double>(msg->velocity(this->actuatorNumber)),\n    static_cast<double>(this->maxRotVelocity));",
  "formula": "ω_cmd = min(ω_msg, ω_max)",
  "variables": {
   "ω_msg": "actuator message velocity [rad/s]",
   "ω_max": "maxRotVelocity [rad/s]"
  },
  "defaults": [
   {
    "name": "maxRotVelocity",
    "value": "838.0",
    "unit": "rad/s",
    "where": "MulticopterMotorModel.cc L172 (about 8000 rpm)"
   },
   {
    "name": "maxRotVelocity x500",
    "value": "1000.0",
    "unit": "rad/s",
    "where": "x500 model.sdf L14"
   },
   {
    "name": "maxRotVelocity standard_vtol",
    "value": "1500 (lift), 3500 (pusher)",
    "unit": "rad/s",
    "where": "standard_vtol model.sdf"
   }
  ],
  "assumptions": "Only an upper clamp; no lower clamp (a negative command passes through).",
  "notes": "No slew-rate limit other than the first-order filter."
 },
 {
  "id": "GZ-16",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Filtered rotor speed written to the physics joint (slowed-down by a factor)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "678-681",
  "code": "_ecm.SetComponentData<components::JointVelocityCmd>(\n  this->jointEntity,\n  {this->turningDirection * refMotorRotVel / this->rotorVelocitySlowdownSim});",
  "formula": "ω_joint,cmd = d · ω_filtered / s",
  "variables": {
   "d": "turning direction [-]",
   "ω_filtered": "filtered rotor speed [rad/s]",
   "s": "rotorVelocitySlowdownSim [-]"
  },
  "defaults": [
   {
    "name": "rotorVelocitySlowdownSim",
    "value": "10 (x500, rc_cessna, advanced_plane), 20 (standard_vtol)",
    "unit": "-",
    "where": "model.sdf files"
   }
  ],
  "assumptions": "The simulated joint spins s times slower than the modeled rotor to avoid aliasing; thrust re-multiplies by s (GZ-09).",
  "notes": "Rotor link gyroscopic effect in the physics engine therefore corresponds to the slowed speed, not the real one."
 },
 {
  "id": "GZ-17",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Aliasing (Nyquist) warning check on joint speed",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "558-564",
  "code": "double motorRotVel = jointVelocity->Data()[0];\nif (motorRotVel / (2 * GZ_PI) > 1 / (2 * this->samplingTime))\n  gzerr << \"Aliasing on motor ...\";",
  "formula": "warn if  ω_joint/(2π) > 1/(2Δt)",
  "variables": {
   "ω_joint": "joint speed [rad/s]",
   "Δt": "samplingTime [s]"
  },
  "defaults": [
   {
    "name": "samplingTime",
    "value": "0.01",
    "unit": "s",
    "where": "MulticopterMotorModel.cc L158 (overwritten from _info.dt every step, L475-476)"
   }
  ],
  "assumptions": "Diagnostic only, does not change physics.",
  "notes": "Explains why rotorVelocitySlowdownSim exists."
 },
 {
  "id": "GZ-18",
  "group": 3,
  "group_name": "motor dynamics",
  "quantity": "Thruster propeller speed PID",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/thruster/Thruster.cc",
  "lines": "726-736",
  "code": "auto currentAngular = (link.WorldAngularVelocity(_ecm))->Dot(unitVector);\nauto angularError = currentAngular - desiredPropellerAngVel;\nif (abs(angularError) > 0.1)\n  torque = this->dataPtr->propellerController.Update(angularError, _info.dt);",
  "formula": "e = ω_link·û − ω_des;  τ = PID(e, Δt) only if |e| > 0.1 rad/s",
  "variables": {
   "ω_link": "world angular velocity of the link [rad/s]",
   "û": "joint axis [-]",
   "ω_des": "desired propeller speed [rad/s]"
  },
  "defaults": [
   {
    "name": "PID gains",
    "value": "not found in the lines read (math::PID configured from SDF p_gain/i_gain etc., defaults not read)",
    "unit": "-",
    "where": "Thruster.cc L105-107"
   }
  ],
  "assumptions": "Applied only when velocityControl is false.",
  "notes": "Error is defined as current − desired, so the PID gains must be negative for stable behaviour (sign not verified in source)."
 },
 {
  "id": "GZ-19",
  "group": 2,
  "group_name": "rotor thrust & torque",
  "quantity": "PX4 x500 quadrotor motor parameters",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/x500/model.sdf",
  "lines": "7-23",
  "code": "<turningDirection>ccw</turningDirection>\n<timeConstantUp>0.0125</timeConstantUp>\n<timeConstantDown>0.025</timeConstantDown>\n<maxRotVelocity>1000.0</maxRotVelocity>\n<motorConstant>8.54858e-06</motorConstant>\n<momentConstant>0.016</momentConstant>",
  "formula": "T_max = k_f·ω_max² = 8.54858e-06 · 1000² = 8.55 N per rotor (hand computation); 4 rotors give 34.2 N vs weight 2.0 kg·9.8 = 19.6 N",
  "variables": {
   "k_f": "motorConstant [N/(rad/s)^2]",
   "ω_max": "maxRotVelocity [rad/s]"
  },
  "defaults": [
   {
    "name": "rotors 0,1 turningDirection",
    "value": "ccw",
    "unit": "-",
    "where": "x500 model.sdf L11, L28"
   },
   {
    "name": "rotors 2,3 turningDirection",
    "value": "cw",
    "unit": "-",
    "where": "L45, L62"
   },
   {
    "name": "rotorDragCoefficient",
    "value": "8.06428e-05",
    "unit": "-",
    "where": "L19"
   },
   {
    "name": "rollingMomentCoefficient",
    "value": "1e-06",
    "unit": "-",
    "where": "L20"
   },
   {
    "name": "rotorVelocitySlowdownSim",
    "value": "10",
    "unit": "-",
    "where": "L21"
   },
   {
    "name": "mass x500_base base_link",
    "value": "2.0",
    "unit": "kg",
    "where": "x500_base model.sdf L9"
   },
   {
    "name": "inertia ixx, iyy, izz",
    "value": "0.0216667, 0.0216667, 0.04",
    "unit": "kg·m^2",
    "where": "x500_base model.sdf L11-16"
   },
   {
    "name": "rotor link mass",
    "value": "0.016077",
    "unit": "kg",
    "where": "x500_base L321"
   },
   {
    "name": "rotor link inertia ixx, iyy, izz",
    "value": "3.8465e-07, 2.6116e-05, 2.6499e-05",
    "unit": "kg·m^2",
    "where": "x500_base L323-325"
   },
   {
    "name": "rotor positions (x,y) m",
    "value": "r0 (0.174,-0.174), r1 (-0.174,0.174), r2 (0.174,0.174), r3 (-0.174,-0.174)",
    "unit": "m",
    "where": "x500_base L319, 391, 463, 535"
   }
  ],
  "assumptions": "Uses deprecated <motorNumber> (the plugin accepts it with a warning, gz-sim L291-296). Hover per-rotor thrust about 4.9 N needs ω of about 757 rad/s (hand computation sqrt(4.9/8.54858e-6)).",
  "notes": "x500_base declares an empty <velocity_decay/> (L20), so no linear or angular damping beyond rotor drag."
 },
 {
  "id": "GZ-20",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "Rotor in-plane air drag force (blade flapping / induced drag)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "613-629",
  "code": "Vector3 relativeWindVelocityWorld = bodyVelocityWorld - windSpeedWorld;\nVector3 bodyVelocityPerpendicular = relativeWindVelocityWorld -\n  (relativeWindVelocityWorld.Dot(jointAxis) * jointAxis);\nVector3 airDrag = -std::abs(realMotorVelocity) * this->rotorDragCoefficient * bodyVelocityPerpendicular;\nlink.AddWorldForce(_ecm, airDrag);",
  "formula": "v_rel = v_link − v_wind;  v_⊥ = v_rel − (v_rel·â)â;  F_drag = −|ω| · c_d · v_⊥  (â = joint axis in world frame; source cites Martin and Salaun 2010, F = −ω λ₁ v_⊥)",
  "variables": {
   "v_link": "rotor link world linear velocity [m/s]",
   "v_wind": "world wind velocity [m/s]",
   "â": "rotor axis [-]",
   "ω": "real rotor speed [rad/s]",
   "c_d": "rotorDragCoefficient [kg/rad, see notes]"
  },
  "defaults": [
   {
    "name": "rotorDragCoefficient",
    "value": "1.0e-4",
    "unit": "N per (rad/s)(m/s)",
    "where": "MulticopterMotorModel.cc L193 (comment says N/(m/s^2))"
   },
   {
    "name": "rotorDragCoefficient x500",
    "value": "8.06428e-05",
    "unit": "same",
    "where": "x500 model.sdf L19"
   }
  ],
  "assumptions": "Linear in airspeed perpendicular to the axis and linear in |ω|. Axial inflow (along â) is removed, so there is NO thrust loss from axial/climb velocity.",
  "notes": "Dimension of c_d: F [N] = |ω| [rad/s] · c_d · v [m/s], so c_d has units N·s²/(m·rad) = kg/rad; the source comment unit 'N/(m/s^2)' does not match this. The wind component pointer is dereferenced without a null check (L608-611)."
 },
 {
  "id": "GZ-21",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "Rotor rolling moment from lateral flow",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "656-661",
  "code": "Vector3 rollingMoment;\n// - \\omega * \\mu_1 * V_A^{\\perp}\nrollingMoment = -std::abs(realMotorVelocity) * this->rollingMomentCoefficient * bodyVelocityPerpendicular;\nparentWorldTorque += rollingMoment;",
  "formula": "M_roll = −|ω| · μ₁ · v_⊥, added to the torque on the parent (body) link",
  "variables": {
   "ω": "real rotor speed [rad/s]",
   "μ₁": "rollingMomentCoefficient",
   "v_⊥": "relative air velocity perpendicular to the rotor axis [m/s]"
  },
  "defaults": [
   {
    "name": "rollingMomentCoefficient",
    "value": "1.0e-6",
    "unit": "N·m per (rad/s)(m/s)",
    "where": "MulticopterMotorModel.cc L189 (x500 SDF L20 same)"
   }
  ],
  "assumptions": "Torque is a vector parallel to v_⊥ (not perpendicular, as a true rolling moment about the hub would be); it is added in world frame with no lever arm.",
  "notes": "Taken from the same Martin and Salaun model as GZ-20."
 },
 {
  "id": "GZ-22",
  "group": 4,
  "group_name": "rotor aerodynamics",
  "quantity": "Advance-ratio dependent thrust coefficient (Thruster)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/thruster/Thruster.cc",
  "lines": "585-590",
  "code": "this->thrustCoefficient = this->alpha1 + this->alpha2 *\n    (((1 - this->wakeFraction) * this->linearVelocity)\n    / (this->propellerAngVel * this->propellerDiameter));",
  "formula": "C_T = α₁ + α₂ · J_a,   J_a = (1 − w)·V / (n·D)",
  "variables": {
   "w": "wakeFraction [-]",
   "V": "linear speed of the vehicle link [m/s]",
   "n": "propeller angular velocity [rad/s]",
   "D": "propellerDiameter [m]",
   "α₁, α₂": "coefficients [-]"
  },
  "defaults": [
   {
    "name": "wakeFraction",
    "value": "0.2",
    "unit": "-",
    "where": "Thruster.cc L130"
   },
   {
    "name": "alpha_1",
    "value": "1",
    "unit": "-",
    "where": "L134"
   },
   {
    "name": "alpha_2",
    "value": "0",
    "unit": "-",
    "where": "L138"
   }
  ],
  "assumptions": "Only updated when thrust_coefficient is NOT set in SDF and n is nonzero (L567-571). V is the speed magnitude of the link (L763-765), not its component along the shaft. With n in rad/s the usual advance ratio J = V/(nD) is off by a factor 2π.",
  "notes": "This is the only advance-ratio effect in the Gazebo stack; MulticopterMotorModel has none."
 },
 {
  "id": "GZ-23",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "x500 body drag",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/x500_base/model.sdf",
  "lines": "20",
  "code": "<velocity_decay />",
  "formula": "not found",
  "variables": {},
  "defaults": [
   {
    "name": "velocity_decay linear/angular",
    "value": "empty element (engine default, assumed 0)",
    "unit": "-",
    "where": "x500_base model.sdf L20"
   }
  ],
  "assumptions": "No plugin in x500 or x500_base models models a body drag area or fuselage drag; only the rotor drag of GZ-20 acts.",
  "notes": "Searched all <plugin> tags of x500_base (none) and x500 (4 motor models plus MotorFailureSystem). Zero-effect of an empty velocity_decay on DART is not verified in source."
 },
 {
  "id": "GZ-24",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "Airship viscous force and moment",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lighter_than_air_dynamics/LighterThanAirDynamics.cc",
  "lines": "381-420",
  "code": "double forceViscMag_ = q0EpsV*(-forceInviscCoeff*std::sin(2*gammaEpsV) + forceViscCoeff*std::sin(gammaEpsV)*std::sin(gammaEpsV));\ndouble momentViscMag_ = q0EpsV*(-momentInviscCoeff*std::sin(2*gammaEpsV) + momentViscCoeff*std::sin(gammaEpsV)*std::sin(gammaEpsV));",
  "formula": "v_ε = v + ω × (−ε,0,0);  q = ½ρ|v_ε|²;  γ = atan2(√(v_y²+v_z²), v_x);  F = q(−c_fi·sin2γ + c_fv·sin²γ)·(0,−n_y,−n_z);  M = q(−c_mi·sin2γ + c_mv·sin²γ)·(0,n_z,−n_y)",
  "variables": {
   "ε": "eps_v offset [m]",
   "γ": "angle between velocity and body x axis [rad]",
   "c_fi, c_fv, c_mi, c_mv": "force/moment inviscid and viscous coefficients",
   "n_y, n_z": "unit normal components of the transverse velocity"
  },
  "defaults": [
   {
    "name": "all coefficients",
    "value": "no default; SDF required (errors printed if missing, L273-332)",
    "unit": "-",
    "where": "LighterThanAirDynamics.cc"
   }
  ],
  "assumptions": "Airship (Fossen / Li et al. style) model; velocities are in the body frame (L378-379). Not used in any PX4 aircraft model.",
  "notes": "Relevant only to lighter-than-air vehicles, recorded for completeness. windVector is stored (L101-115) but never used in PreUpdate: wind has no effect here."
 },
 {
  "id": "GZ-25",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "Airship added-mass force and torque",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lighter_than_air_dynamics/LighterThanAirDynamics.cc",
  "lines": "192-217, 423-433",
  "code": "math::Vector3d forces = skewAngVel * (m11 * lin_vel + m12 * ang_vel);\nmath::Vector3d torque = skewLinVel * (m11 * lin_vel*0 + m12 * ang_vel) + skewAngVel * (m21 * lin_vel + m22 * ang_vel);",
  "formula": "F_am = −[ω]× (M11 v + M12 ω);  τ_am = −( [v]× M12 ω + [ω]× (M21 v + M22 ω) )",
  "variables": {
   "M11..M22": "blocks of the fluid added-mass matrix (from the link's WorldFluidAddedMassMatrix) [kg, kg·m, kg·m^2]",
   "v": "body linear velocity [m/s]",
   "ω": "body angular velocity [rad/s]"
  },
  "defaults": [],
  "assumptions": "The M11·v term of the torque is multiplied by 0 on purpose (comment L210-212: already inside the inviscid term).",
  "notes": "Airship only."
 },
 {
  "id": "GZ-26",
  "group": 5,
  "group_name": "body aerodynamics",
  "quantity": "Airship axial drag",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lighter_than_air_dynamics/LighterThanAirDynamics.cc",
  "lines": "435-441",
  "code": "double q0 = DynamicPressure(linearVel, this->dataPtr->airDensity);\ndouble angleOfAttack = std::atan2(linearVel[2], linearVel[0]);\nauto forceAxialDrag = math::Vector3d(-q0 * axialDragCoeff * std::cos(angleOfAttack) * std::cos(angleOfAttack), 0, 0);",
  "formula": "F_x = −q · c_ax · cos²(atan2(v_z, v_x)),  q = ½ρ|v|²",
  "variables": {
   "c_ax": "axial_drag_coeff [m^2]",
   "q": "dynamic pressure [Pa]"
  },
  "defaults": [],
  "assumptions": "Along body x only.",
  "notes": "Airship only."
 },
 {
  "id": "GZ-27",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: air-relative velocity at the center of pressure",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "299-313",
  "code": "const auto cpWorld = pose.Rot().RotateVector(this->cp);\nauto vel = worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld);\nif (windLinearVel != nullptr){\n  vel = worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld) - windLinearVel->Data(); }\nif (vel.Length() <= 0.01) return;",
  "formula": "v = v_link + ω × R·r_cp − v_wind;  skip the update if |v| ≤ 0.01 m/s",
  "variables": {
   "v_link": "world linear velocity of the link origin [m/s]",
   "ω": "world angular velocity [rad/s]",
   "r_cp": "center of pressure in link frame [m]",
   "v_wind": "world wind velocity (air motion) [m/s]"
  },
  "defaults": [
   {
    "name": "cp",
    "value": "(0,0,0)",
    "unit": "m",
    "where": "LiftDrag.cc L120"
   }
  ],
  "assumptions": "v_link is the velocity of the link frame origin; the code adds ω × r_cp but r_cp is measured from the link frame origin as written, not from the center of mass (see the comment on L506-509).",
  "notes": "Wind convention: wind velocity is the velocity of the AIR; air-relative velocity = body − wind. Quote condensed from several code lines."
 },
 {
  "id": "GZ-28",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: sweep angle and angle of attack",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "338-400",
  "code": "const auto spanwiseI = forwardI.Cross(upwardI).Normalize();\ndouble sinSweepAngle = math::clamp(spanwiseI.Dot(velI), -1.0, 1.0);\ndouble cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;\nconst auto velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;\nconst double cosAlpha = math::clamp(liftI.Dot(upwardI), -1.0, 1.0);\ndouble alpha = this->alpha0 - std::acos(cosAlpha);\nif (liftI.Dot(forwardI) >= 0.0) alpha = this->alpha0 + std::acos(cosAlpha);",
  "formula": "ŝ = f̂ × û;  sinΛ = ŝ·v̂;  cos²Λ = 1 − sin²Λ;  v_LD = v − (v·ŝ)ŝ;  drag dir d̂ = −v̂_LD;  lift dir l̂ = ŝ × v_LD / |…|;  α = α₀ ± acos(l̂·û) (sign from l̂·f̂), then wrapped into [−90°, +90°]",
  "variables": {
   "f̂, û, ŝ": "forward, upward, spanwise unit vectors in world frame [-]",
   "Λ": "sweep angle [rad]",
   "α₀": "zero-lift angle a0 [rad]",
   "α": "angle of attack [rad]"
  },
  "defaults": [
   {
    "name": "forward",
    "value": "(1,0,0)",
    "unit": "-",
    "where": "LiftDrag.cc L124"
   },
   {
    "name": "upward",
    "value": "(0,0,1)",
    "unit": "-",
    "where": "L129"
   },
   {
    "name": "a0 (alpha0)",
    "value": "0.0",
    "unit": "rad",
    "where": "L116"
   },
   {
    "name": "non-reversible rule",
    "value": "no force if f̂·v ≤ 0",
    "unit": "-",
    "where": "L318-323"
   }
  ],
  "assumptions": "Sweep correction multiplies all coefficients by cos²Λ. The angle formula returns α with the sign defined by the lift direction relative to forward.",
  "notes": "Sideslip is only represented through the sweep factor; no explicit beta. Quote condensed from several code lines."
 },
 {
  "id": "GZ-29",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: dynamic pressure and lift coefficient with post-stall branch",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "402-435",
  "code": "const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;\nif (alpha > this->alphaStall) { cl = (this->cla * this->alphaStall + this->claStall * (alpha - this->alphaStall)) * cos2SweepAngle; cl = std::max(0.0, cl); }\nelse if (alpha < -this->alphaStall) { cl = (-this->cla * this->alphaStall + this->claStall * (alpha + this->alphaStall)) * cos2SweepAngle; cl = std::min(0.0, cl); }\nelse cl = this->cla * alpha * cos2SweepAngle;\ncl = cl + this->controlJointRadToCL * controlJointPosition->Data()[0];\nmath::Vector3d lift = cl * q * this->area * liftI;",
  "formula": "q = ½ρ|v_LD|²;  C_L = cos²Λ·{ C_Lα·α (|α|≤α_s);  C_Lα·α_s + C_Lα,s·(α−α_s) (α>α_s, clamped ≥0);  −C_Lα·α_s + C_Lα,s·(α+α_s) (α<−α_s, clamped ≤0) } + k_δ·δ;  L = C_L·q·S·l̂",
  "variables": {
   "C_Lα": "cla, lift slope [1/rad]",
   "C_Lα,s": "cla_stall, post-stall slope [1/rad]",
   "α_s": "alpha_stall [rad]",
   "k_δ": "control_joint_rad_to_cl [per rad]",
   "δ": "control joint position [rad]",
   "S": "area [m^2]"
  },
  "defaults": [
   {
    "name": "cla",
    "value": "1.0",
    "unit": "1/rad",
    "where": "LiftDrag.cc L67"
   },
   {
    "name": "alphaStall",
    "value": "π/2",
    "unit": "rad",
    "where": "L80"
   },
   {
    "name": "claStall",
    "value": "0.0",
    "unit": "1/rad",
    "where": "L83"
   },
   {
    "name": "controlJointRadToCL",
    "value": "4.0",
    "unit": "per rad",
    "where": "L133"
   },
   {
    "name": "area",
    "value": "1.0",
    "unit": "m^2",
    "where": "L113"
   }
  ],
  "assumptions": "CL has no constant CL0 offset in this plugin: lift is zero at α = 0 (zero-lift angle a0 shifts α, GZ-28). Control surface adds to C_L linearly, with C_L changes applied after sweep correction.",
  "notes": "/// todo comment on L431: cm and cd do not change with the control surface (apart from cm_delta, GZ-31)."
 },
 {
  "id": "GZ-30",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: drag coefficient and drag force",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "437-458",
  "code": "if (alpha > this->alphaStall) cd = (this->cda * this->alphaStall + this->cdaStall * (alpha - this->alphaStall)) * cos2SweepAngle;\nelse if (alpha < -this->alphaStall) cd = (-this->cda * this->alphaStall + this->cdaStall * (alpha + this->alphaStall)) * cos2SweepAngle;\nelse cd = (this->cda * alpha) * cos2SweepAngle;\ncd = std::fabs(cd);\nmath::Vector3d drag = cd * q * this->area * dragDirection;",
  "formula": "C_D = |cos²Λ · { C_Dα·α  (|α|≤α_s);  C_Dα·α_s + C_Dα,s·(α−α_s)  (α>α_s);  −C_Dα·α_s + C_Dα,s·(α+α_s)  (α<−α_s) }|;  D = C_D·q·S·d̂",
  "variables": {
   "C_Dα": "cda [1/rad]",
   "C_Dα,s": "cda_stall [1/rad]"
  },
  "defaults": [
   {
    "name": "cda",
    "value": "0.01",
    "unit": "1/rad",
    "where": "LiftDrag.cc L72"
   },
   {
    "name": "cdaStall",
    "value": "1.0",
    "unit": "1/rad",
    "where": "L87"
   }
  ],
  "assumptions": "Drag is proportional to |α| and is ZERO at α = 0: there is no zero-lift drag C_D0 and no induced drag term C_L²/(πAR e). The rc_cessna SDF uses cda 1.5 or 0.6417, standard_vtol 0.6417.",
  "notes": "So a fixed-wing with this plugin has no parasitic drag at zero angle of attack (apart from fuselage collision shapes, which produce none). Quote condensed from several code lines."
 },
 {
  "id": "GZ-31",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: pitching moment",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "460-489",
  "code": "cm = this->cma * alpha * cos2SweepAngle;   // (+ stall branches with cmaStall, clamped by sign)\ncm += this->cm_delta * controlJointPosition->Data()[0];\nmath::Vector3d moment = cm * q * this->area * spanwiseI;",
  "formula": "C_M = cos²Λ·C_Mα·α (with stall branches like C_L) + k_m·δ;  M = C_M·q·S·ŝ",
  "variables": {
   "C_Mα": "cma [1/rad]",
   "k_m": "cm_delta [per rad]"
  },
  "defaults": [
   {
    "name": "cma",
    "value": "0.0",
    "unit": "1/rad",
    "where": "LiftDrag.cc L77"
   },
   {
    "name": "cmaStall",
    "value": "0.0",
    "unit": "1/rad",
    "where": "L90"
   },
   {
    "name": "cm_delta",
    "value": "0.0",
    "unit": "per rad",
    "where": "L94"
   }
  ],
  "assumptions": "Moment axis is the spanwise direction; no mean chord in the formula (C_M·q·S has dimension N·m only if S is read as area × chord).",
  "notes": "All PX4 rc_cessna / standard_vtol blocks set cma = 0, so pitch stability comes only from the lift offset at cp."
 },
 {
  "id": "GZ-32",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "LiftDrag: wrench applied to the link",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/lift_drag/LiftDrag.cc",
  "lines": "491-515",
  "code": "math::Vector3d force = lift + drag;\nmath::Vector3d torque = moment;\n...\nconst auto totalTorque = torque + cpWorld.Cross(force);\nlink.AddWorldWrench(_ecm, force, totalTorque);",
  "formula": "F = L + D;  τ = M + (R·r_cp) × F  (about the link origin, world frame)",
  "variables": {
   "r_cp": "center of pressure in link frame [m]"
  },
  "defaults": [],
  "assumptions": "Each wing/control surface of rc_cessna and standard_vtol is a separate plugin instance on the same base_link, with its own cp, so total aero force is a sum of 6 or 3 instances.",
  "notes": "NaN and inf are zeroed with Vector3::Correct() (L495-497)."
 },
 {
  "id": "GZ-33",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: stability and body axes, air velocity, span and mean chord",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "514-557, 570-572",
  "code": "air_velocity = worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld) - windLinearVel->Data();\nbody_z_axis = -1*(pose.Rot().RotateVector(this->upward));\nbody_y_axis =  body_z_axis.Cross(body_x_axis);\nvelInLDPlane = air_velocity - air_velocity.Dot(body_y_axis)*body_y_axis;\ndouble span = std::sqrt(this->area*this->AR);\ndouble rr = AngVel->Data()[0]; double pr = -1*AngVel->Data()[1]; double yr = -1*AngVel->Data()[2];",
  "formula": "v_a = v + ω×R·r_cp − v_wind;  x_b = f̂, z_b = −û, y_b = z_b × x_b;  v_LD = v_a − (v_a·y_b)y_b;  stability x_s = v_LD/|v_LD|, y_s = y_b, z_s = x_s × y_s;  b = √(S·AR);  c̄ = S/b if mac = 0;  p = ω_x, q = −ω_y, r = −ω_z",
  "variables": {
   "b": "wing span [m]",
   "c̄": "mean aerodynamic chord mac [m]",
   "p, q, r": "roll, pitch, yaw rates in AVL sign convention (FRD-like) [rad/s]",
   "S": "area [m^2]",
   "AR": "aspect ratio [-]"
  },
  "defaults": [
   {
    "name": "AR",
    "value": "0.0",
    "unit": "-",
    "where": "AdvancedLiftDrag.cc L231"
   },
   {
    "name": "mac",
    "value": "0.0 (replaced by S/b)",
    "unit": "m",
    "where": "L234, L548-554"
   },
   {
    "name": "eff",
    "value": "0.0",
    "unit": "-",
    "where": "L237"
   },
   {
    "name": "stop if speed ≤",
    "value": "1e-9",
    "unit": "m/s",
    "where": "L538"
   }
  ],
  "assumptions": "Axis convention follows AVL (z down, y right, x forward). Rates are read from the link AngularVelocity component, which is the body-frame angular velocity (L562, gz-sim Physics.cc L4078-4108), so p,q,r flip signs of y and z to the AVL frame.",
  "notes": "AR defaults to 0 and eff to 0: the induced-drag term C_L²/(π·AR·eff) divides by zero unless both are set (SDF sets AR 6.5, eff 0.97 for advanced_plane)."
 },
 {
  "id": "GZ-34",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: angle of attack, sideslip, dynamic pressure",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "576-590",
  "code": "double stabx_proj_bodyx = stability_x_axis.Dot(body_x_axis);\ndouble stabx_proj_bodyz = stability_x_axis.Dot(body_z_axis);\nthis->alpha = atan2(stabx_proj_bodyz, stabx_proj_bodyx);\nthis->beta = (atan2(velSW, velFW));\ndouble dyn_pres = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;\ndouble half_rho_vel = 0.5 * this->rho * speedInLDPlane;",
  "formula": "α = atan2(x_s·z_b, x_s·x_b);  β = atan2(v_a·y_b, v_a·x_b);  q̄ = ½ρV_LD²;  q̃ = ½ρV_LD",
  "variables": {
   "α": "angle of attack [rad]",
   "β": "sideslip [rad]",
   "V_LD": "speed in the lift-drag plane [m/s]"
  },
  "defaults": [],
  "assumptions": "Full-range α (±180°); β from the body axes, uses the full air velocity.",
  "notes": "q̃ = ½ρV multiplies every rate-derivative term (GZ-36 to GZ-42), i.e. the rate terms are scaled by ½ρV, not by ½ρV² times a non-dimensional rate."
 },
 {
  "id": "GZ-35",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: stall blending function",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "240, 596-599",
  "code": "public: double M = 15.0;\ndouble sigma = (1+exp(-1*this->M*(this->alpha-this->alphaStall))+exp(this->M*(this->alpha+this->alphaStall)))/((1+exp(-1*this->M*(this->alpha-this->alphaStall)))*(1+exp(this->M*(this->alpha+this->alphaStall))));",
  "formula": "σ(α) = [1 + e^{−M(α−α_s)} + e^{M(α+α_s)}] / {[1 + e^{−M(α−α_s)}][1 + e^{M(α+α_s)}]},  M = 15",
  "variables": {
   "M": "blending sharpness [1/rad]",
   "α_s": "alpha_stall [rad]"
  },
  "defaults": [
   {
    "name": "M",
    "value": "15.0",
    "unit": "1/rad",
    "where": "AdvancedLiftDrag.cc L240 (hard-coded member, not read from SDF)"
   },
   {
    "name": "alphaStall",
    "value": "π/2",
    "unit": "rad",
    "where": "L128"
   }
  ],
  "assumptions": "σ → 0 for |α| < α_s (pre-stall model), σ → 1 for |α| > α_s (post-stall model).",
  "notes": "Used by CL and CD."
 },
 {
  "id": "GZ-36",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: lift coefficient (pre-stall linear, post-stall flat plate)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "615-619, 657, 660-663",
  "code": "CL = (1-sigma)*(this->CL0 + this->CLa*this->alpha) + sigma*(2*(this->alpha/abs(this->alpha))*sinAlpha*sinAlpha*cosAlpha);\nCL = CL + this->CLb*this->beta;\nCL = CL+CL_ctrl_tot;\nlift = (CL * dyn_pres + (CLp*(rr*span/2)*half_rho_vel) + (CLq*(pr*mac/2)*half_rho_vel) + (CLr*(yr*span/2)*half_rho_vel)) * (area * (-1 * stability_z_axis));",
  "formula": "C_L = (1−σ)(C_L0 + C_Lα·α) + σ·2·sgn(α)·sin²α·cosα + C_Lβ·β + C_L,ctrl;  L = [C_L·q̄ + (C_Lp·pb/2 + C_Lq·q c̄/2 + C_Lr·rb/2)·q̃] · S · (−z_s)",
  "variables": {
   "C_L0": "CL0 [-]",
   "C_Lα": "CLa [1/rad]",
   "C_Lβ": "CLb [1/rad]",
   "C_Lp,q,r": "rate derivatives [-]"
  },
  "defaults": [
   {
    "name": "CL0, CLa, CLb",
    "value": "0.0, 0.0, 0.0",
    "unit": "-",
    "where": "AdvancedLiftDrag.cc L87, L97, L113"
   }
  ],
  "assumptions": "Quasi-steady AVL coefficient build-up. The post-stall term uses unqualified abs(alpha) (L616): if this resolves to the integer abs(int) the ratio α/|α| is not a sign function. Not verified which overload is selected.",
  "notes": "The SDF tags CLa_stall, CDa_stall (advanced_plane) are never read by Load (it reads only alpha_stall and Cema_stall), and a0 is also unused here. Quote condensed from several code lines."
 },
 {
  "id": "GZ-37",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: control surface coefficient sums",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "629-653",
  "code": "controlAngle = tmp_controlJointPosition->Data()[0] * 180 / GZ_PI;\nCL_ctrl_tot += controlAngle*this->CL_ctrl[i]*this->ctrl_surface_direction[i];\nCD_ctrl_tot += controlAngle*this->CD_ctrl[i]*this->ctrl_surface_direction[i];",
  "formula": "δ_i[deg] = δ_i[rad]·180/π;  ΔC_x = Σ_i δ_i[deg]·C_x,ctrl,i·dir_i  for x ∈ {L, D, Y, l, m, n}",
  "variables": {
   "δ_i": "control joint position [rad, converted to deg]",
   "C_x,ctrl,i": "AVL control derivative per degree [1/deg]",
   "dir_i": "direction ±1 [-]"
  },
  "defaults": [],
  "assumptions": "Control derivatives are PER DEGREE (AVL convention); deflection linear, no saturation in the plugin beyond the joint limits.",
  "notes": "Comment L639-640 warns AVL and Gazebo positive deflection may differ in sign. advanced_plane servo_2 uses direction −1."
 },
 {
  "id": "GZ-38",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: drag coefficient (quadratic polar plus flat-plate post stall)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "243-244, 690-704",
  "code": "double CD_fp = 2 / (1 + exp(this->CD_fp_k1 + this->CD_fp_k2 * (std::max(this->AR, 1 / this->AR))));\nCD = (1 - sigma) * (this->CD0 + (CL*CL) / (GZ_PI * this->AR * this->eff)) + sigma * abs(CD_fp * (0.5 - 0.5 * cos(2 * this->alpha)));\nCD = CD + CD_ctrl_tot;\ndrag = (CD*dyn_pres + (CDp*(rr*span/2)*half_rho_vel) + (CDq*(pr*mac/2)*half_rho_vel) + (CDr*(yr*span/2)*half_rho_vel)) * (area * (-1*stability_x_axis));",
  "formula": "C_D,fp = 2/(1 + exp(k₁ + k₂·max(AR, 1/AR)));  C_D = (1−σ)(C_D0 + C_L²/(π·AR·e)) + σ·|C_D,fp·(½ − ½cos2α)| + C_D,ctrl;  D = [C_D·q̄ + (C_Dp·pb/2 + C_Dq·qc̄/2 + C_Dr·rb/2)·q̃]·S·(−x_s)",
  "variables": {
   "C_D0": "zero-lift drag [-]",
   "AR": "aspect ratio [-]",
   "e": "Oswald efficiency eff [-]",
   "k₁, k₂": "flat-plate sigmoid fit constants [-]"
  },
  "defaults": [
   {
    "name": "CD_fp_k1",
    "value": "-0.224",
    "unit": "-",
    "where": "AdvancedLiftDrag.cc L243"
   },
   {
    "name": "CD_fp_k2",
    "value": "-0.115",
    "unit": "-",
    "where": "L244"
   },
   {
    "name": "CD0",
    "value": "0.0",
    "unit": "-",
    "where": "L90"
   }
  ],
  "assumptions": "Sigmoid fitted to Ostowari and Naik wind tunnel data for a NACA 44XX section (source comment L669-690; the author notes k1, k2 may need tuning). Uses unqualified abs(...) again (L693-694).",
  "notes": "This is the only polar in the stack with induced drag C_L²/(πARe). Quote condensed from several code lines."
 },
 {
  "id": "GZ-39",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: side force",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "708-713",
  "code": "double CY = this->CYa * this->alpha + this->CYb * this->beta + CY_ctrl_tot;\ngz::math::Vector3d sideforce = (CY * dyn_pres + (this->CYp*(rr*span/2)*half_rho_vel) + (this->CYq*(pr*this->mac/2)*half_rho_vel) + (this->CYr*(yr*span/2)*half_rho_vel)) * (this->area * stability_y_axis);",
  "formula": "C_Y = C_Yα·α + C_Yβ·β + C_Y,ctrl;  Y = [C_Y·q̄ + (C_Yp·pb/2 + C_Yq·qc̄/2 + C_Yr·rb/2)·q̃]·S·y_s",
  "variables": {
   "C_Yβ": "CYb [1/rad]"
  },
  "defaults": [
   {
    "name": "CYa, CYb",
    "value": "0.0, 0.0",
    "unit": "1/rad",
    "where": "L100, L116"
   }
  ],
  "assumptions": "Side force acts along y_s (= body y).",
  "notes": ""
 },
 {
  "id": "GZ-40",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: pitching moment",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "726-748",
  "code": "Cem = this->Cem0 + this->Cema * this->alpha;   // or stall branches with CemaStall\nCem += this->Cemb * this->beta;\nCem += Cem_ctrl_tot;\npm = ((Cem * dyn_pres) + (Cemp*(rr*span/2)*half_rho_vel) + (Cemq*(pr*mac/2)*half_rho_vel) + (Cemr*(yr*span/2)*half_rho_vel)) * (this->area * this->mac * body_y_axis);",
  "formula": "C_m = C_m0 + C_mα·α (stall: C_m0 ± (C_mα·α_s + C_mα,s·(α ∓ α_s))) + C_mβ·β + C_m,ctrl;  M_pitch = [C_m·q̄ + (C_mp·pb/2 + C_mq·qc̄/2 + C_mr·rb/2)·q̃]·S·c̄·y_b",
  "variables": {
   "C_m0": "Cem0 [-]",
   "C_mα": "Cema [1/rad]",
   "C_mq": "Cemq pitch damping [-]"
  },
  "defaults": [
   {
    "name": "Cem0, Cema, Cemb, CemaStall",
    "value": "0 each",
    "unit": "-",
    "where": "L93, L106, L122, L137"
   }
  ],
  "assumptions": "Pitch damping C_mq·(q c̄/2)·(½ρV)·S·c̄ is the only pitch rate damping in the Gazebo aero. advanced_plane: Cema −0.463966, Cemq −12.14014.",
  "notes": "Quote condensed from several code lines."
 },
 {
  "id": "GZ-41",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: roll moment",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "752-757",
  "code": "double Cell = this-> Cella * this->alpha + this->Cellb * this-> beta + Cell_ctrl_tot;\ngz::math::Vector3d rm = ((Cell * dyn_pres) + (Cellp*(rr*span/2)*half_rho_vel) + (Cellq*(pr*mac/2)*half_rho_vel) + (Cellr*(yr*span/2)*half_rho_vel)) * (this->area * span * body_x_axis);",
  "formula": "C_l = C_lα·α + C_lβ·β + C_l,ctrl;  M_roll = [C_l·q̄ + (C_lp·pb/2 + …)·q̃]·S·b·x_b",
  "variables": {
   "C_lp": "Cellp roll damping [-]"
  },
  "defaults": [
   {
    "name": "Cella, Cellb",
    "value": "0, 0",
    "unit": "-",
    "where": "L103, L119"
   }
  ],
  "assumptions": "advanced_plane: Cellp −0.487407, Cellb −0.03925.",
  "notes": "Quote condensed from several code lines."
 },
 {
  "id": "GZ-42",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "AdvancedLiftDrag: yaw moment and total wrench",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/advanced_lift_drag/AdvancedLiftDrag.cc",
  "lines": "761-783",
  "code": "double Cen = this->Cena * this->alpha + this->Cenb * this->beta + Cen_ctrl_tot;\ngz::math::Vector3d ym = ((Cen * dyn_pres) + ... ) * (this->area * span * body_z_axis);\ngz::math::Vector3d moment = pm+rm+ym;\ngz::math::Vector3d force = lift + drag + sideforce;\nconst auto totalTorque = torque + cpWorld.Cross(force);\nlink.AddWorldWrench(_ecm, force, totalTorque);",
  "formula": "C_n = C_nα·α + C_nβ·β + C_n,ctrl;  M_yaw = [C_n·q̄ + (C_np·pb/2 + C_nq·qc̄/2 + C_nr·rb/2)·q̃]·S·b·z_b;  M = M_pitch + M_roll + M_yaw;  F = L + D + Y;  τ = M + (R·r_cp) × F",
  "variables": {
   "C_nβ": "Cenb weathercock stability [1/rad]"
  },
  "defaults": [
   {
    "name": "Cena, Cenb",
    "value": "0, 0",
    "unit": "-",
    "where": "L109, L125"
   }
  ],
  "assumptions": "body_z_axis = −upward, so the yaw moment axis is z_b (down). advanced_plane: Cenb 0.100826, Cenr −0.089947.",
  "notes": "All coefficients are defined at the single cp of one plugin instance (advanced_plane uses one instance for the whole airframe, cp = (−0.12,0,0))."
 },
 {
  "id": "GZ-43",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "PX4 rc_cessna aerodynamic parameters (LiftDrag instances)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/rc_cessna/model.sdf",
  "lines": "763-898",
  "code": "<a0>0.05984281113</a0>\n<cla>4.752798721</cla>\n<cda>1.5</cda>\n<alpha_stall>0.3391428111</alpha_stall>\n<cla_stall>-3.85</cla_stall>\n<cda_stall>-0.9233984055</cda_stall>\n<cp>-0.05 0.45 0.05</cp>\n<area>0.6</area>",
  "formula": "Six LiftDrag plugins: left aileron servo_0 (cp −0.05,0.45,0.05; area 0.6; cda 1.5; k_δ −0.3), right aileron servo_1 (cp −0.05,−0.45,0.05; same), left flap (cp −0.05,0.15,0.05; cda 0.6417; k_δ −0.1), right flap (cp −0.05,−0.15,0.05), elevator servo_2 (a0 −0.2; cp −0.5,0,0; area 0.01; cda 0.6417; k_δ −4.0), rudder (upward 0 1 0; cp −0.5,0,0.05; area 0.1; k_δ 0.8)",
  "variables": {},
  "defaults": [
   {
    "name": "mass base_link",
    "value": "1.5",
    "unit": "kg",
    "where": "rc_cessna model.sdf L9"
   },
   {
    "name": "inertia ixx, iyy, izz",
    "value": "0.197563, 0.1458929, 0.1477",
    "unit": "kg·m^2",
    "where": "L11-16"
   },
   {
    "name": "cma, cma_stall",
    "value": "0, 0",
    "unit": "-",
    "where": "each plugin"
   },
   {
    "name": "air_density",
    "value": "1.2041",
    "unit": "kg/m^3",
    "where": "each plugin"
   },
   {
    "name": "servo joint damping",
    "value": "1.000",
    "unit": "N·m·s/rad",
    "where": "L603 etc."
   },
   {
    "name": "servo joint limits",
    "value": "±0.78",
    "unit": "rad",
    "where": "L600, 619, 639, 659, 679, 699"
   },
   {
    "name": "servo position controller p_gain",
    "value": "10.0",
    "unit": "-",
    "where": "L786, 811, 872, 897"
   },
   {
    "name": "pusher motor: motorConstant, momentConstant, maxRotVelocity",
    "value": "1.2e-05, 0.016, 1000",
    "unit": "N/(rad/s)^2, m, rad/s",
    "where": "L899-915 (turningDirection cw, rotorDragCoefficient 8.06428e-05)"
   },
   {
    "name": "rotor_puller link mass",
    "value": "0.005",
    "unit": "kg",
    "where": "L235"
   }
  ],
  "assumptions": "The wing is split into aileron/flap/tail sections; lift per the simple LiftDrag model (GZ-27 to GZ-32). Parameters follow the original Gazebo classic Cessna model.",
  "notes": "k_δ is control_joint_rad_to_cl (per rad)."
 },
 {
  "id": "GZ-44",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "PX4 advanced_plane aerodynamic parameters (AdvancedLiftDrag, AVL coefficients)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/advanced_plane/model.sdf",
  "lines": "619-705",
  "code": "<CL0>0.15188</CL0> <AR>6.5</AR> <eff>0.97</eff> <CLa>5.015</CLa> <CD0>0.029</CD0>\n<Cem0>0.075</Cem0> <Cema>-0.463966</Cema> <CYb>-0.258244</CYb>\n<cp>-0.12 0.0 0.0</cp> <area>0.34</area> <mac>0.22</mac>",
  "formula": "Coefficients: C_L0 0.15188, C_Lα 5.015, C_D0 0.029, AR 6.5, e 0.97, C_m0 0.075, C_mα −0.463966, C_Yβ −0.258244, C_lβ −0.03925, C_nβ 0.100826; rate derivatives: C_Yp 0.065861, C_lp −0.487407, C_np −0.040416, C_Dq 0.055166, C_Lq 7.971792, C_mq −12.14014, C_Yr 0.230299, C_lr 0.078165, C_nr −0.089947; control (per deg) servo_0: C_D −5.9e-5, C_Y 1.71e-4, C_L −0.01194, C_l −0.003331, C_m 0.001498, C_n −5.7e-5; servo_2: C_L 0.010696, C_m −0.025798, direction −1",
  "variables": {},
  "defaults": [
   {
    "name": "mass base_link",
    "value": "1",
    "unit": "kg",
    "where": "advanced_plane model.sdf L9"
   },
   {
    "name": "inertia ixx, iyy, izz",
    "value": "0.197563, 0.1458929, 0.1477",
    "unit": "kg·m^2",
    "where": "L11-16"
   },
   {
    "name": "num_ctrl_surfaces",
    "value": "4",
    "unit": "-",
    "where": "L660"
   },
   {
    "name": "alpha_stall, Cema_stall",
    "value": "0.3391428111, 0",
    "unit": "rad, 1/rad",
    "where": "L649, L652"
   },
   {
    "name": "pusher motor: motorConstant, momentConstant, maxRotVelocity",
    "value": "8.54858e-06, 0.01, 3500",
    "unit": "N/(rad/s)^2, m, rad/s",
    "where": "L706-723"
   },
   {
    "name": "ParachutePlugin cd_a, angular_damping",
    "value": "0.64, 1.0",
    "unit": "m^2, -",
    "where": "L724-728 (plugin source is in PX4-Autopilot, not read)"
   }
  ],
  "assumptions": "Mass 1 kg but inertia identical to rc_cessna (1.5 kg): inertia/mass mismatch noted, not a source bug. SDF carries tags a0, CLa_stall, CDa_stall that AdvancedLiftDrag::Load does not read.",
  "notes": "Also loads px4::AirSpeed plugin (libAirSpeedPlugin.so, L616-618), whose source was not read."
 },
 {
  "id": "GZ-45",
  "group": 6,
  "group_name": "wings/fixed-wing/VTOL aero",
  "quantity": "PX4 standard_vtol parameters (VTOL: 4 lift rotors + pusher, LiftDrag wings)",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/standard_vtol/model.sdf",
  "lines": "10-17, 682-842",
  "code": "<mass>5</mass> ... <ixx>0.477708333333</ixx> <iyy>0.341666666667</iyy> <izz>0.811041666667</izz>\n<motorConstant>2e-05</motorConstant> <momentConstant>0.06</momentConstant> <maxRotVelocity>1500</maxRotVelocity>",
  "formula": "Three LiftDrag instances: left elevon servo_0 (cp −0.05,0.3,0.05; area 0.50; cda 0.6417112299; k_δ −1.0), right elevon servo_1 (cp −0.05,−0.3,0.05), elevator servo_2 (a0 −0.2; cp −0.5,0,0; area 0.01; k_δ −12.0). Lift rotors: k_f 2e-5, k_m 0.06, ω_max 1500; pusher k_f 8.54858e-06, k_m 0.01, ω_max 3500",
  "variables": {},
  "defaults": [
   {
    "name": "mass base_link",
    "value": "5",
    "unit": "kg",
    "where": "L10"
   },
   {
    "name": "rotorVelocitySlowdownSim",
    "value": "20",
    "unit": "-",
    "where": "L769, 786, 804, 822, 840"
   },
   {
    "name": "rotorDragCoefficient (all rotors)",
    "value": "0.000106428",
    "unit": "-",
    "where": "L767 etc."
   },
   {
    "name": "lift rotor hover capability",
    "value": "4 × 2e-5 × 1500² = 180 N (hand computation) vs weight 5·9.8 = 49 N",
    "unit": "N",
    "where": "derived"
   },
   {
    "name": "duplicate tag",
    "value": "<control_joint_name>servo_2 listed twice",
    "unit": "-",
    "where": "L745-746"
   }
  ],
  "assumptions": "A VTOL = multirotor motor models on the same body as LiftDrag wings; no tilt, no transition model beyond the sum of forces. Wing control surfaces servo_0..2 have no p_gain in the position controller (default used).",
  "notes": "VTOL row; vehicle labeled fixed-wing as instructed. Quote condensed from several code lines."
 },
 {
  "id": "GZ-46",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind magnitude first-order rise",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/wind_effects/WindEffects.cc",
  "lines": "499-531",
  "code": "double kMag = period / this->characteristicTimeForWindRise;\ndouble seedXYMagnitude = std::sqrt(seed.X()*seed.X() + seed.Y()*seed.Y());\nthis->magnitudeMean = (1.0 - kMag) * this->magnitudeMean + kMag * seedXYMagnitude;\nmagnitude += this->magnitudeSinAmplitudePercent * this->magnitudeMean * std::sin(2 * GZ_PI * simTime / this->magnitudeSinPeriod);",
  "formula": "m̄[k+1] = (1 − Δt/τ_w)·m̄[k] + (Δt/τ_w)·|seed_xy|  (explicit Euler lag);  m = m̄ + A_m·m̄·sin(2π t / T_m); m ← Noise(m)",
  "variables": {
   "τ_w": "characteristicTimeForWindRise [s]",
   "seed_xy": "commanded wind velocity (WorldLinearVelocitySeed) [m/s]",
   "A_m": "amplitude_percent [fraction]",
   "T_m": "sin period [s]",
   "t": "simulation time [s]"
  },
  "defaults": [
   {
    "name": "characteristicTimeForWindRise",
    "value": "1.0",
    "unit": "s",
    "where": "WindEffects.cc L266"
   },
   {
    "name": "magnitudeSinAmplitudePercent",
    "value": "0.0",
    "unit": "-",
    "where": "L269"
   },
   {
    "name": "magnitudeSinPeriod",
    "value": "1.0",
    "unit": "s",
    "where": "L272"
   }
  ],
  "assumptions": "A deterministic sinusoidal modulation is the only gust model. Noise from sensors::NoiseFactory (Gaussian; applied with dt = 0 so there is no dynamic bias). No Dryden or von Karman turbulence.",
  "notes": "Explicit Euler form is unstable if Δt/τ_w > 2. Quote condensed from several code lines."
 },
 {
  "id": "GZ-47",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind direction first-order lag and sinusoid",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/wind_effects/WindEffects.cc",
  "lines": "535-560",
  "code": "double direction = GZ_RTOD(atan2(windLinVelSeed->Data().Y(), windLinVelSeed->Data().X()));\nthis->directionMean = (1.0 - kDir) * this->directionMean.value() + kDir * direction;\ndirection += this->orientationSinAmplitude * std::sin(2 * GZ_PI * simTime / this->orientationSinPeriod);\nwindVel.X(magnitude * std::cos(GZ_DTOR(direction)));\nwindVel.Y(magnitude * std::sin(GZ_DTOR(direction)));",
  "formula": "ψ_seed = atan2(seed_y, seed_x) [deg];  ψ̄ ← (1−k_d)ψ̄ + k_d·ψ_seed;  ψ = ψ̄ + A_ψ·sin(2πt/T_ψ) (+noise);  w_x = m·cosψ, w_y = m·sinψ",
  "variables": {
   "k_d": "Δt/τ_dir [-]",
   "A_ψ": "orientationSinAmplitude [deg]",
   "T_ψ": "orientationSinPeriod [s]"
  },
  "defaults": [
   {
    "name": "characteristicTimeForWindOrientationChange",
    "value": "1.0",
    "unit": "s",
    "where": "L275"
   },
   {
    "name": "orientationSinAmplitude",
    "value": "0.0",
    "unit": "deg",
    "where": "L278"
   },
   {
    "name": "orientationSinPeriod",
    "value": "1.0",
    "unit": "s",
    "where": "L281"
   }
  ],
  "assumptions": "Direction averaged as an angle, so it wraps incorrectly across ±180°. Wind frame: ENU world (x = east, y = north), comment L574 'Publish wind information (ENU)'.",
  "notes": "Angle +x toward +y, as in ENU math convention (not compass)."
 },
 {
  "id": "GZ-48",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Vertical wind",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/wind_effects/WindEffects.cc",
  "lines": "502, 523-525, 562-569",
  "code": "this->magnitudeMeanVertical = (1. - kMagVertical) * this->magnitudeMeanVertical + kMagVertical * windLinVelSeed->Data().Z();\nwindVel.Z(this->noiseVertical->Apply(this->magnitudeMeanVertical));",
  "formula": "w_z[k+1] = (1 − Δt/τ_v)·w_z[k] + (Δt/τ_v)·seed_z  (+ Gaussian noise if configured)",
  "variables": {
   "τ_v": "characteristicTimeForWindRiseVertical [s]"
  },
  "defaults": [
   {
    "name": "characteristicTimeForWindRiseVertical",
    "value": "1.0",
    "unit": "s",
    "where": "L294"
   }
  ],
  "assumptions": "Upward positive (ENU).",
  "notes": ""
 },
 {
  "id": "GZ-49",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind force on arbitrary links (drag toward wind)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/wind_effects/WindEffects.cc",
  "lines": "617-629",
  "code": "const math::Vector3d windForce = _inertial->Data().MassMatrix().Mass() * forceScalingFactor * (windVel->Data() - _linkVel->Data());\nlink.AddWorldForce(_ecm, windForce);",
  "formula": "F_wind = m · κ(x) · (v_wind − v_link),   applied at the link center of mass",
  "variables": {
   "m": "link mass [kg]",
   "κ(x)": "force_approximation_scaling_factor at link position [1/s]",
   "v_wind": "world wind velocity [m/s]",
   "v_link": "link world linear velocity [m/s]"
  },
  "defaults": [
   {
    "name": "force_approximation_scaling_factor",
    "value": "1.0 (constant)",
    "unit": "1/s",
    "where": "WindEffects.cc L455-456"
   }
  ],
  "assumptions": "Applied only to links whose WindMode component is true. Linear drag with time constant 1/κ, proportional to mass: wind is treated as a first-order relaxation of velocity toward the wind. It is a separate mechanism from the lift/drag and rotor-drag wind terms (GZ-20, GZ-27).",
  "notes": "Sign: force pushes the link toward the wind velocity, so wind velocity here is the velocity of the air."
 },
 {
  "id": "GZ-50",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind sign convention summary",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "620",
  "code": "Vector3 relativeWindVelocityWorld = bodyVelocityWorld - windSpeedWorld;",
  "formula": "Wind velocity w = velocity of the AIR in the ENU world frame; air-relative body velocity v_rel = v_body − w. Same convention in LiftDrag.cc L303-306 and AdvancedLiftDrag.cc L518-521 (… − windLinearVel->Data()), WindEffects.cc L626 (windVel − linkVel), and AirSpeedSensor.cc L174-175 (vel − R^T·wind)",
  "variables": {
   "w": "wind velocity [m/s]",
   "v_rel": "relative airflow seen by the body [m/s]"
  },
  "defaults": [],
  "assumptions": "World frame is ENU (z up). Contradiction to note: the AirSpeedSensor receives the wind SEED velocity (GZ-51), not the filtered wind.",
  "notes": "A +x wind blows toward +x (an eastward wind of 5 m/s gives w = (5,0,0))."
 },
 {
  "id": "GZ-51",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Wind seen by the airspeed sensor",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/air_speed/AirSpeed.cc",
  "lines": "290-299",
  "code": "math::Vector3d sensorRelativeVel = relativeVel(_entity, _ecm);\nit->second->SetVelocity(sensorRelativeVel);\nauto windVelSeed = _ecm.Component<components::WorldLinearVelocitySeed>(this->windEntity);\nmath::Vector3d windVel{windVelSeed->Data().X(), windVelSeed->Data().Y(), windVelSeed->Data().Z()};",
  "formula": "sensor wind = seed velocity (unfiltered, no sin, no noise); sensor body velocity v_b = R^T·v_world (Util.cc L136-137)",
  "variables": {
   "R": "sensor world orientation [-]"
  },
  "defaults": [],
  "assumptions": "The sensor ignores the WindEffects lag and sinusoid (GZ-46 to GZ-48); the physics uses the filtered wind, so the sensor and the aerodynamics see different winds during ramp-up.",
  "notes": "windVelSeed is dereferenced without a null check."
 },
 {
  "id": "GZ-52",
  "group": 7,
  "group_name": "wind & gusts",
  "quantity": "Gust and turbulence models",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/wind_effects/WindEffects.cc",
  "lines": "entire file scanned",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "No Dryden, von Karman, discrete gust or wind shear model. The only time variation is the first-order rise, the sine modulation (GZ-46, GZ-47) and Gaussian noise."
 },
 {
  "id": "GZ-53",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Plugin forces are accumulated as an external wrench",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/Link.cc",
  "lines": "541-569",
  "code": "auto posComWorldCoord = linkWorldPose.Rot().RotateVector(_offset);\nmath::Vector3d torqueWithOffset = _torque + posComWorldCoord.Cross(_force);\n... msgs::Set(wrench.Data().mutable_force(), _force);\nmsgs::Set(wrench.Data().mutable_torque(), torqueWithOffset);",
  "formula": "τ = τ_given + (R·r_off) × F;  wrenches from several plugins are summed in ExternalWorldWrenchCmd",
  "variables": {
   "r_off": "force application offset in the link frame (default 0) [m]",
   "F": "force in world frame [N]"
  },
  "defaults": [],
  "assumptions": "Torque is about the link ORIGIN, as documented on L557-558, which is also where Physics applies it (GZ-54).",
  "notes": "AddWorldForce is AddWorldWrench with zero torque."
 },
 {
  "id": "GZ-54",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Hand-over of the external wrench to the engine",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/physics/Physics.cc",
  "lines": "2736-2739",
  "code": "math::Vector3 force = msgs::Convert(_wrenchComp->Data().force());\nmath::Vector3 torque = msgs::Convert(_wrenchComp->Data().torque());\nlinkForceFeature->AddExternalForce(math::eigen3::convert(force));\nlinkForceFeature->AddExternalTorque(math::eigen3::convert(torque));",
  "formula": "F_ext, τ_ext added to the link for the next engine step, in world frame",
  "variables": {
   "F_ext": "external force [N]",
   "τ_ext": "external torque [N·m]"
  },
  "defaults": [],
  "assumptions": "gz-physics AddExternalForce with default position (link-frame zero) goes to AddLinkExternalForceInWorld(force, position) (include/gz/physics/detail/Link.hh L31-41, default arg not read).",
  "notes": ""
 },
 {
  "id": "GZ-55",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "DART external force and torque application",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "dartsim/src/LinkFeatures.cc",
  "lines": "30-46",
  "code": "bn->addExtForce(_force, _position, false, false);\n...\nbn->addExtTorque(_torque, false);",
  "formula": "DART body gets F_ext at point p (world-frame force, world-frame offset) and τ_ext (world frame); both are cleared after the step (World::step with resetCommand)",
  "variables": {
   "p": "application point (relative to link origin, world axes) [m]"
  },
  "defaults": [],
  "assumptions": "flags false,false = force NOT in local frame, offset NOT in local frame.",
  "notes": "bullet-featherstone uses addBaseForce/addBaseTorque(relPos × F) (bullet-featherstone/src/LinkFeatures.cc L44-54, L74-79)."
 },
 {
  "id": "GZ-56",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Gravity for links with fluid added mass (explicit m·g)",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "dartsim/src/SimulationFeatures.cc",
  "lines": "98-111",
  "code": "auto mass = info->inertial->MassMatrix().Mass();\nauto g = world->getGravity();\ninfo->link->addExtForce(mass * g, com, false, true);",
  "formula": "F = m·g added at the center of mass (offset in local frame) for links that have FluidAddedMass",
  "variables": {
   "m": "link mass [kg]",
   "g": "world gravity [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Only for links with added mass; other links get gravity inside DART.",
  "notes": "Not relevant to PX4 aircraft models."
 },
 {
  "id": "GZ-57",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "DART Newton dynamics (forward dynamics and velocity update)",
  "simulator": "Gazebo (DART)",
  "vehicle": "both",
  "repo": "dartsim/dart",
  "commit": "2558a3d",
  "file": "dart/simulation/World.cpp",
  "lines": "1484-1490",
  "code": "skel->computeForwardDynamics();\nskel->integrateVelocities(mTimeStep);\n...\nmConstraintSolver->solve();",
  "formula": "q̈ = ABA(q, q̇, τ, F_ext, g) (articulated body algorithm);  q̇ ← q̇ + q̈·Δt;  then contact/joint-limit constraint solve (LCP) corrects velocities",
  "variables": {
   "q̈": "joint accelerations [rad/s^2 or m/s^2]",
   "Δt": "mTimeStep [s]"
  },
  "defaults": [],
  "assumptions": "Read from the DART release-6.20 branch head, which may be newer than the DART version linked by a given gz-physics build. The ABA internals (Skeleton::computeForwardDynamics) were not read.",
  "notes": "External library; only the step structure was read."
 },
 {
  "id": "GZ-58",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "Linear acceleration published for the IMU",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/physics/Physics.cc",
  "lines": "4082-4094",
  "code": "Eigen::Vector3d bodyLinAccel = R_bs * frameData.linearAcceleration;\n... bodyLinAccelComp->SetData(math::eigen3::convert(bodyLinAccel), this->vec3Eql)",
  "formula": "a_body = R_bsᵀ-type rotation of the engine's world-frame link linear acceleration into the sensor/link frame (R_bs as defined in the function, not re-derived here)",
  "variables": {
   "a_body": "LinearAcceleration component [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Frame-data acceleration is the engine coordinate acceleration (free fall gives g); IMU adds the gravity term (GZ-81).",
  "notes": "R_bs definition (lines before 4082) not read in detail."
 },
 {
  "id": "GZ-59",
  "group": 8,
  "group_name": "rigid-body translation",
  "quantity": "ApplyLinkWrench system",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/apply_link_wrench/ApplyLinkWrench.cc",
  "lines": "93-110",
  "code": "if (_msg.wrench().has_force()) _force = msgs::Convert(_msg.wrench().force());\nif (_msg.wrench().has_torque()) _torque = msgs::Convert(_msg.wrench().torque());",
  "formula": "no physics: forwards user-supplied force, torque and force offset to Link::AddWorldWrench (GZ-53), instantaneous or persistent",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "Scanned the whole file for arithmetic; only message extraction and queueing found. Quote condensed from several code lines."
 },
 {
  "id": "GZ-60",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "DART pose update with exponential map (free body)",
  "simulator": "Gazebo (DART)",
  "vehicle": "both",
  "repo": "dartsim/dart",
  "commit": "2558a3d",
  "file": "dart/dynamics/FreeJoint.cpp",
  "lines": "702-712",
  "code": "nextRelativeTransform.linear() = rotation * math::expMapRot(spatialVelocity.head<3>() * _dt);\nnextRelativeTransform.translation() += rotation * spatialVelocity.tail<3>() * _dt;",
  "formula": "R[k+1] = R[k]·Exp(ω_body·Δt);  p[k+1] = p[k] + R[k]·v_body·Δt   (semi-implicit Euler: velocities are updated first, GZ-57)",
  "variables": {
   "R": "body orientation [-]",
   "ω_body": "angular velocity in the body frame [rad/s]",
   "v_body": "linear velocity in the body frame [m/s]",
   "Exp": "SO(3) exponential map [-]"
  },
  "defaults": [],
  "assumptions": "Rotation is kept on SO(3) exactly; translation uses the body velocity rotated by the OLD rotation. Euler's equation of rigid-body rotation (ω̇ = I⁻¹(τ − ω×Iω)) lives in DART's ABA (Skeleton.cpp, not read).",
  "notes": "Read from DART release-6.20; external library."
 },
 {
  "id": "GZ-61",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Angular acceleration and rate frames output",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/physics/Physics.cc",
  "lines": "4082-4108",
  "code": "Eigen::Vector3d bodyAngAccel = R_bs * frameData.angularAcceleration;",
  "formula": "body-frame angular acceleration = rotation of the engine's world-frame value; AngularVelocity component (L4078-4079) is the body-frame angular rate used by IMU and by AdvancedLiftDrag",
  "variables": {},
  "defaults": [],
  "assumptions": "Frame definition of R_bs not re-derived.",
  "notes": ""
 },
 {
  "id": "GZ-62",
  "group": 9,
  "group_name": "rigid-body rotation",
  "quantity": "Rigid-body rotational dynamics (Euler equations)",
  "simulator": "Gazebo (DART)",
  "vehicle": "both",
  "repo": "dartsim/dart",
  "commit": "2558a3d",
  "file": "dart/dynamics/Skeleton.cpp",
  "lines": "external library — not read",
  "code": "",
  "formula": "external library — not read",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "Only World::step and FreeJoint integration were read (GZ-57, GZ-60). Inertia enters from SDF <inertial> via SDFFeatures; rotor gyroscopic coupling appears only through the slowed-down rotor joint (GZ-16)."
 },
 {
  "id": "GZ-63",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Simulation step size handed to DART",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "dartsim/src/SimulationFeatures.cc",
  "lines": "83-96, 114",
  "code": "std::chrono::duration<double> dt = *dtDur;\nif (std::fabs(dt.count() - world->getTimeStep()) > tol) { world->setTimeStep(dt.count()); }\n...\nworld->step();",
  "formula": "Δt_DART = Δt_sim (set whenever it differs by more than 1e-6 s); one DART step per simulation step",
  "variables": {
   "Δt": "step [s]"
  },
  "defaults": [
   {
    "name": "tol",
    "value": "1e-6",
    "unit": "s",
    "where": "L85"
   }
  ],
  "assumptions": "gz-sim passes _info.dt through ForwardStep::Input (Physics.cc L3579-3587).",
  "notes": ""
 },
 {
  "id": "GZ-64",
  "group": 10,
  "group_name": "time integration",
  "quantity": "DART default step and integration scheme",
  "simulator": "Gazebo (DART)",
  "vehicle": "both",
  "repo": "dartsim/dart",
  "commit": "2558a3d",
  "file": "dart/simulation/World.cpp",
  "lines": "1266, 1484-1525",
  "code": "mTimeStep(0.001),\n...\nskel->integrateVelocities(mTimeStep); ... skel->integratePositions(mTimeStep); ... mTime += mTimeStep;",
  "formula": "semi-implicit (symplectic) Euler: v ← v + a·Δt, then x ← x + v_new·Δt; Δt default 0.001 s",
  "variables": {
   "Δt": "mTimeStep [s]"
  },
  "defaults": [
   {
    "name": "mTimeStep",
    "value": "0.001",
    "unit": "s",
    "where": "DART World.cpp L1266 (overridden by gz-sim)"
   },
   {
    "name": "integrateVelocity",
    "value": "vel + acc·dt",
    "unit": "-",
    "where": "dart/math/detail/ConfigurationSpace.hpp L229"
   }
  ],
  "assumptions": "The gz-sim step size comes from the world SDF <physics><max_step_size>, whose default lives in sdformat/gz-sim server config and was not read. PX4 world files were not part of the requested repos.",
  "notes": "Plugins (motor model, wind, battery) take Δt from _info.dt each step (MulticopterMotorModel.cc L475-476; WindEffects.cc L499; LinearBatteryPlugin.cc L636)."
 },
 {
  "id": "GZ-65",
  "group": 10,
  "group_name": "time integration",
  "quantity": "Bullet-featherstone fixed sub-step",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "bullet-featherstone/src/SimulationFeatures.cc",
  "lines": "589-594",
  "code": "worldInfo->world->stepSimulation(static_cast<btScalar>(stepSize), 1, static_cast<btScalar>(stepSize));",
  "formula": "Bullet stepSimulation(Δt, maxSubSteps = 1, fixedTimeStep = Δt)",
  "variables": {
   "Δt": "step size [s]"
  },
  "defaults": [],
  "assumptions": "Alternative engine; DART is the default in Gazebo with PX4 as far as the files read show (not verified: engine choice is in the gz-sim server config).",
  "notes": "The source warns varying dt may not work (L589-592)."
 },
 {
  "id": "GZ-66",
  "group": 11,
  "group_name": "ground contact",
  "quantity": "Friction and restitution passed to DART from SDF",
  "simulator": "Gazebo (gz-physics)",
  "vehicle": "both",
  "repo": "gazebosim/gz-physics",
  "commit": "d54f373",
  "file": "dartsim/src/SDFFeatures.cc",
  "lines": "946-956, 995-998",
  "code": "aspect->setFrictionCoeff(odeFriction->Get<double>(\"mu\"));\naspect->setSecondaryFrictionCoeff(odeFriction->Get<double>(\"mu2\"));\naspect->setRestitutionCoeff(surfaceBounce->Get<double>(\"restitution_coefficient\"));",
  "formula": "Coulomb friction coefficients μ, μ₂ per collision, restitution e per collision; DART combines two surfaces inside its constraint solver (not read)",
  "variables": {
   "μ, μ₂": "friction coefficients [-]",
   "e": "restitution [-]"
  },
  "defaults": [
   {
    "name": "mu, mu2, restitution",
    "value": "sdformat defaults, not read",
    "unit": "-",
    "where": "SDFFeatures.cc L953-998"
   },
   {
    "name": "slip1, slip2, fdir1",
    "value": "read as slip compliance and friction direction",
    "unit": "-",
    "where": "L958-981"
   }
  ],
  "assumptions": "The ODE-style surface elements are reused for DART. Contact stiffness / damping (kp, kd) are NOT read here.",
  "notes": "min_depth and max_vel in the PX4 SDF belong to <contact><ode>; their use in DART was not found in SDFFeatures lines read."
 },
 {
  "id": "GZ-67",
  "group": 11,
  "group_name": "ground contact",
  "quantity": "PX4 x500 and rc_cessna contact parameters",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/x500_base/model.sdf",
  "lines": "132-139",
  "code": "<ode>\n  <min_depth>0.001</min_depth>\n  <max_vel>0</max_vel>\n</ode>\n<friction><ode /></friction>",
  "formula": "x500_base collisions: min_depth 0.001 m, max_vel 0 m/s, friction ode element empty (defaults)",
  "variables": {},
  "defaults": [
   {
    "name": "x500 min_depth, max_vel",
    "value": "0.001, 0",
    "unit": "m, m/s",
    "where": "x500_base model.sdf L133-134 (same pattern L152 etc.)"
   },
   {
    "name": "rc_cessna base_link collisions min_depth, max_vel",
    "value": "0.01, 10",
    "unit": "m, m/s",
    "where": "rc_cessna model.sdf L29-30, L45-46"
   },
   {
    "name": "rc_cessna wheel mu",
    "value": "1.0",
    "unit": "-",
    "where": "rc_cessna L363, L406, L449"
   },
   {
    "name": "rc_cessna wheel joint friction",
    "value": "0.02",
    "unit": "N·m",
    "where": "rc_cessna L723, L740, L757"
   },
   {
    "name": "wheel radius, mass",
    "value": "0.03 m (center 0.025 m), 0.05 kg",
    "unit": "m, kg",
    "where": "rc_cessna L348, L391, L434; L331, L374, L417"
   }
  ],
  "assumptions": "No kp, kd or ground-effect parameters in either model. Ground effect on thrust: not found anywhere in the scanned files.",
  "notes": "A copy of the same tags on the ground plane lives in the PX4 world files (not in these repos)."
 },
 {
  "id": "GZ-68",
  "group": 11,
  "group_name": "ground contact",
  "quantity": "Contact constraint solving (impact, friction cone, penetration correction)",
  "simulator": "Gazebo (DART)",
  "vehicle": "both",
  "repo": "dartsim/dart",
  "commit": "2558a3d",
  "file": "dart/constraint/ConstraintSolver.cpp",
  "lines": "external library — not read",
  "code": "",
  "formula": "external library — not read",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "Only the call mConstraintSolver->solve() was read (World.cpp L1490). gz-physics also reads contacts back with world->getLastCollisionResult() (dartsim/src/SimulationFeatures.cc L292)."
 },
 {
  "id": "GZ-69",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Rotor allocation matrix (thrust and moments from rotor ω²)",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_control/Common.cc",
  "lines": "176-192",
  "code": "allocationMatrix(0, i) = sin(rotor.angle) * rotor.armLength * rotor.forceConstant;\nallocationMatrix(1, i) = -cos(rotor.angle) * rotor.armLength * rotor.forceConstant;\nallocationMatrix(2, i) = -rotor.direction * rotor.forceConstant * rotor.momentConstant;\nallocationMatrix(3, i) = rotor.forceConstant;",
  "formula": "[τ_x, τ_y, τ_z, T]ᵀ = A·[ω_1², …, ω_n²]ᵀ with column i = [k_f·ℓ·sinθ_i, −k_f·ℓ·cosθ_i, −d_i·k_f·k_m, k_f]ᵀ  (Lee et al. 2010 Eq.1)",
  "variables": {
   "ℓ": "arm length [m]",
   "θ_i": "angle of rotor i from body +x [rad]",
   "d_i": "direction (+1 ccw, −1 cw) [-]",
   "k_f": "forceConstant times the projection of the rotor axis on body z [N/(rad/s)^2]",
   "k_m": "momentConstant [m]"
  },
  "defaults": [
   {
    "name": "rank threshold",
    "value": "1e-9 (rank must be 4)",
    "unit": "-",
    "where": "Common.cc L198-206"
   }
  ],
  "assumptions": "Rotor plane is the body XY plane (L107-108); forceConstant is scaled by the cosine between the rotor axis and body z (L132-143).",
  "notes": "Matches the sign convention of the motor model drag torque (GZ-11)."
 },
 {
  "id": "GZ-70",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Pseudo-inverse mapping from angular acceleration and thrust to ω²",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_control/LeeVelocityController.cc",
  "lines": "62-112",
  "code": "this->angularAccToRotorVelocities = aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;\n...\n_rotorVelocities = this->angularAccToRotorVelocities * angularAccelerationThrust;\n_rotorVelocities = _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));\n_rotorVelocities = _rotorVelocities.cwiseSqrt();",
  "formula": "ω² = Aᵀ(AAᵀ)⁻¹·diag(I, 1)·[α_des; T];  ω_i = √max(ω_i², 0);  T = −m·(a_des·b₃)",
  "variables": {
   "A": "allocation matrix [-]",
   "I": "vehicle inertia matrix [kg·m^2]",
   "α_des": "desired angular acceleration [rad/s^2]",
   "T": "collective thrust [N]",
   "a_des": "desired acceleration including gravity [m/s^2]",
   "b₃": "body z axis in world frame [-]"
  },
  "defaults": [],
  "assumptions": "Needs the total vehicle mass and inertia summed from all links (MulticopterVelocityControl.cc L96-99). Gravity vector is read from the world (L132-140).",
  "notes": "Used by Gazebo's MulticopterVelocityControl plugin, NOT by PX4 SITL (PX4 does its own mixing); x500 SDF does not load it."
 },
 {
  "id": "GZ-71",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Lee velocity controller: desired acceleration",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_control/LeeVelocityController.cc",
  "lines": "118-129",
  "code": "Eigen::Vector3d velocityError = _frameData.linearVelocityWorld - _frameData.pose.linear() * _cmdVel.linear;\nEigen::Vector3d accelCommand = velocityError.cwiseProduct(this->controllerParameters.velocityGain) / this->vehicleParameters.mass;\naccelCommand = accelCommand.cwiseAbs().cwiseMin(maxLinearAcceleration).cwiseProduct(accelCommand.cwiseSign());\nreturn accelCommand + this->vehicleParameters.gravity;",
  "formula": "e_v = v − R·v_cmd;  a = clip(K_v∘e_v / m, ±a_max) + g",
  "variables": {
   "K_v": "velocityGain (3-vector) [N·s/m]",
   "a_max": "maxLinearAcceleration [m/s^2]",
   "g": "gravity vector (z negative) [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Velocity command is given in the body frame (rotated by R). The returned acceleration includes +g, and thrust is later computed as −m·(a·b₃) (L99), so b₃ is the body axis along which thrust is NEGATIVE of the acceleration; this sign convention was read, not derived.",
  "notes": "Quote condensed from several code lines."
 },
 {
  "id": "GZ-72",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Lee attitude error and angular acceleration command",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_control/LeeVelocityController.cc",
  "lines": "173-203",
  "code": "Eigen::Matrix3d angleErrorMatrix = 0.5 * (rotDes.transpose() * rot - rot.transpose() * rotDes);\nEigen::Vector3d angularRateError = _frameData.angularVelocityBody - rot.transpose() * rotDes * angularRateDes;\nreturn -1 * angleError.cwiseProduct(this->normalizedAttitudeGain) - angularRateError.cwiseProduct(this->normalizedAngularRateGain);",
  "formula": "e_R = ½(R_dᵀR − RᵀR_d)^∨;  e_ω = ω − RᵀR_d ω_des;  α_cmd = −K_R I⁻¹ ∘ e_R − K_ω I⁻¹ ∘ e_ω  (gyroscopic terms ω×Iω dropped on purpose, L187-201)",
  "variables": {
   "R, R_d": "current and desired rotation [-]",
   "K_R, K_ω": "attitudeGain, angularRateGain [N·m/rad, N·m·s/rad]",
   "I": "inertia [kg·m^2]"
  },
  "defaults": [],
  "assumptions": "Desired body axes: b₃d = −a/|a|, b₁d chosen from current x axis (or y/z if near parallel, tol 1e-3), yaw rate only from the command. The code follows RotorS rather than the paper for e_ω (comment L180-186).",
  "notes": ""
 },
 {
  "id": "GZ-73",
  "group": 12,
  "group_name": "control allocation/mixing",
  "quantity": "Measurement noise injected into the controller feedback",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_control/Common.cc",
  "lines": "264-284",
  "code": "if (_stdDev(i) > 0) { _val(i) += math::Rand::DblNormal(_mean(i), _stdDev(i)); }",
  "formula": "v_meas = v + N(μ, σ²) per axis, added to world linear velocity and body angular velocity",
  "variables": {
   "μ, σ": "noise mean and std-dev [m/s or rad/s]"
  },
  "defaults": [],
  "assumptions": "White Gaussian, no bias process.",
  "notes": ""
 },
 {
  "id": "GZ-74",
  "group": 13,
  "group_name": "battery",
  "quantity": "Battery discharge current and low-pass filter",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/battery_plugin/LinearBatteryPlugin.cc",
  "lines": "746-765",
  "code": "double k = dt / this->dataPtr->tau;\nthis->dataPtr->iraw = totalpower / _battery->Voltage();\nif (this->dataPtr->startCharging && this->dataPtr->StateOfCharge() < 0.9) this->dataPtr->iraw -= iCharge;\nthis->dataPtr->ismooth = this->dataPtr->ismooth + k * (this->dataPtr->iraw - this->dataPtr->ismooth);",
  "formula": "i_raw = P_total / V;  i_smooth ← i_smooth + (Δt/τ)(i_raw − i_smooth)",
  "variables": {
   "P_total": "sum of power loads of all consumers [W]",
   "V": "battery voltage of the previous step [V]",
   "τ": "smooth_current_tau [s]"
  },
  "defaults": [
   {
    "name": "tau",
    "value": "1.0",
    "unit": "s",
    "where": "LinearBatteryPlugin.cc L118 (clamped to ≥ Δt, L638-646)"
   }
  ],
  "assumptions": "Constant-power load model (current rises as voltage falls). Power loads include the initial load plus consumer entities (L534-544; the Thruster adds one, Thruster.cc L688-692).",
  "notes": "Motor model does NOT register a battery consumer: multirotor motor power is not tied to thrust in these files."
 },
 {
  "id": "GZ-75",
  "group": 13,
  "group_name": "battery",
  "quantity": "Battery charge integration and open-circuit voltage",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/battery_plugin/LinearBatteryPlugin.cc",
  "lines": "778-785",
  "code": "this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) / 3600.0);\ndouble voltage = this->dataPtr->e0 + this->dataPtr->e1 * (1 - this->dataPtr->q / this->dataPtr->c) - this->dataPtr->r * this->dataPtr->ismooth;",
  "formula": "Q ← Q − i_smooth·Δt/3600 [Ah];  V = E₀ + E₁·(1 − Q/C) − R·i_smooth   (header comment: E(t) = e0 + e1·Q(t)/c, L104)",
  "variables": {
   "Q": "remaining charge [Ah]",
   "C": "capacity [Ah]",
   "E₀": "open_circuit_voltage_constant_coef [V]",
   "E₁": "open_circuit_voltage_linear_coef [V]",
   "R": "resistance [Ω]"
  },
  "defaults": [
   {
    "name": "e0, e1, c, r, q0",
    "value": "0.0 (SDF parameters; capacity must be > 0)",
    "unit": "V, V, Ah, Ω, Ah",
    "where": "LinearBatteryPlugin.cc L105-115, L241-248"
   },
   {
    "name": "initial_charge",
    "value": "= capacity",
    "unit": "Ah",
    "where": "L248-251"
   }
  ],
  "assumptions": "Voltage rises with CHARGE USED (1 − Q/C) in the code, while the header comment writes e0 + e1·Q/c, so e1 must be negative for a battery whose voltage drops as it discharges. Ideal linear model, no temperature or rate-capacity effect.",
  "notes": "Contradiction between comment (L104) and code (L783-785) noted."
 },
 {
  "id": "GZ-76",
  "group": 13,
  "group_name": "battery",
  "quantity": "State of charge and charging current",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/battery_plugin/LinearBatteryPlugin.cc",
  "lines": "758-762, 787-796",
  "code": "auto iCharge = this->dataPtr->c / this->dataPtr->tCharge;\nif (this->dataPtr->fixIssue225) this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;\nelse { ... this->dataPtr->soc = this->dataPtr->soc - isum / this->dataPtr->c; }",
  "formula": "SoC = Q/C (fix_issue_225 = true);  legacy: SoC ← SoC − (Σ_{last 100 steps} i·Δt/3600)/C;  charging current I_c = C / t_charge, applied while SoC < 0.9",
  "variables": {
   "t_charge": "charging_time [h]",
   "SoC": "state of charge [0..1]"
  },
  "defaults": [
   {
    "name": "soc initial",
    "value": "1.0",
    "unit": "-",
    "where": "L130"
   }
  ],
  "assumptions": "The legacy SoC formula integrates the last 100 samples each step (a known bug, tracked as issue 225; selectable with fix_issue_225).",
  "notes": "Thruster stops when any battery SoC ≤ 0 (Thruster.cc L602-621)."
 },
 {
  "id": "GZ-77",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gaussian noise: white part",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/GaussianNoiseModel.cc",
  "lines": "108-113, 139",
  "code": "double whiteNoise = this->dataPtr->stdDev > 0.0 ? math::Rand::DblNormal(this->dataPtr->mean, this->dataPtr->stdDev) : this->dataPtr->mean;\ndouble output = _in + this->dataPtr->bias + whiteNoise;",
  "formula": "y = x + b + N(μ, σ²)",
  "variables": {
   "x": "true value",
   "b": "bias (static + dynamic)",
   "μ": "noise mean",
   "σ": "noise stddev"
  },
  "defaults": [
   {
    "name": "mean, stdDev",
    "value": "0.0, 0.0",
    "unit": "sensor units",
    "where": "GaussianNoiseModel.cc L36, L40"
   }
  ],
  "assumptions": "Applied independently at each sample, uncorrelated in time and across axes. No scale-factor or misalignment error exists in the model.",
  "notes": "The noise is applied per sample with no dependence on update rate (variance does not scale with 1/Δt)."
 },
 {
  "id": "GZ-78",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gaussian noise: constant bias at start-up",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/GaussianNoiseModel.cc",
  "lines": "82-96",
  "code": "if (biasStdDev > 0.0) this->dataPtr->bias = math::Rand::DblNormal(biasMean, biasStdDev);\nelse this->dataPtr->bias = biasMean;\nif (math::Rand::DblUniform() < 0.5) this->dataPtr->bias = -this->dataPtr->bias;",
  "formula": "b₀ ~ N(μ_b, σ_b²) (or μ_b), then sign flipped with probability ½",
  "variables": {
   "μ_b, σ_b": "bias_mean, bias_stddev [sensor units]"
  },
  "defaults": [
   {
    "name": "bias",
    "value": "0.0",
    "unit": "sensor units",
    "where": "L43"
   }
  ],
  "assumptions": "Constant for the run unless dynamic bias is enabled (GZ-79). A random sign flip is applied even for a deterministic bias_mean.",
  "notes": ""
 },
 {
  "id": "GZ-79",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gaussian noise: dynamic bias (first-order Gauss-Markov, not a pure random walk)",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/GaussianNoiseModel.cc",
  "lines": "125-137",
  "code": "double sigma_b_d = sqrt(-sigma_b * sigma_b * tau / 2 * expm1(-2 * _dt / tau));\ndouble phi_d = exp(-_dt / tau);\nthis->dataPtr->bias = phi_d * this->dataPtr->bias + math::Rand::DblNormal(0, sigma_b_d);",
  "formula": "b[k+1] = e^{−Δt/τ}·b[k] + N(0, σ_d²),  σ_d² = −σ_b²·(τ/2)·expm1(−2Δt/τ) = σ_b²·(τ/2)·(1 − e^{−2Δt/τ})",
  "variables": {
   "σ_b": "dynamic_bias_stddev [sensor units]",
   "τ": "dynamic_bias_correlation_time [s]",
   "Δt": "time since the last sample [s]"
  },
  "defaults": [
   {
    "name": "dynamicBiasStdDev, dynamicBiasCorrTime",
    "value": "0.0, 0.0 (disabled)",
    "unit": "-",
    "where": "L47, L51"
   }
  ],
  "assumptions": "Active only if σ_b > 0, τ > 0 and Δt > 0. Δt is nonzero only when the sensor passes dt to Apply: the IMU does (GZ-82), the others call Apply(x) with the default dt = 0 (Noise.hh L94), so their dynamic bias never evolves. The stationary variance of b is σ_b² (rotors/Kalibr model).",
  "notes": "Bias is added into the output at L139; code follows RotorS gazebo_imu_plugin (L116-117)."
 },
 {
  "id": "GZ-80",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Gaussian noise: quantization",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/GaussianNoiseModel.cc",
  "lines": "141-149",
  "code": "output = std::round(output / this->dataPtr->precision) * this->dataPtr->precision;",
  "formula": "y_q = round(y / p)·p",
  "variables": {
   "p": "precision [sensor units]"
  },
  "defaults": [
   {
    "name": "precision",
    "value": "0.0 (disabled)",
    "unit": "sensor units",
    "where": "L55; quantized if |p| > 1e-6, L103-104"
   }
  ],
  "assumptions": "Applied after bias and white noise.",
  "notes": ""
 },
 {
  "id": "GZ-81",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU gravity term (specific force)",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/ImuSensor.cc",
  "lines": "228-232",
  "code": "this->dataPtr->linearAcc -= this->dataPtr->worldPose.Rot().Inverse().RotateVector(this->dataPtr->gravity);",
  "formula": "f_b = a_b − Rᵀ·g   (accelerometer reads specific force; at rest a_b = 0 so f_b = −Rᵀ·g, +9.8 m/s² along body up for g = (0,0,−9.8))",
  "variables": {
   "a_b": "body-frame linear acceleration from the engine [m/s^2]",
   "R": "sensor world orientation [-]",
   "g": "gravity vector [m/s^2]"
  },
  "defaults": [],
  "assumptions": "Gravity is taken from the world and is assumed constant (gz-sim Imu.cc L219-220, updated every step L322-327). The code comment (L229) 'Skip if gravity is not enabled?' means the gravity term is added even if the link has gravity disabled.",
  "notes": "World gravity sign assumed (0,0,−9.8)-like, not verified in this stack."
 },
 {
  "id": "GZ-82",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU noise application and covariance",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/ImuSensor.cc",
  "lines": "215-247, 263-290",
  "code": "auto delay = std::chrono::duration_cast<std::chrono::duration<float>>(_now - this->dataPtr->prevStep);\ndt = delay.count();\n...\nvalue = this->dataPtr->noises[noiseType]->Apply(value, dt);\n... return static_cast<float>(gaussian->StdDev() * gaussian->StdDev());",
  "formula": "each of the 3 accelerometer and 3 gyro channels gets its own Gaussian noise object with Δt = time since the last IMU sample (0 on the first sample); reported covariance = σ²",
  "variables": {
   "Δt": "IMU sample interval [s]"
  },
  "defaults": [],
  "assumptions": "Independent noise per axis. Orientation covariance reported 0 (L288-290). No temperature, g-sensitivity, or scale-factor error.",
  "notes": "Δt is computed in float (L219-221)."
 },
 {
  "id": "GZ-83",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU orientation output",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/ImuSensor.cc",
  "lines": "293-302",
  "code": "this->dataPtr->orientation = this->dataPtr->orientationReference.Inverse() * this->dataPtr->worldPose.Rot();",
  "formula": "q_out = q_ref⁻¹ ⊗ q_world,  published without noise",
  "variables": {
   "q_ref": "orientation reference (ENU, NED, NWU or custom), set from SDF or world heading [-]"
  },
  "defaults": [
   {
    "name": "orientationEnabled",
    "value": "true",
    "unit": "-",
    "where": "L64"
   }
  ],
  "assumptions": "Orientation is exact (no noise). Frame transform table at L374-405 (ENU/NED/NWU).",
  "notes": ""
 },
 {
  "id": "GZ-84",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "IMU angular velocity input",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/imu/Imu.cc",
  "lines": "333-337",
  "code": "it->second->SetAngularVelocity(_angularVel->Data());\nit->second->SetLinearAcceleration(_linearAccel->Data());",
  "formula": "gyro = body-frame angular velocity of the sensor link (plus noise, GZ-82); accel = body-frame engine acceleration (before the gravity correction, GZ-81)",
  "variables": {},
  "defaults": [],
  "assumptions": "No lever-arm, vibration or sensor-offset correction.",
  "notes": "The sensor entity needs AngularVelocity and LinearAcceleration components, which the physics system fills (GZ-58, GZ-61)."
 },
 {
  "id": "GZ-85",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Altimeter",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/AltimeterSensor.cc",
  "lines": "203-218, 160-177",
  "code": "this->dataPtr->verticalPosition = _pos - this->dataPtr->verticalReference;\n... verticalPosition = Noise->Apply(verticalPosition); verticalVelocity = Noise->Apply(verticalVelocity);",
  "formula": "z_alt = z_world − z_ref (z_ref = world z at sensor creation, gz-sim Altimeter.cc L210-212);  v_z = world linear velocity z (gz-sim L266);  each noise-corrupted independently",
  "variables": {
   "z_ref": "vertical reference [m]"
  },
  "defaults": [],
  "assumptions": "Altitude relative to the spawn height, not above ground.",
  "notes": "No atmosphere or pressure involved. Quote condensed from several code lines."
 },
 {
  "id": "GZ-86",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Air-pressure sensor noise and variance field",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/AirPressureSensor.cc",
  "lines": "182-200",
  "code": "this->dataPtr->pressure = this->dataPtr->noises[AIR_PRESSURE_NOISE_PASCALS]->Apply(this->dataPtr->pressure);\n... msg.set_variance(sqrt(gaussian->StdDev()));",
  "formula": "p_meas = p(h) + noise;  message variance field = √σ (source writes sqrt of the standard deviation)",
  "variables": {
   "σ": "noise stddev [Pa]"
  },
  "defaults": [
   {
    "name": "baro noise x500",
    "value": "mean 0, stddev 3 (BMP390)",
    "unit": "Pa",
    "where": "x500_base model.sdf L224-228"
   }
  ],
  "assumptions": "The variance field is √σ rather than σ², an apparent bug (variance should be σ²). Noted, not changed.",
  "notes": "Also the pressure includes no wind or dynamic pressure effect."
 },
 {
  "id": "GZ-87",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Airspeed sensor differential pressure",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "fixed-wing",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/AirSpeedSensor.cc",
  "lines": "172-188",
  "code": "math::Vector3d air_vel_in_body_ = this->dataPtr->vel - veh_q_world_to_body.RotateVectorReverse(this->dataPtr->windVel);\nfloat diff_pressure = math::sgn(air_vel_in_body_.X()) * 0.005f * air_density * air_vel_in_body_.X() * air_vel_in_body_.X();\n... msg.set_diff_pressure(diff_pressure * 100.0f);",
  "formula": "v_air,b = v_b − Rᵀ·w;  Δp[hPa] = sgn(v_air,b,x)·0.005·ρ·v_air,b,x²   (= ½ρV² in Pa divided by 100);  published Δp[Pa] = (Δp + noise)·100",
  "variables": {
   "v_b": "sensor-frame velocity [m/s]",
   "w": "world wind (seed) [m/s]",
   "ρ": "local density from the ISA model (GZ-04) [kg/m^3]"
  },
  "defaults": [
   {
    "name": "airspeed noise x500/rc_cessna",
    "value": "stddev 0.01 (rc_cessna airspeed sensor L220-221), x500 has no airspeed sensor",
    "unit": "hPa (noise applied before ×100)",
    "where": "rc_cessna model.sdf L214-221"
   }
  ],
  "assumptions": "Only the body-x component enters (a pitot tube). Noise is added in hPa units. Fixed-wing only: x500 has no airspeed sensor.",
  "notes": "The conversion factor 0.005 = 0.5 × 0.01 (hPa/Pa)."
 },
 {
  "id": "GZ-88",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Magnetic field model (declination, inclination, strength)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/magnetometer/Magnetometer.cc",
  "lines": "467-505",
  "code": "float H = cosf(inclination_rad);\nH *= this->useUnitsGauss ? strength_ga : strength_tesla;\nfloat Z_ned = tanf(inclination_rad) * H;\nfloat X_ned = H * cosf(declination_rad);\nfloat Y_ned = H * sinf(declination_rad);\n// ENU: X = Y_ned; Y = X_ned; Z = -1.0 * Z_ned;",
  "formula": "B_total = strength; H = B·cos(inc); (N, E, D) = (H·cos(dec), H·sin(dec), H·tan(inc)); ENU world field = (E, N, −D)",
  "variables": {
   "inc, dec": "inclination and declination from lookup functions get_mag_inclination, get_mag_declination [rad]",
   "B": "total field = 1e-4 × (0.01 × get_mag_strength) T"
  },
  "defaults": [
   {
    "name": "strength unit conversion",
    "value": "10^5 nT = 10^-2 centi-gauss; tesla = 1e-4 × gauss",
    "unit": "-",
    "where": "L478-484"
   }
  ],
  "assumptions": "The lookup tables (get_mag_*) are a separate header (geo_mag_declination style), not read here; this appears to follow the PX4 geo-mag lookup. Field is the world field at the sensor latitude and longitude, constant in time and with altitude.",
  "notes": "Earth-frame convention selectable (useEarthFrameNED)."
 },
 {
  "id": "GZ-89",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "Magnetometer body-frame field and noise",
  "simulator": "Gazebo (gz-sensors)",
  "vehicle": "both",
  "repo": "gazebosim/gz-sensors",
  "commit": "8ddf4c1",
  "file": "src/MagnetometerSensor.cc",
  "lines": "159-162, 170-195",
  "code": "this->dataPtr->localField = this->dataPtr->worldPose.Rot().Inverse().RotateVector(this->dataPtr->worldField);\n// Apply magnetometer noise after converting to body frame",
  "formula": "B_b = Rᵀ·B_world, then independent Gaussian noise on each axis (X, Y, Z)",
  "variables": {
   "R": "sensor world orientation [-]"
  },
  "defaults": [
   {
    "name": "x500 noise stddev per axis",
    "value": "3e-7 (3 mgauss RMS, IIS2MDC)",
    "unit": "T",
    "where": "x500_base model.sdf L237-253"
   }
  ],
  "assumptions": "No hard-iron, soft-iron or temperature errors.",
  "notes": ""
 },
 {
  "id": "GZ-90",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "NavSat (GPS) position and velocity",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/navsat/NavSat.cc",
  "lines": "255-269",
  "code": "it->second->SetLatitude(GZ_DTOR(latLonEle.value().X()));\nit->second->SetLongitude(GZ_DTOR(latLonEle.value().Y()));\nit->second->SetAltitude(latLonEle.value().Z());\nit->second->SetVelocity(_worldLinearVel->Data());",
  "formula": "lat, lon, alt from the world spherical coordinates conversion of the sensor position; velocity = world ENU linear velocity (m/s); gz-sensors adds Gaussian noise to each (NavSatSensor.cc L175-197: latitude/longitude noise applied in degrees, altitude in metres, each velocity component)",
  "variables": {},
  "defaults": [
   {
    "name": "x500 navsat noise",
    "value": "none (no <noise> in the sensor)",
    "unit": "-",
    "where": "x500_base model.sdf L309-313"
   },
   {
    "name": "update rate",
    "value": "30",
    "unit": "Hz",
    "where": "x500_base L312"
   }
  ],
  "assumptions": "No GPS latency, no multipath, no HDOP; position error only from SDF noise elements. Spherical conversion of local position to latitude and longitude is done by sphericalCoordinates (not read).",
  "notes": "Horizontal noise sigma is in degrees for lat/lon (L179-182), not metres."
 },
 {
  "id": "GZ-91",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "PX4 x500 IMU noise parameters",
  "simulator": "Gazebo",
  "vehicle": "multirotor",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/x500_base/model.sdf",
  "lines": "257-308",
  "code": "<update_rate>250</update_rate>\n<angular_velocity><x><noise type=\"gaussian\"><mean>0.0</mean><stddev>0.0008726646</stddev>\n<linear_acceleration> x,y stddev 0.00637, z stddev 0.00686",
  "formula": "gyro σ = 0.0008726646 rad/s (0.05 deg/s, IIM42653); accel σ_x = σ_y = 0.00637 m/s² (0.65 mg), σ_z = 0.00686 m/s² (0.70 mg); mean 0; no bias process",
  "variables": {},
  "defaults": [
   {
    "name": "IMU rate",
    "value": "250",
    "unit": "Hz",
    "where": "L260"
   },
   {
    "name": "baro rate",
    "value": "50",
    "unit": "Hz",
    "where": "L221"
   },
   {
    "name": "magnetometer rate",
    "value": "100",
    "unit": "Hz",
    "where": "L235"
   }
  ],
  "assumptions": "Noise is white per sample; a density-based (per √Hz) specification was converted to per-sample RMS without regard to the rate.",
  "notes": "x500 does not use dynamic bias; rc_cessna does (GZ-92). Quote condensed from several code lines."
 },
 {
  "id": "GZ-92",
  "group": 14,
  "group_name": "sensors & noise",
  "quantity": "PX4 rc_cessna IMU noise with dynamic bias",
  "simulator": "Gazebo",
  "vehicle": "fixed-wing",
  "repo": "PX4/PX4-gazebo-models",
  "commit": "e5997f4",
  "file": "models/rc_cessna/model.sdf",
  "lines": "90-136",
  "code": "<stddev>0.0003394</stddev>\n<dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>\n<dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>\n...\n<stddev>0.004</stddev> <dynamic_bias_stddev>0.006</dynamic_bias_stddev> <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>",
  "formula": "gyro: σ = 3.394e-4 rad/s, σ_b = 3.8785e-5 rad/s, τ = 1000 s;  accelerometer: σ = 0.004 m/s², σ_b = 0.006 m/s², τ = 300 s (mean 0)",
  "variables": {},
  "defaults": [
   {
    "name": "IMU rate",
    "value": "250",
    "unit": "Hz",
    "where": "rc_cessna L86"
   },
   {
    "name": "baro noise stddev",
    "value": "0.01",
    "unit": "Pa",
    "where": "rc_cessna L155 (value read from the sensor at L149-155, noise element under air_pressure)"
   },
   {
    "name": "magnetometer stddev",
    "value": "3e-7",
    "unit": "T",
    "where": "rc_cessna L170-180"
   }
  ],
  "assumptions": "Uses the Gauss-Markov dynamic bias of GZ-79. Baro stddev attribution to the pressure noise element is inferred from the SDF layout (L149-155), not from a separate read.",
  "notes": "Parameters follow the Kalibr / RotorS IMU model (units rad/s and m/s²)."
 },
 {
  "id": "GZ-93",
  "group": 15,
  "group_name": "estimators",
  "quantity": "State estimators (EKF, complementary filter, etc.)",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/** scanned list",
  "lines": "n/a",
  "code": "",
  "formula": "not found",
  "variables": {},
  "defaults": [],
  "assumptions": "",
  "notes": "Gazebo has no state estimator. The estimator (EKF2) lives in PX4. The only filters in the files read are the motor lag (GZ-14), wind lag (GZ-46) and battery current lag (GZ-74). ImuSensor / odometry_publisher were not found to contain filtering (OdometryPublisher.cc was listed but not read)."
 },
 {
  "id": "GZ-94",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Motor command (rad/s) to thrust, end to end",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/multicopter_motor_model/MulticopterMotorModel.cc",
  "lines": "527-532, 565-573, 675-681",
  "code": "this->refMotorInput = std::min(msg->velocity(this->actuatorNumber), this->maxRotVelocity);\nrefMotorRotVel = this->rotorVelocityFilter->UpdateFilter(this->refMotorInput, this->samplingTime);\nthrust = d·sgn(ω_real)·ω_real²·k_f",
  "formula": "ω_cmd (message, rad/s) → min(ω_cmd, ω_max) → first-order lag (τ_up/τ_down) → joint velocity d·ω/s → measured ω_real = ω_joint·s → T = k_f·ω_real²",
  "variables": {
   "ω_cmd": "actuators message velocity [rad/s]"
  },
  "defaults": [],
  "assumptions": "The command is a rotor speed, not a normalized throttle. Normalization (PX4 PWM or 0..1 → rad/s) is done by the PX4 gz bridge, not in these repos. Thrust is therefore ≈ k_f·ω_cmd² in steady state; static thrust at ω_max = k_f·ω_max².",
  "notes": "The x500 plugin subscribes to command/motor_speed; PX4 normalization is not in the repos read. Quote condensed from several code lines."
 },
 {
  "id": "GZ-95",
  "group": 16,
  "group_name": "command-to-thrust models",
  "quantity": "Thruster: thrust command to propeller speed",
  "simulator": "Gazebo",
  "vehicle": "both",
  "repo": "gazebosim/gz-sim",
  "commit": "9e3b78f",
  "file": "src/systems/thruster/Thruster.cc",
  "lines": "562-582, 520-525",
  "code": "auto propAngularVelocity = sqrt(abs(_thrust / (this->fluidDensity * this->thrustCoefficient * pow(this->propellerDiameter, 4))));\npropAngularVelocity *= (_thrust * this->thrustCoefficient > 0) ? 1: -1;",
  "formula": "n = sgn(T·C_T)·√(|T| / (ρ·C_T·D⁴));  command thrust clamped to [cmdMin, cmdMax] first",
  "variables": {
   "T": "commanded thrust [N]"
  },
  "defaults": [
   {
    "name": "cmdMax, cmdMin",
    "value": "1000, −1000",
    "unit": "N",
    "where": "Thruster.cc L115, L119"
   }
  ],
  "assumptions": "Inverse of GZ-12.",
  "notes": ""
 }
];
