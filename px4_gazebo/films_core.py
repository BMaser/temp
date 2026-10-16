"""Pure-Python part of the PX4 + Gazebo feature films (slice 1: F05 night, F06 clouds and rain; the job can make any subset of F01 to F11).

No Gazebo, no MAVSDK and no PIL import here, so tests/test_px4_gazebo_films.py can run it anywhere.

What a film is: one or more CONDITIONS (a condition = the same short flight of the x500 in the world baylands, with the world text
patched by `patch_world` and the model patched by `x500_model_text`). The drone camera (nose) and a chase camera are filmed side by
side. All look settings are SDF elements of the world file: the `scene` block (ambient, background, sky), the directional light
`sunUTC`, extra lights (point lights on posts, a spot light on the drone) and particle emitters (rain, snow, the Fuel model
"Fog Generator").

World coordinates: the drone spawns at about world (0, 0) (base run: ground reference (-0.0017, 0.00006, -0.0988)), so the extras are
placed in world coordinates around the origin; the drone flies from (0, 0) to (MOVE_X, 0) heading east (+x).
"""

from __future__ import annotations

import re

# ------------------------------------------------------------------------------------------------ the flight of every condition
SEG_S = 7.5           # simulated seconds from the arm command to the end of a segment
CLIMB_Z = 2.0         # m, hover height
CLIMB_RATE = 0.8      # m/s
MOVE_START_S = 2.5    # s after arm: start to fly east
MOVE_X = 4.0          # m east
MOVE_V = MOVE_X / (SEG_S - MOVE_START_S)  # 0.8 m/s
YAW_EAST_NED_DEG = 90.0

PANEL = (640, 480)    # one camera picture
FILM_FPS = 15

FUEL_FOG_URI = "https://fuel.gazebosim.org/1.0/openrobotics/models/fog generator"

# ------------------------------------------------------------------------------------------------ world pieces


def sun_xml(direction: str, diffuse: str, specular: str, intensity: float, cast_shadows: bool = False) -> str:
    return (f"<light name='sunUTC' type='directional'><pose>0 0 500 0 -0 0</pose><cast_shadows>{'true' if cast_shadows else 'false'}</cast_shadows>"
            f"<intensity>{intensity}</intensity><direction>{direction}</direction><diffuse>{diffuse}</diffuse><specular>{specular}</specular>"
            f"<attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation></light>")


def scene_xml(ambient: str, background: str, sky_xml: str = "", fog_xml: str = "") -> str:
    return (f"<scene><ambient>{ambient} 1</ambient><background>{background} 1</background><grid>false</grid>{sky_xml}"
            f"<shadows>1</shadows>{fog_xml}</scene>")


def lamp_xml(k: int, x: float, y: float, z0: float = -0.1) -> str:
    """A street lamp: grey post, glowing bulb and a point light (SDF light element, type point)."""
    return (f"<model name='lamp_{k}'><static>true</static><pose>{x} {y} {z0} 0 0 0</pose><link name='l'>"
            f"<visual name='post'><pose>0 0 1.5 0 0 0</pose><geometry><cylinder><radius>0.05</radius><length>3.0</length></cylinder></geometry>"
            f"<material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>"
            f"<visual name='bulb'><pose>0 0 3.1 0 0 0</pose><geometry><sphere><radius>0.15</radius></sphere></geometry>"
            f"<material><ambient>1 0.9 0.6 1</ambient><diffuse>1 0.9 0.6 1</diffuse><emissive>1 0.9 0.6 1</emissive></material></visual>"
            f"<light name='lamp_light_{k}' type='point'><pose>0 0 3.1 0 0 0</pose><cast_shadows>false</cast_shadows>"
            f"<diffuse>1 0.85 0.55 1</diffuse><specular>0.3 0.3 0.2 1</specular><intensity>2</intensity>"
            f"<attenuation><range>16</range><constant>0.2</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation></light>"
            f"</link></model>")


LAMP_POSITIONS = [(3.0, -3.5), (7.0, 3.0), (-3.0, 3.0), (11.0, -3.0)]


def emitter_xml(name: str, x: float, y: float, z: float, pitch: float, size: str, particle_size: str, lifetime: float, vmin: float, vmax: float,
                rate: float, color_start: str, color_end: str, scale_rate: float = 0.0) -> str:
    """A static model with one box particle emitter. Emission direction = +x of the emitter frame; pitch = +pi/2 sends it down (-z)."""
    return (f"<model name='{name}'><static>true</static><pose>{x} {y} {z} 0 {pitch} 0</pose><link name='{name}_link'>"
            f"<particle_emitter name='{name}_emitter' type='box'><emitting>true</emitting><size>{size}</size><particle_size>{particle_size}</particle_size>"
            f"<lifetime>{lifetime}</lifetime><min_velocity>{vmin}</min_velocity><max_velocity>{vmax}</max_velocity><scale_rate>{scale_rate}</scale_rate>"
            f"<rate>{rate}</rate><color_start>{color_start}</color_start><color_end>{color_end}</color_end>"
            f"<material><diffuse>0.8 0.8 0.85</diffuse><specular>0.3 0.3 0.3</specular></material></particle_emitter></link></model>")


def rain_xml() -> str:
    # a 30 m x 30 m sheet 11 m above the ground, drops going down at 7 to 9 m/s, 1.5 s of life (about 11 m of fall)
    return emitter_xml("rain", 4.0, 0.0, 11.0, 1.5708, "0.1 30 30", "0.03 0.5 0.03", 1.5, 7.0, 9.0, 4000, "0.8 0.85 0.95 0.55", "0.8 0.85 0.95 0.25")


def snow_xml() -> str:
    # flakes of 8 cm falling 0.8 to 1.2 m/s from 8 m, 8 s of life
    return emitter_xml("snow", 4.0, 0.0, 8.0, 1.5708, "0.1 30 30", "0.08 0.08 0.08", 8.0, 0.8, 1.2, 1500, "1 1 1 0.9", "1 1 1 0.7")


def fog_xml() -> str:
    # the ready asset: Gazebo Fuel model "Fog Generator" (OpenRobotics), pose as in the gz-sim particle tutorial (pitch -pi/2: emits up)
    return (f"<include><uri>{FUEL_FOG_URI}</uri><name>fog_generator</name><pose>4 0 -0.1 0 -1.5707 0</pose></include>")


# natural objects close to the flight path (owner's order 2026-10-16), all from Gazebo Fuel (owner OpenRobotics); licenses read from the Fuel
# API on 2026-10-16: Oak tree, Pine Tree, Juniper Tree = Creative Commons Zero v1.0; Falling Rock 1 to 3 = Creative Commons Attribution 4.0.
# (name on Fuel, instance name, x, y, yaw in world coordinates; z = GROUND_Z)
GROUND_Z = -0.1
SCENERY = [
    ("Oak tree", "oak_a", 9.0, 4.5, 0.3), ("Oak tree", "oak_b", 13.0, -5.0, 1.2),
    ("Pine Tree", "pine_a", 6.5, -4.5, 0.0), ("Pine Tree", "pine_b", 11.0, 6.5, 2.0),
    ("Juniper Tree", "juniper_a", 4.5, 3.2, 0.5), ("Juniper Tree", "juniper_b", 7.5, -2.8, 2.5),
    ("Falling Rock 1", "rock_a", 3.5, -2.2, 0.0), ("Falling Rock 2", "rock_b", 6.0, 2.2, 1.0), ("Falling Rock 3", "rock_c", 8.5, -1.5, 2.0),
]
FUEL_BASE = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"


def scenery_xml() -> str:
    return "".join(f"<include><uri>{FUEL_BASE}{n}</uri><name>{i}</name><pose>{x} {y} {GROUND_Z} 0 0 {yaw}</pose></include>" for n, i, x, y, yaw in SCENERY)


# ROOT CAUSE found in run 38950664623: a <plugin> in the world file switches OFF the default systems of PX4's server.config (gz-sim logs "No systems
# loaded from SDF, loading defaults" only without it): no scene broadcaster, so /world/baylands/scene/info never appears and PX4 times out after 30 s.
# The particle emitter system is therefore NOT added to the main rain, snow and fog conditions (the emitter is drawn without it); the probe
# "probe_rain_fullplugins" adds it together with a copy of the default systems of px4/src/modules/simulation/gz_bridge/server.config.
FULL_SERVER_PLUGINS = "".join(f"<plugin filename='{f}' name='{n}'>{extra}</plugin>" for f, n, extra in [
    ("gz-sim-physics-system", "gz::sim::systems::Physics", ""), ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", ""),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", ""), ("gz-sim-contact-system", "gz::sim::systems::Contact", ""),
    ("gz-sim-imu-system", "gz::sim::systems::Imu", ""), ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure", ""),
    ("gz-sim-air-speed-system", "gz::sim::systems::AirSpeed", ""), ("gz-sim-apply-link-wrench-system", "gz::sim::systems::ApplyLinkWrench", ""),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat", ""), ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer", ""),
    ("gz-sim-sensors-system", "gz::sim::systems::Sensors", "<render_engine>ogre2</render_engine>"),
    ("libOpticalFlowSystem.so", "custom::OpticalFlowSystem", ""), ("libGstCameraSystem.so", "custom::GstCameraSystem", "")])

PARTICLE_PLUGIN = "<plugin filename='gz-sim-particle-emitter-system' name='gz::sim::systems::ParticleEmitter'></plugin>"

SKY_DEFAULT = None  # keep the sky element the world has (<sky><clouds>true</clouds></sky>)
SKY_NONE = ""       # remove the sky element: only the background colour is drawn (to be measured)
SKY_TIME_22 = ("<sky><time>22</time><sunrise>6</sunrise><sunset>20</sunset>"
               "<clouds><speed>0.6</speed><humidity>0.9</humidity><mean_size>0.5</mean_size></clouds></sky>")

# ------------------------------------------------------------------------------------------------ the conditions
# film: which film the condition belongs to ("P" = probe: a still and a short film that only answer a question about the engine)
OVERCAST = dict(ambient="0.6 0.6 0.65", background="0.66 0.68 0.72", sky=SKY_NONE)
OVERCAST_SUN = dict(direction="0.001 0.625 -0.78", diffuse="0.55 0.55 0.6 1", specular="0.05 0.05 0.05 1", intensity=0.5)

CONDITIONS = {
    "day": dict(film="F05", label="DAY: world as shipped (sun, sky with clouds)", scene=None, sun=None, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "dusk": dict(film="F05", label="DUSK: low warm sun, orange background, no sky element",
                 scene=dict(ambient="0.45 0.32 0.42", background="0.95 0.6 0.35", sky=SKY_NONE),
                 sun=dict(direction="0.9 0.25 -0.12", diffuse="1.0 0.55 0.25 1", specular="0.4 0.2 0.1 1", intensity=0.9),
                 headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "night": dict(film="F05", label="NIGHT: dim blue moon light, dark background, lamps and a spot light on the drone",
                  scene=dict(ambient="0.05 0.05 0.12", background="0.01 0.01 0.035", sky=SKY_NONE),
                  sun=dict(direction="0.3 0.2 -0.9", diffuse="0.10 0.12 0.25 1", specular="0.02 0.02 0.05 1", intensity=0.5),
                  headlamp=True, lamps=True, scenery=True, extras=[], prefill_s=2.0),
    "overcast": dict(film="F06", label="OVERCAST: grey light, grey background, no sky element",
                     scene=OVERCAST, sun=OVERCAST_SUN, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "fog": dict(film="F06", label="OVERCAST + FOG: Fuel model Fog Generator (particle emitter)",
                scene=OVERCAST, sun=OVERCAST_SUN, headlamp=False, lamps=False, scenery=True, extras=["fog"], prefill_s=25.0),
    "rain": dict(film="F06", label="OVERCAST + RAIN: our particle emitter, 4000 drops/s",
                 scene=OVERCAST, sun=OVERCAST_SUN,
                 headlamp=False, lamps=False, scenery=True, extras=["rain"], prefill_s=4.0),
    "snow": dict(film="F06", label="OVERCAST + SNOW: our particle emitter, 1500 flakes/s",
                 scene=OVERCAST, sun=OVERCAST_SUN,
                 headlamp=False, lamps=False, scenery=True, extras=["snow"], prefill_s=9.0),
    # probes: what does the engine do with the sky element and the scene fog element?
    "probe_sky_time_22": dict(film="P", label="PROBE: sky element with time 22, sunset 20, clouds humidity 0.9 (rest as shipped)",
                              scene=dict(ambient="0.8 0.5 1", background="0.7 0.7 0.7", sky=SKY_TIME_22), sun=None, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "probe_no_sky": dict(film="P", label="PROBE: no sky element, background 0.45 0.6 0.9 (rest as shipped)",
                         scene=dict(ambient="0.8 0.5 1", background="0.45 0.6 0.9", sky=SKY_NONE), sun=None, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "probe_scene_fog": dict(film="P", label="PROBE: scene fog element (linear, start 3 m, end 25 m, grey), no sky element",
                            scene=dict(ambient="0.6 0.6 0.65", background="0.7 0.7 0.74", sky=SKY_NONE,
                                       fog="<fog><color>0.7 0.7 0.74 1</color><type>linear</type><start>3</start><end>25</end><density>1</density></fog>"),
                            sun=OVERCAST_SUN, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    # twin of overcast (same world, second run): how much do two identical runs differ? (reference for the rain and fog differences)
    "overcast_twin": dict(film="P", label="TWIN: overcast again", scene=OVERCAST, sun=OVERCAST_SUN, headlamp=False, lamps=False, scenery=True, extras=[], prefill_s=2.0),
    "probe_rain_fullplugins": dict(film="P", label="PROBE: rain emitter with the particle emitter system plugin AND a copy of the default systems",
                                   scene=OVERCAST, sun=OVERCAST_SUN, headlamp=False, lamps=False, scenery=True, extras=["rain_fullplugins"], prefill_s=4.0),
}

FILMS = {
    "F05": dict(short="night", feature="Night", conditions=["day", "dusk", "night"], probes=[], doc_ids=["D15"]),
    "F06": dict(short="clouds_rain", feature="Clouds and rain", conditions=["overcast", "fog", "rain", "snow"], probes=["overcast_twin", "probe_rain_fullplugins"], doc_ids=["D16"]),
}

# the document claims, copied from runs/2026-10-14_feature_matrix/features_data.json (answers.px4_gazebo) and recheck.json
DOC_CLAIMS = {
    "D15": {"s": "yes", "n": "World file sets the sun's direction and color; no time-of-day clock is stated.",
            "q": "<direction>0.001 0.625 -0.78</direction> <diffuse>0.904 0.904 0.904 1</diffuse>",
            "src": "runs/2026-10-14_feature_matrix/sources/px4_gazebo/world_default.sdf"},
    "D16": {"s": "yes", "n": "Gazebo's particle emitter makes smoke and fog that cameras see, and the tutorial shows the effect on camera images. "
                             "The sky can also draw clouds, as the PX4 baylands world does. Rain and snow are not covered.",
            "q": "This tutorial shows how to use the particle emitter system to add and configure particle effects like smoke and fog in simulation.",
            "src": "runs/2026-10-15_px4_gazebo_features/sources/gz_sim/tutorials__particle_tutorial.md"},
}

# set by hand after the contact sheets were looked at (never by the job itself): film -> verdict and the reason
VERDICTS = {
    "F05": dict(verdict="pending_review", why="not reviewed yet"),
    "F06": dict(verdict="pending_review", why="not reviewed yet"),
}


# ------------------------------------------------------------------------------------------------ patching
def world_extras(cond: dict) -> str:
    parts = []
    for e in cond.get("extras", []):
        parts.append({"rain": rain_xml, "rain_fullplugins": rain_xml, "snow": snow_xml, "fog": fog_xml}[e]())
    if cond.get("scenery"):
        parts.append(scenery_xml())
    if cond.get("lamps"):
        parts += [lamp_xml(k, x, y) for k, (x, y) in enumerate(LAMP_POSITIONS)]
    if "rain_fullplugins" in cond.get("extras", []):
        parts.append(FULL_SERVER_PLUGINS + PARTICLE_PLUGIN)
    return "".join(parts)


def patch_world(text: str, cond: dict) -> str:
    """The world text of one condition. Raises when an expected piece is missing, so a changed baylands.sdf is never patched silently."""
    out = text
    sc = cond.get("scene")
    if sc is not None:
        sky = sc.get("sky")
        if sky is SKY_DEFAULT:
            m = re.search(r"<sky>.*?</sky>", text, re.S)
            sky = m.group(0) if m else ""
        out, n = re.subn(r"<scene>.*?</scene>", lambda _m: scene_xml(sc["ambient"], sc["background"], sky, sc.get("fog", "")), out, count=1, flags=re.S)
        if n != 1:
            raise ValueError("no <scene> block in the world")
    sun = cond.get("sun")
    if sun is not None:
        out, n = re.subn(r"<light name=['\"]sunUTC['\"].*?</light>", lambda _m: sun_xml(sun["direction"], sun["diffuse"], sun["specular"], sun["intensity"]),
                         out, count=1, flags=re.S)
        if n != 1:
            raise ValueError("no sunUTC light in the world")
    extras = world_extras(cond)
    if extras:
        k = out.rfind("</world>")
        if k < 0:
            raise ValueError("no </world>")
        out = out[:k] + extras + out[k:]
    return out


def headlamp_link() -> str:
    """A spot light on the drone: a massless link, fixed to base_link, light looking forward and a little down."""
    return ("<link name='headlamp_link'><pose relative_to='base_link'>0.2 0 0.02 0 0 0</pose><gravity>false</gravity>"
            "<inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>"
            "<light name='headlamp' type='spot'><pose>0 0 0 0 0 0</pose><cast_shadows>false</cast_shadows>"
            "<diffuse>1 0.95 0.8 1</diffuse><specular>0.5 0.5 0.4 1</specular><intensity>3</intensity>"
            "<attenuation><range>30</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.002</quadratic></attenuation>"
            "<direction>1 0 -0.25</direction><spot><inner_angle>0.35</inner_angle><outer_angle>0.8</outer_angle><falloff>1.0</falloff></spot></light></link>"
            "<joint name='HeadlampJoint' type='fixed'><parent>base_link</parent><child>headlamp_link</child></joint>")


def x500_model_text(pristine: str, chase_link: str, headlamp: bool) -> str:
    """The x500_mono_cam model.sdf of a condition: pristine text + the chase camera link (+ the headlamp)."""
    k = pristine.rindex("</model>")
    add = chase_link + (headlamp_link() if headlamp else "")
    return pristine[:k] + add + pristine[k:]


def mono_cam_text(pristine: str, width: int, height: int, from_w: str = "1280", from_h: str = "960") -> str:
    return pristine.replace(f"<width>{from_w}</width>", f"<width>{width}</width>").replace(f"<height>{from_h}</height>", f"<height>{height}</height>")


def setpoint(tt: float):
    """Position setpoint (north, east, down in metres, from the spawn point) and velocity feed-forward at tt seconds after arm."""
    z = min(CLIMB_Z, CLIMB_RATE * max(0.0, tt))
    vz = CLIMB_RATE if z < CLIMB_Z else 0.0
    moving = tt >= MOVE_START_S
    x = min(MOVE_X, MOVE_V * (tt - MOVE_START_S)) if moving else 0.0
    vx = MOVE_V if moving and x < MOVE_X else 0.0
    return (0.0, x, -z), (0.0, vx, -vz)
