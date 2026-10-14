"""LSY's single-drone racing environment plus flat floor pads (start and end), without changing any LSY file.

How the environment is built, and why this class is the same kind of object (all line numbers are from the
sources read, lsy_drone_racing commit ac7377b and Gymnasium v1.3.0):
  - `gymnasium.make("DroneRacing-v0", ...)` finds the registration made in lsy_drone_racing/envs/__init__.py:
    entry point `lsy_drone_racing.envs.drone_race:DroneRaceEnv`, max_episode_steps=1500,
    disable_env_checker=True (the vector entry point is not used by `make`).
  - Gymnasium's `make` (registration.py lines 641-830) calls the entry point with the registered kwargs updated
    by the `make` kwargs (lines 687-688 and 735), then wraps: no PassiveEnvChecker (disabled, line 802),
    OrderEnforcing (order_enforce is True by default, line 806), TimeLimit(1500) (line 813).
  - Here `register` below makes the id "DroneRacingPads-v0" with the SAME settings (max_episode_steps=1500,
    disable_env_checker=True, order_enforce default) and the entry point `PadDroneRaceEnv`, a subclass of
    `DroneRaceEnv`. `make` accepts a class as entry point (`callable(env_spec.entry_point)`, line 693) and the
    wrapper chain is therefore the same, and `env.unwrapped` is a `DroneRaceEnv`.
  - RaceCoreEnv.__init__ (race_core.py lines 333-440) calls `self._load_track_into_sim(track)` (line 394) right
    after it builds the Sim, and before `use_box_collision`, the contact masks and the environment data. That
    method (lines 677-702) attaches the gates and poles to `self.sim.spec` and ends with `self.sim.build_mjx()`
    (line 701), which compiles the model once. `PadDroneRaceEnv` overrides that one method: it attaches the pad
    bodies to the same `self.sim.spec` first, in the same way as LSY attaches a gate (new frame on the world
    body, `frame.attach_body(body, "", ":0")`, set `pos`), and then calls the original, so the one compile
    inside the original includes the pads. Nothing else is overridden.
The pads are plain static bodies: no joint, not mocap (so LSY's render sync, which writes mocap positions of
gates and poles, never touches them), and their geom has contype 0 and conaffinity 0 (see assets/pads/*.xml).
"""

from pathlib import Path

import gymnasium
import mujoco
from lsy_drone_racing.envs.drone_race import DroneRaceEnv

PADS_DIR = Path(__file__).resolve().parent.parent / "assets" / "pads"
PAD_ENV_ID = "DroneRacingPads-v0"
BASE_ENV_ID = "DroneRacing-v0"
# pad name -> model file in the pads folder. The body inside each file has the same name as the pad.
PAD_FILES = {"pad_start": "pad_start.xml", "pad_end": "pad_end.xml"}
PAD_HEIGHT_M = 0.001  # the pad spans z = 0 to 0.001 m above its body origin (the geom is in the pad xml)


class PadDroneRaceEnv(DroneRaceEnv):
    """DroneRaceEnv with flat pads on the floor.

    Extra keyword argument `pads`: {"pad_start": (x, y), "pad_end": (x, y)}; either key may be missing.
    The body of each pad is placed at (x, y, 0) and has no rotation, so the top of the picture points to +y.
    Extra keyword argument `pads_dir`: folder with the pad model files (default: assets/pads of this repository).
    """

    def __init__(self, *args, pads: dict | None = None, pads_dir: str | Path | None = None, **kwargs):
        # These must exist before the parent __init__ runs, because that calls _load_track_into_sim.
        self._pads = {name: (float(xy[0]), float(xy[1])) for name, xy in (pads or {}).items()}
        unknown = sorted(set(self._pads) - set(PAD_FILES))
        if unknown:
            raise ValueError(f"unknown pad names {unknown}; known: {sorted(PAD_FILES)}")
        self._pads_dir = Path(pads_dir) if pads_dir else PADS_DIR
        self._pad_specs = []  # keep the pad specs alive for as long as the environment lives
        super().__init__(*args, **kwargs)

    def _load_track_into_sim(self, track):
        """Attach the pads, then let LSY attach gates and poles and compile the model (one compile for all)."""
        for name, (x, y) in self._pads.items():
            path = self._pads_dir / PAD_FILES[name]
            if not path.is_file():
                raise FileNotFoundError(f"pad model {path} does not exist")
            spec = mujoco.MjSpec.from_file(str(path))
            self._pad_specs.append(spec)
            body = spec.body(name)
            if body is None:
                raise ValueError(f"body {name!r} not found in {path}")
            frame = self.sim.spec.worldbody.add_frame()
            pad = frame.attach_body(body, "", ":0")
            pad.pos = [x, y, 0.0]
        super()._load_track_into_sim(track)


def register_pad_env() -> str:
    """Register the id of the pad environment (same settings as LSY's registration) and return the id."""
    gymnasium.register(
        id=PAD_ENV_ID,
        entry_point=PadDroneRaceEnv,
        max_episode_steps=1500,  # same as lsy_drone_racing/envs/__init__.py
        disable_env_checker=True,  # same as lsy_drone_racing/envs/__init__.py
    )
    return PAD_ENV_ID
