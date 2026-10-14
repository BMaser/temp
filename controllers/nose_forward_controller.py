"""State controller that flies the example path and keeps the nose (and so the camera) along the direction of flight.

This is the example controller of lsy_drone_racing at commit ac7377b (control/state_controller.py) with ONE change:
the yaw. The start point, the 11 other points, the 22.5 s flight time, the cubic spline, and the zero `vel`, `acc` and
`ang_vel` entries of the command are the same, so a difference between this flight and the example flight comes from
the yaw alone.

Yaw. The example commands yaw 0.0 at every step (line 88 of the example), so the nose never turns and the drone flew
backward where the track turns back. Here the whole yaw plan is computed once, in `__init__`, on the control time grid
t_k = k / freq, from the first derivative of the spline (the planned velocity), with `controllers/nose_yaw.py`:
the heading of the planned horizontal velocity, started early and limited to `YAW_RATE_MAX` rad/s.

What crazyflow does with the command (crazyflow commit d28ec70, control/mellinger/control.py): of the quaternion only
the yaw is used (line 103, `R.from_quat(setpoint_quat).as_euler("xyz")[..., 2]`), and the three `ang_vel` numbers
(command entries 13 to 16) are staged as the body-rate setpoint of the attitude loop (line 656, used at line 672).
Roll and pitch of the quaternion are ignored; position, velocity and acceleration entries give the thrust direction.

Class attributes (the cloud test sets them on the class before it makes the controller):
  NOSE_MODE     "forward" = the plan above; "fixed" = yaw 0.0 at every step, the same command as the example.
  YAW_RATE_MAX  largest yaw rate of the plan in rad/s.
  V_MIN         horizontal planned speed in m/s below which the plan holds its last heading.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent))  # load_controller loads this file by path
import nose_yaw  # noqa: E402

from lsy_drone_racing.control import Controller  # noqa: E402

if TYPE_CHECKING:
    from numpy.typing import NDArray

# The 11 points after the start point: copied from lsy_drone_racing/control/state_controller.py lines 48-60 at ac7377b.
POINTS_AFTER_START = np.array(
    [
        [-1.0, 0.75, 0.4],
        [0.3, 0.35, 0.7],
        [1.3, -0.15, 0.9],
        [0.85, 0.85, 1.2],
        [-0.5, -0.05, 0.7],
        [-1.2, -0.2, 0.8],
        [-1.2, -0.2, 1.2],
        [-0.0, -0.7, 1.2],
        [1.2, -0.15, 1.2],
        [1.05, 0.75, 1.2],
        [0.25, 1.25, 1.2],
    ]
)
T_TOTAL = 22.5  # s, as in the example (line 62)


class NoseForwardController(Controller):
    """Example path, yaw along the direction of flight."""

    NOSE_MODE = "forward"
    YAW_RATE_MAX = 2.0  # rad/s
    V_MIN = 0.15  # m/s

    def __init__(self, obs: dict[str, NDArray[np.floating]], info: dict, config: dict):
        """Build the spline (as the example does) and the whole yaw plan.

        Args:
            obs: The initial observation. `obs["pos"]` is the first point of the path, `obs["quat"]` (order x y z w)
                gives the start yaw of the plan.
            info: The initial environment information from the reset.
            config: The race configuration; `config.env.freq` is the control frequency.
        """
        super().__init__(obs, info, config)
        self._freq = config.env.freq
        mode = self.NOSE_MODE
        if mode not in ("forward", "fixed"):
            raise ValueError(f"NOSE_MODE must be 'forward' or 'fixed', got {mode!r}")

        start_pos = obs["pos"]
        waypoints = np.concatenate((np.asarray(start_pos, dtype=float).reshape(1, 3), POINTS_AFTER_START), axis=0)
        self._t_total = T_TOTAL
        t = np.linspace(0, self._t_total, len(waypoints))
        self._des_pos_spline = CubicSpline(t, waypoints)

        self._tick = 0
        self._finished = False

        # One plan value for every tick 0 .. round(T_TOTAL * freq): the last tick is the one that ends the flight.
        n = int(round(self._t_total * self._freq)) + 1
        self._times = np.arange(n) / self._freq
        self._start_yaw = float(nose_yaw.yaw_from_quat_xyzw(np.asarray(obs["quat"], dtype=float)))
        if mode == "fixed":
            self._plan = np.zeros(n)  # yaw 0.0, the command of the example
        else:
            velocity = self._des_pos_spline(self._times, 1)[:, :2]  # first derivative: planned velocity
            self._plan = nose_yaw.plan_yaw(
                velocity, 1.0 / self._freq, self.YAW_RATE_MAX, self.V_MIN, self._start_yaw
            )

    def planned_yaw(self, tick: int) -> float:
        """Planned yaw in rad at control tick `tick` (ticks past the end give the last value)."""
        return float(self._plan[min(max(int(tick), 0), len(self._plan) - 1)])

    def plan(self) -> NDArray[np.floating]:
        """A copy of the whole yaw plan, one value per tick."""
        return self._plan.copy()

    def planned_xy(self, n: int = 200) -> NDArray[np.floating]:
        """Horizontal positions of the planned path at n evenly spaced times, shape (n, 2)."""
        return self._des_pos_spline(np.linspace(0, self._t_total, n))[:, :2]

    def compute_control(
        self, obs: dict[str, NDArray[np.floating]], info: dict | None = None
    ) -> NDArray[np.floating]:
        """Next desired state [x, y, z, vx, vy, vz, ax, ay, az, qx, qy, qz, qw, wx, wy, wz]."""
        t = min(self._tick / self._freq, self._t_total)
        if t >= self._t_total:  # Maximum duration reached
            self._finished = True

        des_pos = self._des_pos_spline(t)
        des_yaw = self.planned_yaw(self._tick)
        des_quat = R.from_euler("z", des_yaw).as_quat()  # order x y z w
        action = np.concatenate((des_pos, np.zeros(6), des_quat, np.zeros(3)), dtype=np.float32)
        return action

    def step_callback(
        self,
        action: NDArray[np.floating],
        obs: dict[str, NDArray[np.floating]],
        reward: float,
        terminated: bool,
        truncated: bool,
        info: dict,
    ) -> bool:
        """Increment the time step counter. Returns True when the flight time is over."""
        self._tick += 1
        return self._finished

    def episode_callback(self):
        """Reset the internal state."""
        self._tick = 0
