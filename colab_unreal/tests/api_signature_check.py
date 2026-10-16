"""Checks, against the REAL airsim 1.8.1 client installed in the current Python, that every name and call shape airsim_flight.py uses exists.

  /path/to/airsim-venv/bin/python api_signature_check.py
This does not start a simulator. It reads the client's own signatures (inspect.signature) and fails on a missing name or parameter.
"""

import inspect
import sys

import airsim

problems = []


def need(cond, text):
    if not cond:
        problems.append(text)


C = airsim.MultirotorClient
for name, params in {
    "confirmConnection": [], "enableApiControl": ["is_enabled"], "armDisarm": ["arm"], "simPause": ["is_paused"],
    "simContinueForTime": ["seconds"], "hoverAsync": [], "getMultirotorState": [], "simGetCollisionInfo": [],
    "moveByVelocityZAsync": ["vx", "vy", "z", "duration", "drivetrain", "yaw_mode"],
    "simGetImages": ["requests", "vehicle_name", "external"],
    "simSetCameraPose": ["camera_name", "pose", "vehicle_name", "external"],
}.items():
    need(hasattr(C, name), f"MultirotorClient.{name} missing")
    if hasattr(C, name):
        have = list(inspect.signature(getattr(C, name)).parameters)
        for p in params:
            need(p in have, f"{name} has no parameter {p} (has {have})")
need("timeout_value" in inspect.signature(C.__init__).parameters, "MultirotorClient has no timeout_value")
for name in ("ImageRequest", "ImageType", "DrivetrainType", "YawMode", "Pose", "Vector3r", "to_quaternion"):
    need(hasattr(airsim, name), f"airsim.{name} missing")
need(hasattr(airsim.ImageType, "Scene"), "ImageType.Scene missing")
need(hasattr(airsim.DrivetrainType, "MaxDegreeOfFreedom"), "DrivetrainType.MaxDegreeOfFreedom missing")
r = airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)
y = airsim.YawMode(False, 12.0)
q = airsim.to_quaternion(-0.39, 0.0, 1.0)
p = airsim.Pose(airsim.Vector3r(1, 2, 3), q)
need(all(hasattr(q, a) for a in ("w_val", "x_val", "y_val", "z_val")), "quaternion field names differ")
need(hasattr(p, "position") and hasattr(p, "orientation"), "Pose field names differ")
print("airsim client", airsim.__version__, "| problems:", problems or "none")
sys.exit(1 if problems else 0)
