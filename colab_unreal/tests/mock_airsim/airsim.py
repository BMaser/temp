"""A MOCK of the AirSim Python client, only for testing airsim_flight.py and the film code without AirSim.

It is NOT AirSim. It has the same names and call shapes that airsim_flight.py uses (checked against PythonClient/airsim/client.py of
AirSim v1.8.1 by api_signature_check.py), a very simple drone (velocity follows the command with a 0.3 s delay, height follows the
command with a 0.5 s delay, yaw turns at most 120 degrees per second) and pictures that are drawn here (sky, ground, posts), so
the films differ from picture to picture. Anything made with it must be called a mock test, never a simulator film.
"""

import math

import numpy as np

__version__ = "mock"


class ImageType:
    Scene = 0


class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1


class YawMode:
    def __init__(self, is_rate=True, yaw_or_rate=0.0):
        self.is_rate, self.yaw_or_rate = is_rate, yaw_or_rate


class Vector3r:
    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val, self.y_val, self.z_val = x_val, y_val, z_val


class Quaternionr:
    def __init__(self, w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0):
        self.w_val, self.x_val, self.y_val, self.z_val = w_val, x_val, y_val, z_val


class Pose:
    def __init__(self, position_val=None, orientation_val=None):
        self.position = position_val or Vector3r()
        self.orientation = orientation_val or Quaternionr()


def to_quaternion(pitch, roll, yaw):
    t0, t1 = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    t2, t3 = math.cos(roll * 0.5), math.sin(roll * 0.5)
    t4, t5 = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return Quaternionr(w_val=t0 * t2 * t4 + t1 * t3 * t5, x_val=t0 * t3 * t4 - t1 * t2 * t5,
                       y_val=t0 * t2 * t5 + t1 * t3 * t4, z_val=t1 * t2 * t4 - t0 * t3 * t5)


class ImageRequest:
    def __init__(self, camera_name, image_type, pixels_as_float=False, compress=True):
        self.camera_name, self.image_type = str(camera_name), image_type


class _Resp:
    def __init__(self, w, h, data):
        self.width, self.height, self.image_data_uint8 = w, h, data


class _Obj:
    pass


def _render(cn, ce, cd, yaw, pitch, w=640, h=480):
    """BGR picture: blue sky (more blue than red), green ground, red posts every 10 m; the horizon moves with the pitch."""
    img = np.zeros((h, w, 3), np.uint8)
    hor = int(h * (0.5 + 0.9 * math.tan(-pitch) * 0.5))
    hor = min(max(hor, 5), h - 5)
    img[:hor, :, 0] = 235          # B
    img[:hor, :, 1] = np.linspace(170, 215, hor)[:, None]
    img[:hor, :, 2] = 130          # R
    img[hor:, :, 1] = 120
    img[hor:, :, 0] = 50
    img[hor:, :, 2] = 60
    for post_n in range(-60, 61, 10):
        for post_e in range(-60, 61, 10):
            dn, de = post_n - cn, post_e - ce
            fwd = dn * math.cos(yaw) + de * math.sin(yaw)
            side = -dn * math.sin(yaw) + de * math.cos(yaw)
            if fwd < 1.0:
                continue
            u = int(w / 2 + side / fwd * w / 2)
            top = int(hor - 300 / fwd)
            if 0 <= u < w - 3:
                img[max(top, 0):hor + int(60 / fwd), u:u + 3] = (30, 30, 200)
    return img


class MultirotorClient:
    def __init__(self, ip="", port=41451, timeout_value=3600):
        self.t_ns = 10_000_000_000
        self.p = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0]
        self.yaw = 0.0
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.paused = False
        self.chase = (0.0, 0.0, -2.5, 0.0, 0.0)
        self.armed = False

    def confirmConnection(self):
        return True

    def enableApiControl(self, is_enabled, vehicle_name=""):
        return True

    def armDisarm(self, arm, vehicle_name=""):
        self.armed = arm
        return True

    def simPause(self, is_paused):
        self.paused = is_paused

    def hoverAsync(self, vehicle_name=""):
        self.cmd = (0.0, 0.0, self.p[2], self.yaw)

    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain=0, yaw_mode=None, vehicle_name=""):
        self.cmd = (vx, vy, z, math.radians(yaw_mode.yaw_or_rate) if yaw_mode is not None else self.yaw)
        return None

    def simContinueForTime(self, seconds):
        steps = int(round(seconds / 0.01))
        dt = seconds / max(steps, 1)
        for _ in range(steps):
            a = 1 - math.exp(-dt / 0.3)
            self.v[0] += (self.cmd[0] - self.v[0]) * a
            self.v[1] += (self.cmd[1] - self.v[1]) * a
            self.p[0] += self.v[0] * dt
            self.p[1] += self.v[1] * dt
            self.p[2] += (self.cmd[2] - self.p[2]) * (1 - math.exp(-dt / 0.5))
            diff = (self.cmd[3] - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += max(-math.radians(120) * dt, min(math.radians(120) * dt, diff))
            self.t_ns += int(dt * 1e9)

    def getMultirotorState(self, vehicle_name=""):
        st, k, pos, ori = _Obj(), _Obj(), Vector3r(*self.p), to_quaternion(0.0, 0.0, self.yaw)
        k.position, k.orientation = pos, ori
        st.kinematics_estimated, st.timestamp = k, self.t_ns
        return st

    def simGetCollisionInfo(self, vehicle_name=""):
        c = _Obj()
        c.has_collided = False
        return c

    def simSetCameraPose(self, camera_name, pose, vehicle_name="", external=False):
        q = pose.orientation
        yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val), 1 - 2 * (q.y_val ** 2 + q.z_val ** 2))
        pitch = math.asin(max(-1, min(1, 2 * (q.w_val * q.y_val - q.z_val * q.x_val))))
        self.chase = (pose.position.x_val, pose.position.y_val, pose.position.z_val, yaw, pitch)

    def simGetImages(self, requests, vehicle_name="", external=False):
        out = []
        for r in requests:
            if external:
                if r.camera_name != "Chase":
                    raise RuntimeError("unknown external camera " + r.camera_name)
                cn, ce, cd, yaw, pitch = self.chase
            else:
                if r.camera_name != "front":
                    raise RuntimeError("unknown camera " + r.camera_name)
                cn, ce, cd, yaw, pitch = self.p[0], self.p[1], self.p[2], self.yaw, 0.0
            img = _render(cn, ce, cd, yaw, pitch)
            out.append(_Resp(640, 480, img.tobytes()))
        return out
