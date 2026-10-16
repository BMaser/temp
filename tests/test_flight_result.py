"""Tests of tools/flight_result.py. Run with plain `python tests/test_flight_result.py`; exit code 1 on a failure."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import flight_result as fr  # noqa: E402


# ---------------------------------------------------------------- helpers (written independently of the tool)

def qmul(a, b):
    """Hamilton product of two quaternions in the order x y z w."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def axis_quat(axis, deg):
    h = math.radians(deg) / 2
    v = np.zeros(4)
    v[{"x": 0, "y": 1, "z": 2}[axis]] = math.sin(h)
    v[3] = math.cos(h)
    return v


def rotate(q, v):
    """Rotate vector v by quaternion q (body to world), by the sandwich product."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    conj = np.array([-q[0], -q[1], -q[2], q[3]])
    return qmul(qmul(q, qv), conj)[:3]


def make_flight(n=10, passed=None, seq=(2, 0, 1), end="finished", dropped=0):
    time = np.arange(n) / 50.0
    pos = np.stack([0.1 * np.arange(n), np.zeros(n), np.full(n, 0.5)], axis=1)
    vel = np.tile([5.0, 0.0, 0.0], (n, 1))
    quat = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
    passed = np.array(passed if passed is not None else [0] * n, dtype=np.int64)
    gates_pos = np.array([[0.2, 0.0, 0.5], [0.5, 0.0, 0.5], [0.8, 0.0, 0.5]])
    a = {"time": time, "pos": pos, "vel": vel, "quat": quat, "passed": passed, "gates_pos": gates_pos,
         "gates_quat": np.tile([0.0, 0.0, 0.0, 1.0], (3, 1)), "gate_sequence": np.array(seq, dtype=np.int64),
         "gate_direction": np.ones(len(seq), dtype=np.int64), "obstacles_pos": np.array([[1.0, 0.5, 1.5]]),
         "pad_start": np.array([0.0, 0.0]), "pad_end": np.array([1.0, 1.0]), "freq": np.array(50)}
    child = {"tag": "test", "end": end, "rows_saved": n, "passages": int(passed[-1]), "gates_total": len(seq),
             "terminated_steps_dropped": dropped, "versions": {"mujoco": "1"}, "nose_mode": "forward", "yaw_rate_max": 4.0}
    layout = {"title": "x", "start": {"pos": [0.0, 0.0, 0.01], "yaw_rad": 0.0},
              "gates": [{"pos": g.tolist(), "yaw_rad": 0.0, "shape": "square"} for g in gates_pos],
              "obstacles": [{"pos": [1.0, 0.5, 1.5]}],
              "level_file_safety_limits": {"pos_limit_low": [-2.5, -1.5, -0.001], "pos_limit_high": [2.5, 1.5, 2.0]}}
    return a, child, layout


def write_inputs(d: Path, a, child, layout):
    np.savez(d / "f.npz", **a)
    (d / "c.json").write_text(json.dumps(child))
    (d / "l.json").write_text(json.dumps(layout))
    return ["--npz", str(d / "f.npz"), "--child", str(d / "c.json"), "--layout", str(d / "l.json"), "--out", str(d / "out.json")]


# ---------------------------------------------------------------- tests

def yaw_ok(convert) -> bool:
    for deg in (0, 90, 180, -90):
        _, _, yaw = convert(np.array([axis_quat("z", deg)]))
        if abs((yaw[0] - deg + 180) % 360 - 180) > 1e-6:
            return False
    return True


def test_yaw_from_quaternion():
    assert yaw_ok(fr.quat_to_attitude_deg)


def test_yaw_check_fails_on_a_wrong_sign():  # negative control: the check above can fail
    assert not yaw_ok(lambda q: tuple(-v for v in fr.quat_to_attitude_deg(q)))


def test_pitch_positive_is_nose_up():
    q = axis_quat("y", -20)  # tips the nose up
    nose = rotate(q, [1, 0, 0])
    assert nose[2] > 0.3  # geometry says: nose is up
    roll, pitch, yaw = fr.quat_to_attitude_deg(np.array([q]))
    assert abs(pitch[0] - math.degrees(math.asin(nose[2]))) < 1e-9 and abs(pitch[0] - 20) < 1e-9
    assert abs(roll[0]) < 1e-9 and abs(yaw[0]) < 1e-9
    _, pitch_down, _ = fr.quat_to_attitude_deg(np.array([axis_quat("y", 15)]))
    assert abs(pitch_down[0] + 15) < 1e-9  # nose down is negative


def test_roll_positive_is_left_bank():
    for yaw_deg in (0, 90):
        q = qmul(axis_quat("z", yaw_deg), axis_quat("x", -25))
        thrust, nose = rotate(q, [0, 0, 1]), rotate(q, [1, 0, 0])
        left_level = np.array([-math.sin(math.radians(yaw_deg)), math.cos(math.radians(yaw_deg)), 0.0])  # left of the nose, horizontal
        assert np.dot(thrust, left_level) > 0.4  # geometry says: the thrust leans to the left of the nose
        assert abs(np.dot(nose, [math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)), 0]) - 1) < 1e-9
        roll, pitch, yaw = fr.quat_to_attitude_deg(np.array([q]))
        assert abs(roll[0] - 25) < 1e-9 and abs(pitch[0]) < 1e-9 and abs(yaw[0] - yaw_deg) < 1e-9
    roll, _, _ = fr.quat_to_attitude_deg(np.array([axis_quat("x", -25)]))
    assert roll[0] > 0
    assert abs(fr.tilt_deg(np.array([qmul(axis_quat("z", 40), axis_quat("x", -25))]))[0] - 25) < 1e-9


def test_passage_events_at_the_rise_of_passed():
    a, child, layout = make_flight(passed=[0, 0, 1, 1, 1, 2, 2, 3, 3, 3], seq=(2, 0, 1))
    assert fr.find_passage_rows(a["passed"]) == [(2, 1), (5, 2), (7, 3)]
    res = fr.build(a, child, layout)
    ev = [e for e in res["events"] if e["type"] == "gate_passage"]
    assert [e["row"] for e in ev] == [2, 5, 7]
    assert [e["gate"] for e in ev] == [3, 1, 2]  # gate_sequence (2, 0, 1) counted from 1
    assert [e["passage"] for e in ev] == [1, 2, 3]
    assert [e["time_s"] for e in ev] == [0.04, 0.1, 0.14]
    assert res["summary"]["passage_times_s"] == [0.04, 0.1, 0.14]
    assert res["summary"]["split_times_s"] == [0.04, 0.06, 0.04]
    assert res["events"][-1] == {"type": "end", "reason": "finished", "time_s": 0.18, "row": 9}
    assert res["summary"]["lap_time_s"] == 0.18 and res["summary"]["passages"] == 3


def test_passage_events_negative_control():  # a flight that never passes has no passage events
    a, child, layout = make_flight(passed=[0] * 10, end="time")
    res = fr.build(a, child, layout)
    assert [e for e in res["events"] if e["type"] == "gate_passage"] == []
    assert res["events"][-1]["reason"] == "time_over" and res["summary"]["lap_time_s"] is None


def test_crash_event():
    a, child, layout = make_flight(passed=[0, 0, 1, 1, 1, 1, 1, 1, 1, 1], end="crash", dropped=1)
    res = fr.build(a, child, layout)
    end = res["events"][-1]
    assert end["type"] == "end" and end["reason"] == "crash"
    assert end["time_s"] == 0.2  # last saved 0.18 + the one dropped step of 0.02
    assert res["summary"]["lap_time_s"] is None and res["summary"]["passages"] == 1
    assert res["gui"]["run"]["ok"] is False


def test_left_box_event_and_its_control():
    a, child, layout = make_flight(end="crash", dropped=1)
    a["pos"][-1] = [2.49, 0.0, 0.5]  # next step (+5 m/s * 0.02 s = +0.1 m) goes beyond x = 2.5
    assert fr.build(a, child, layout)["events"][-1]["reason"] == "left_box"
    a["pos"][-1] = [0.9, 0.0, 0.5]
    assert fr.build(a, child, layout)["events"][-1]["reason"] == "crash"


def test_speed_is_length_of_velocity():
    a, child, layout = make_flight()
    a["vel"][:3] = [[3.0, 4.0, 0.0], [1.0, 2.0, 2.0], [0.0, 0.0, 0.0]]
    res = fr.build(a, child, layout)
    assert res["samples"]["speed_mps"][:3] == [5.0, 3.0, 0.0]
    assert res["summary"]["top_speed_mps"] == 5.0
    assert res["gui"]["path"]["v"][:3] == [5.0, 3.0, 0.0]
    assert res["samples"]["speed_mps"][:3] != [4.0, 2.0, 0.0]  # not the largest component either


def test_pole_clearance_by_hand():
    pos = np.array([[0.0, 0.0, 0.5], [2.0, 0.0, 0.5]])
    time = np.array([0.0, 2.0])
    c, t, k, axis = fr.min_pole_clearance(pos, time, [[1.0, 0.5, 1.5]])
    assert abs(axis - 0.5) < 1e-12 and abs(c - 0.485) < 1e-12 and abs(t - 1.0) < 1e-12 and k == 1
    # the rows alone are 1.118 m away: a row-only measure would be wrong by more than half a meter
    assert abs(math.hypot(1.0, 0.5) - axis) > 0.6
    # above the pole top + 0.1 m nothing counts
    assert fr.min_pole_clearance(pos + [0, 0, 1.2], time, [[1.0, 0.5, 1.5]]) is None
    # the nearer of two poles wins and is numbered from 1
    c2 = fr.min_pole_clearance(pos, time, [[1.0, 0.5, 1.5], [0.5, -0.2, 1.5]])
    assert c2[2] == 2 and abs(c2[3] - 0.2) < 1e-12 and abs(c2[0] - 0.185) < 1e-12


def lists_same_length(samples: dict) -> bool:
    lens = {len(v) for v in samples.values() if isinstance(v, list)}
    return len(lens) == 1


def test_file_written_reads_back_and_lists_match():
    a, child, layout = make_flight(passed=[0, 0, 1, 1, 1, 2, 2, 3, 3, 3])
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        (d / "film.mp4").write_bytes(b"x")
        args = write_inputs(d, a, child, layout) + ["--films", "film.mp4", "--run-id", "7", "--seed", "0"]
        assert fr.main(args) == 0
        res = json.loads((d / "out.json").read_text(encoding="utf-8"))
    assert res["format"] == "flight_result" and res["version"] == 1
    s = res["samples"]
    assert lists_same_length(s) and s["count"] == 10 and len(s["time_s"]) == 10
    assert len([v for v in s.values() if isinstance(v, list)]) == 16
    assert len(res["gui"]["path"]["S"]) == 10 and len(res["gui"]["path"]["yaw"]) == 10 and len(res["gui"]["run"]["profile"]) == 61
    assert res["films"] == ["film.mp4"] and res["layout"]["gate_order"] == [3, 1, 2]
    assert res["layout"]["end"]["pos"] == [1.0, 1.0, 0.0]
    # negative control: a list that is one shorter is caught
    broken = dict(s)
    broken["x_m"] = broken["x_m"][:-1]
    assert not lists_same_length(broken)


def test_rounding_digits():
    a, child, layout = make_flight()
    a["pos"][3] = [0.123456789, -0.987654321, 0.5]
    res = fr.build(a, child, layout)
    assert res["samples"]["x_m"][3] == 0.1235 and res["samples"]["y_m"][3] == -0.9877


def test_bad_inputs_stop_with_message():
    good = [0, 0, 1, 1, 1, 2, 2, 3, 3, 3]
    a, child, layout = make_flight(passed=good)
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        child["rows_saved"] = 99
        assert fr.main(write_inputs(d, a, child, layout)) == 1 and not (d / "out.json").exists()
        a, child, layout = make_flight(passed=good)
        a["vel"][2, 1] = float("nan")
        assert fr.main(write_inputs(d, a, child, layout)) == 1
        a, child, layout = make_flight(passed=good)
        layout["gates"][0]["pos"][0] += 0.5  # layout of another track
        assert fr.main(write_inputs(d, a, child, layout)) == 1
        a, child, layout = make_flight(passed=good)
        assert fr.main(write_inputs(d, a, child, layout) + ["--title", "LSY level 0"]) == 1  # a forbidden name
        assert fr.main(write_inputs(d, a, child, layout) + ["--films", "missing.mp4"]) == 1
        assert fr.main(write_inputs(d, a, child, layout)) == 0  # control: the same good input passes


TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]


def main() -> int:
    passed = 0
    for t in TESTS:
        try:
            t()
            passed += 1
            print(f"PASS {t.__name__}")
        except Exception:  # noqa: BLE001
            print(f"FAIL {t.__name__}")
            traceback.print_exc()
    print(f"{passed} of {len(TESTS)} passed")
    return 0 if passed == len(TESTS) else 1


if __name__ == "__main__":
    sys.exit(main())
