#!/usr/bin/env bash
# Simulator gate, simulator 3 (PX4 + Gazebo): start the simulation headless under Xvfb + Mesa software rendering and run
# px4_gazebo_gate.py against it, once per world: baylands (outdoor, Gazebo Fuel models) and default (grey plane).
# Run from the repository root, after `make px4_sitl_default` has built px4/.
set -u
OUT=results/px4_gazebo
mkdir -p "$OUT"
export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 +extension GLX +render -noreset > "$OUT/xvfb.log" 2>&1 &
XVFB_PID=$!
sleep 3
export LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 GZ_VERBOSE=2
gz sim --versions >> "$OUT/log.txt" 2>&1 || true
glxinfo -B >> "$OUT/log.txt" 2>&1 || true

for world in baylands default; do
  echo "=== PX4 + Gazebo: world $world" | tee -a "$OUT/log.txt"
  rm -f "$OUT/gate.json" "$OUT"/cam_*.png
  ( cd px4 && sleep 1500 | setsid make px4_sitl "gz_x500_mono_cam_${world}" > "../$OUT/sim.log" 2>&1 ) &
  MAKE_PID=$!
  python3 tests/sim_gate/px4_gazebo_gate.py 2>&1 | tee -a "$OUT/log.txt"
  tail -n 60 "$OUT/sim.log" >> "$OUT/log.txt"
  cp "$OUT/sim.log" "$OUT/sim_${world}.log"
  [ -f "$OUT/gate.json" ] && mv "$OUT/gate.json" "$OUT/gate_${world}.json"
  [ -f "$OUT/measurements.json" ] && mv "$OUT/measurements.json" "$OUT/measurements_${world}.json"
  for i in 1 2 3; do [ -f "$OUT/cam_$i.png" ] && mv "$OUT/cam_$i.png" "$OUT/${world}_cam_$i.png"; done
  # stop PX4 + Gazebo
  pkill -TERM -P "$MAKE_PID" 2>/dev/null; kill -TERM "$MAKE_PID" 2>/dev/null
  pkill -f "build/px4_sitl_default/bin/px4" 2>/dev/null; pkill -f "gz sim" 2>/dev/null
  sleep 8
done
kill "$XVFB_PID" 2>/dev/null
python3 tests/sim_gate/px4_merge.py
exit 0
