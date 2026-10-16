#!/usr/bin/env bash
# Start Project AirSim (Unreal Engine 5, software Vulkan) and run the client for ONE film.
# usage: launch_film.sh <film id> <out dir> <unpacked Blocks dir> <sim_config dir> <python of the venv> [extra client args...]
# Unreal Engine refuses to run as root: on GitHub's Linux machine the job already runs as the plain user "runner", so no new user is made.
# The flag -AllowCPUDevices is required: without it UE5 exits with status 1 right after it finds the CPU Vulkan device (measured, run 38854645422).
set -u
FILM=$1; OUT=$2; BIN=$3; SIMCFG=$4; PY=$5; shift 5
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LVP=$(ls /usr/share/vulkan/icd.d/lvp_icd*.json | head -n 1)
SH=$(find "$BIN" -maxdepth 3 -name "*.sh" | head -n 1)
chmod +x "$SH"; find "$BIN" -type f -path "*Binaries/Linux/*" -exec chmod +x {} \; 2>/dev/null
echo "id: $(id)  script: $SH  lvp: $LVP" | tee "$OUT/launch_info.txt"
Xvfb :99 -screen 0 1280x1024x24 +extension GLX +render -noreset -ac > "$OUT/xvfb.log" 2>&1 &
XV=$!
sleep 3
export DISPLAY=:99 VK_ICD_FILENAMES="$LVP" LIBGL_ALWAYS_SOFTWARE=1 SDL_AUDIODRIVER=dummy
T0=$(date +%s)
( cd "$(dirname "$SH")"; exec bash "$SH" -RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed -AllowCPUDevices > "$OUT/sim.log" 2>&1 ) &
SIM=$!
sleep 8
if ! kill -0 "$SIM" 2>/dev/null; then echo "Unreal exited within 8 s" | tee -a "$OUT/launch_info.txt"; fi
if [ -n "${CLIENT_SCRIPT:-}" ]; then   # another client (the RT15 probe): no --film
  timeout -k 10 "${CLIENT_TIMEOUT:-3300}" "$PY" "$HERE/$CLIENT_SCRIPT" --out "$OUT" --sim-config "$SIMCFG" "$@" > "$OUT/client.log" 2>&1
else
  timeout -k 10 "${CLIENT_TIMEOUT:-3300}" "$PY" "$HERE/pasim_films.py" --film "$FILM" --out "$OUT" --sim-config "$SIMCFG" "$@" > "$OUT/client.log" 2>&1
fi
RC=$?
echo "client exit status $RC after $(( $(date +%s) - T0 )) s" | tee -a "$OUT/launch_info.txt"
echo "unreal alive at the end: $(kill -0 "$SIM" 2>/dev/null && echo yes || echo no)" | tee -a "$OUT/launch_info.txt"
# stop Unreal: the process group of the subshell
pkill -TERM -P "$SIM" 2>/dev/null; kill "$SIM" 2>/dev/null; sleep 3
pkill -KILL -f "Binaries/Linux/" 2>/dev/null
kill "$XV" 2>/dev/null
# logs written by Unreal
mkdir -p "$OUT/unreal_logs"
find "$BIN" -path "*Saved/Logs*" -name "*.log" -size -30M -exec cp {} "$OUT/unreal_logs/" \; 2>/dev/null
find "$HOME/.config/Epic" -name "*.log" -size -30M -exec cp {} "$OUT/unreal_logs/" \; 2>/dev/null
exit 0
