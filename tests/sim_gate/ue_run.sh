#!/usr/bin/env bash
# Simulator gate, simulators 4 and 5 (AirSim and Project AirSim, both Unreal Engine): start the Linux binary in each variant
# and measure with airsim_gate.py / projectairsim_gate.py. Run from the repository root:  bash tests/sim_gate/ue_run.sh <airsim|project_airsim>
#
# Variants (name | runner | Unreal arguments | ClockSpeed for AirSim):
#   vulkan        host      -RenderOffScreen, software Vulkan (lavapipe of the runner, Ubuntu 24.04, Mesa 25)
#   opengl        host      -RenderOffScreen -opengl4 (software OpenGL; AirSim's cooked shaders are Vulkan only, so this is expected to fail)
#   vulkan_u2204  docker    same as vulkan, inside an ubuntu:22.04 container (older Mesa lavapipe), network host
#   nullrhi       host      -nullrhi: no drawing at all, physics only
#   nullrhi_fast  host      (AirSim only) -nullrhi with ClockSpeed 20, to see how fast physics can run
# Blocks (small control world) is tried with every variant; the outdoor world is tried with the first variant whose camera worked.
set -u
SIM=${1:?usage: ue_run.sh <airsim|project_airsim>}
case $SIM in
  airsim)
    OUT=results/airsim; PY=.airsim-venv/bin/python; SCRIPT=tests/sim_gate/airsim_gate.py; BIN=airsim_bin; OUTDOOR=Africa_Savannah
    VARIANTS="vulkan opengl nullrhi nullrhi_fast vulkan_u2204"; MEASURE_EXTRA="" ;;
  project_airsim)
    OUT=results/project_airsim; PY=.pasim-venv/bin/python; SCRIPT=tests/sim_gate/projectairsim_gate.py; BIN=pasim_bin; OUTDOOR=LandscapeMountains
    VARIANTS="vulkan opengl nullrhi vulkan_u2204"
    MEASURE_EXTRA="--sim-config $(pwd)/ProjectAirSim_src/client/python/example_user_scripts/sim_config"
    SC=ProjectAirSim_src/client/python/example_user_scripts/sim_config
    # a copy of the shipped scene whose clock is stepped every 0.1 ms of real time instead of every 3 ms (physics speed test)
    sed 's/"real-time-update-rate": 3000000/"real-time-update-rate": 100000/' $SC/scene_basic_drone.jsonc > $SC/scene_basic_drone_fast.jsonc ;;
  *) echo "unknown sim $SIM"; exit 2 ;;
esac
mkdir -p "$OUT/runs"

export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 +extension GLX +render -noreset > "$OUT/xvfb.log" 2>&1 &
XVFB_PID=$!
sleep 3
LVP=$(ls /usr/share/vulkan/icd.d/lvp_icd*.json 2>/dev/null | head -n 1)
echo "lavapipe ICD file: ${LVP:-none}" | tee -a "$OUT/log.txt"
{ echo "== vulkaninfo --summary (lavapipe, host) =="; VK_ICD_FILENAMES="$LVP" vulkaninfo --summary 2>&1 | grep -E "Vulkan Instance|apiVersion|driverVersion|deviceName|driverName|driverInfo" ; } >> "$OUT/log.txt" || true

write_settings() {   # AirSim only: write_settings <ClockSpeed> <home dir>
  mkdir -p "$2/Documents/AirSim"
  cat > "$2/Documents/AirSim/settings.json" <<EOF
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockSpeed": $1,
  "CameraDefaults": {
    "CaptureSettings": [ { "ImageType": 0, "Width": 640, "Height": 480, "FOV_Degrees": 90 } ]
  },
  "Vehicles": { "Drone1": { "VehicleType": "SimpleFlight", "AutoCreate": true } }
}
EOF
  cp "$2/Documents/AirSim/settings.json" "$OUT/settings_used_clock$1.json"
}

diag_logs() {   # Unreal writes its own log files; show the newest ones
  local world=$1 variant=$2 f
  for f in $(find $BIN/$world -name "*.log" -newer "$OUT/xvfb.log" 2>/dev/null | head -n 3); do
    echo "---- [$variant/$world] Unreal log $f (last 60 lines)" >> "$OUT/log.txt"
    tail -n 60 "$f" >> "$OUT/log.txt"
    cp "$f" "$OUT/unreal_${variant}__${world}_$(basename "$f")" 2>/dev/null
  done
  ls -la $BIN/$world | head -n 20 >> "$OUT/log.txt"
}

describe_binary() {   # missing shared libraries and the start script
  local world=$1 sh bin
  sh=$(find "$BIN/$world" -maxdepth 3 -name "*.sh" | head -n 1)
  { echo "---- start script $sh"; cat "$sh"; bin=$(find "$BIN/$world" -type f -path "*Binaries/Linux/*" ! -name "*.so*" ! -name "*.sym" -perm -u+x 2>/dev/null | head -n 1)
    [ -z "$bin" ] && bin=$(find "$BIN/$world" -type f -path "*Binaries/Linux/*" ! -name "*.so*" ! -name "*.sym" | head -n 1)
    echo "---- binary $bin"; ls -la "$bin"; echo "---- ldd (not found only)"; ldd "$bin" 2>&1 | grep -i "not found"; echo "(end ldd)"; } >> "$OUT/log.txt" 2>&1
}

run_variant() {   # run_variant <variant> <world>
  local variant=$1 world=$2 sh args runner=host clock=1 simpid simlog t0 pypid rc
  sh=$(find "$BIN/$world" -maxdepth 3 -name "*.sh" | head -n 1)
  if [ -z "$sh" ]; then echo "[$variant/$world] no start script found"; ls -R "$BIN/$world" | head -n 30; return 1; fi
  chmod +x "$sh" 2>/dev/null
  find "$BIN/$world" -type f -path "*Binaries/Linux/*" -exec chmod +x {} \; 2>/dev/null
  case $variant in
    vulkan)       args="-RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed" ;;
    opengl)       args="-RenderOffScreen -opengl4 -ResX=640 -ResY=480 -nosound -windowed" ;;
    vulkan_u2204) args="-RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed"; runner=docker ;;
    nullrhi)      args="-nullrhi -nosound" ;;
    nullrhi_fast) args="-nullrhi -nosound"; clock=20 ;;
  esac
  local home="$HOME"
  [ "$runner" = docker ] && home="$(pwd)/.home_docker"
  [ "$SIM" = airsim ] && write_settings "$clock" "$home"
  [ "$SIM" = airsim ] && [ "$runner" != docker ] && write_settings "$clock" "$HOME"
  simlog="$OUT/sim_${variant}__${world}.log"
  echo "=== [$variant/$world] start ($runner): $sh $args  (ClockSpeed $clock)" | tee -a "$OUT/log.txt"
  if [ "$runner" = docker ]; then
    docker rm -f ue2204 >/dev/null 2>&1
    docker run --rm --name ue2204 --network host -v "$(pwd)":/w -w /w -e HOME="$home" -e DISPLAY=:99 -e LIBGL_ALWAYS_SOFTWARE=1 ubuntu:22.04 bash -c "
      export DEBIAN_FRONTEND=noninteractive
      apt-get update -qq && apt-get install -y -qq --no-install-recommends xvfb xauth mesa-vulkan-drivers libvulkan1 vulkan-tools libgl1 libgl1-mesa-dri libglu1-mesa \
        libsdl2-2.0-0 libxi6 libxrandr2 libxcursor1 libxinerama1 libnss3 libasound2 ca-certificates x11-xkb-utils > /dev/null
      Xvfb :99 -screen 0 1280x1024x24 +extension GLX +render -noreset > /dev/null 2>&1 &
      sleep 3
      echo 'container vulkan:'; vulkaninfo --summary 2>&1 | grep -E 'apiVersion|driverVersion|deviceName|driverInfo'
      export VK_ICD_FILENAMES=\$(ls /usr/share/vulkan/icd.d/lvp_icd*.json | head -n 1)
      echo \"ICD \$VK_ICD_FILENAMES\"
      useradd -m -u \$(stat -c %u /w) ue
      su ue -c \"export HOME=$home DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 VK_ICD_FILENAMES=\$VK_ICD_FILENAMES; cd /w; bash $sh $args\"
    " > "$simlog" 2>&1 &
    simpid=$!
  else
    case $variant in
      vulkan|nullrhi*) export VK_ICD_FILENAMES="$LVP" ;;
      *)               unset VK_ICD_FILENAMES ;;
    esac
    export LIBGL_ALWAYS_SOFTWARE=1
    setsid bash "$sh" $args > "$simlog" 2>&1 &
    simpid=$!
  fi
  t0=$(date +%s)
  timeout 600 "$PY" "$SCRIPT" measure --variant "$variant" --world "$world" $MEASURE_EXTRA > "$OUT/measure_${variant}__${world}.log" 2>&1 &
  pypid=$!
  local lastss=0
  while kill -0 "$pypid" 2>/dev/null; do
    sleep 5
    local now=$(date +%s)
    if [ $(( now - lastss )) -ge 60 ]; then lastss=$now; echo "[$variant/$world] t+$(( now - t0 )) s listening ports:" >> "$OUT/log.txt"; ss -ltn 2>/dev/null | grep -E "4145|8989|8990" >> "$OUT/log.txt"; fi
    if ! kill -0 "$simpid" 2>/dev/null && [ $(( now - t0 )) -gt 30 ]; then
      echo "[$variant/$world] the sim process ended early; stopping the client" | tee -a "$OUT/log.txt"
      wait "$simpid" 2>/dev/null; echo "[$variant/$world] sim exit status $?" | tee -a "$OUT/log.txt"
      kill "$pypid" 2>/dev/null
      diag_logs "$world" "$variant"
      break
    fi
  done
  wait "$pypid"; rc=$?
  echo "=== [$variant/$world] client exit code $rc after $(( $(date +%s) - t0 )) s" | tee -a "$OUT/log.txt"
  tail -n 60 "$simlog" | cut -c1-300 >> "$OUT/log.txt"
  tail -n 15 "$OUT/measure_${variant}__${world}.log" | cut -c1-300 >> "$OUT/log.txt"
  diag_logs "$world" "$variant"
  if [ "$runner" = docker ]; then docker rm -f ue2204 >/dev/null 2>&1; else kill -TERM -- "-$simpid" 2>/dev/null; sleep 3; kill -KILL -- "-$simpid" 2>/dev/null; fi
  sleep 3
  return $rc
}

describe_binary Blocks
good=""
for v in $VARIANTS; do
  if run_variant "$v" Blocks; then good="$good $v"; fi
done
echo "variants with a good camera on Blocks:$good" | tee -a "$OUT/log.txt"
if [ -d "$BIN/$OUTDOOR" ]; then
  for v in $good; do
    run_variant "$v" "$OUTDOOR" && break
  done
fi
kill "$XVFB_PID" 2>/dev/null
"$PY" "$SCRIPT" merge
