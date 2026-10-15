#!/usr/bin/env bash
# Simulator gate, simulator 5 (Project AirSim): start the Linux binary in each variant and measure with projectairsim_gate.py.
# Run from the repository root. Needs: Xvfb, mesa-vulkan-drivers (lavapipe), the unzipped binaries in pasim_bin/<World>/,
# and a python 3.10 venv in .pasim-venv with the airsim client.
#   variants: vulkan   -RenderOffScreen with software Vulkan (lavapipe)
#             opengl   -RenderOffScreen -opengl4 with Mesa software OpenGL (llvmpipe)
#             nullrhi  -nullrhi (no drawing at all; physics only)
# Blocks (small control world) is tried with every variant; the outdoor world (LandscapeMountains) is tried with the
# variants whose camera worked on Blocks.
set -u
OUT=results/project_airsim
mkdir -p "$OUT/runs"
PY=.pasim-venv/bin/python
SCRIPT=tests/sim_gate/projectairsim_gate.py
SIMCFG=$(pwd)/ProjectAirSim_src/client/python/example_user_scripts/sim_config

export DISPLAY=:99
Xvfb :99 -screen 0 1280x1024x24 +extension GLX +render -noreset > "$OUT/xvfb.log" 2>&1 &
XVFB_PID=$!
sleep 3

LVP=$(ls /usr/share/vulkan/icd.d/lvp_icd*.json 2>/dev/null | head -n 1)
echo "lavapipe ICD file: ${LVP:-none}"
{ echo "== vulkaninfo --summary (lavapipe) =="; VK_ICD_FILENAMES="$LVP" vulkaninfo --summary 2>&1 | head -n 40; } >> "$OUT/log.txt" || true

run_variant() {   # run_variant <variant> <world>
  local variant=$1 world=$2 sh args
  sh=$(find "pasim_bin/$world" -maxdepth 3 -name "*.sh" | head -n 1)
  if [ -z "$sh" ]; then echo "[$variant/$world] no start script found"; ls -R "pasim_bin/$world" | head -n 30; return 1; fi
  chmod +x "$sh" 2>/dev/null
  find "pasim_bin/$world" -type f -path "*Binaries/Linux/*" -exec chmod +x {} \; 2>/dev/null
  case $variant in
    vulkan)  args="-RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed"; export VK_ICD_FILENAMES="$LVP"; export LIBGL_ALWAYS_SOFTWARE=1 ;;
    opengl)  args="-RenderOffScreen -opengl4 -ResX=640 -ResY=480 -nosound -windowed"; unset VK_ICD_FILENAMES; export LIBGL_ALWAYS_SOFTWARE=1 ;;
    nullrhi) args="-nullrhi -nosound"; unset VK_ICD_FILENAMES; export LIBGL_ALWAYS_SOFTWARE=1 ;;
  esac
  echo "=== [$variant/$world] start: $sh $args" | tee -a "$OUT/log.txt"
  local simlog="$OUT/sim_${variant}__${world}.log"
  setsid bash "$sh" $args > "$simlog" 2>&1 &
  local simpid=$!
  local t0=$(date +%s)
  timeout 600 "$PY" "$SCRIPT" measure --variant "$variant" --world "$world" --sim-config "$SIMCFG" > "$OUT/measure_${variant}__${world}.log" 2>&1 &
  local pypid=$!
  while kill -0 "$pypid" 2>/dev/null; do
    sleep 5
    if ! kill -0 "$simpid" 2>/dev/null && [ $(( $(date +%s) - t0 )) -gt 30 ]; then
      echo "[$variant/$world] the sim process ended early; stopping the client" | tee -a "$OUT/log.txt"
      kill "$pypid" 2>/dev/null
      break
    fi
  done
  wait "$pypid"; local rc=$?
  echo "=== [$variant/$world] client exit code $rc after $(( $(date +%s) - t0 )) s" | tee -a "$OUT/log.txt"
  tail -n 40 "$simlog" >> "$OUT/log.txt"
  tail -n 15 "$OUT/measure_${variant}__${world}.log" >> "$OUT/log.txt"
  # stop the sim: its own process group
  kill -TERM -- "-$simpid" 2>/dev/null; sleep 3; kill -KILL -- "-$simpid" 2>/dev/null
  sleep 2
  return $rc
}

good_variants=""
for v in vulkan opengl nullrhi; do
  if run_variant "$v" Blocks; then good_variants="$good_variants $v"; fi
done
echo "variants with a good camera on Blocks:$good_variants" | tee -a "$OUT/log.txt"
if [ -d pasim_bin/LandscapeMountains ]; then
  for v in $good_variants; do
    [ "$v" = nullrhi ] && continue
    run_variant "$v" LandscapeMountains && break
  done
fi

kill "$XVFB_PID" 2>/dev/null
"$PY" "$SCRIPT" merge
