#!/usr/bin/env bash
# Run the FR film up to 3 times: CityEnviron sometimes segfaults while the scene loads (run 39002605108: "Signal 11", Connection reset after 97 s;
# the same happened in F02 runs 38979221353 and 38981676570). A failed attempt that ended within 900 s without a finished capture is repeated.
# usage: run_fr.sh <key> <bin dir> <sim_config> <python> <variant> <rtf> [extra client args]   (env UNREAL_EXTRA, UNREAL_RESX, UNREAL_RESY as for launch_film.sh)
set -u
KEY=$1; BIN=$2; SIMCFG=$3; PY=$4; VARIANT=$5; RTF=$6; shift 6
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$PWD/out/$KEY"
mkdir -p "$PWD/out/attempts_$KEY"
for ATT in 1 2 3; do
  R=$RTF; [ "$ATT" = "3" ] && R=0.04   # last resort if the low clock speed itself were the cause of a load crash
  CLIENT_SCRIPT=films_fr.py CLIENT_TIMEOUT=18500 bash "$HERE/launch_film.sh" "FR" "$OUT" "$BIN" "$SIMCFG" "$PY" --variant "$VARIANT" --rtf "$R" "$@"
  SECS=$(grep -o 'after [0-9]* s' "$OUT/launch_info.txt" | tail -n 1 | grep -o '[0-9]*')
  if grep -q '"stage": "done"' "$OUT/capture.json" 2>/dev/null; then echo "attempt $ATT finished"; break; fi
  echo "attempt $ATT did not finish (client ran ${SECS:-?} s)"
  mkdir -p "$PWD/out/attempts_$KEY/$ATT"
  cp "$OUT/sim.log" "$OUT/client.log" "$OUT/launch_info.txt" "$PWD/out/attempts_$KEY/$ATT/" 2>/dev/null
  if [ "${SECS:-99999}" -gt 900 ]; then echo "too late to repeat"; break; fi
  pkill -KILL -f "Binaries/Linux/" 2>/dev/null; sleep 5
  rm -rf "$OUT"
done
mkdir -p "$OUT/attempts" && cp -r "$PWD/out/attempts_$KEY/." "$OUT/attempts/" 2>/dev/null
exit 0
