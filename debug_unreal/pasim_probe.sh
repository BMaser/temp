#!/usr/bin/env bash
# Project AirSim 1.0.1 (UE5 Shipping) start under strace. usage: pasim_probe.sh <bin dir> <evidence dir> <name> <unreal args...>
set -u
BIN=$1; OUT=$2; NAME=$3; shift 3
ARGS="$*"
E="$OUT/$NAME"
mkdir -p "$E"
LVP=$(ls /usr/share/vulkan/icd.d/lvp_icd*.json | head -n 1)
SH=$(find "$BIN" -maxdepth 3 -name "*.sh" | head -n 1)
chmod +x "$SH"; find "$BIN" -type f -path "*Binaries/Linux/*" -exec chmod +x {} \; 2>/dev/null
echo "script $SH args: $ARGS lvp $LVP" | tee "$E/info.txt"
touch "$E/.marker"; sleep 1
export DISPLAY=:99 VK_ICD_FILENAMES="$LVP" LIBGL_ALWAYS_SOFTWARE=1 SDL_AUDIODRIVER=dummy
t0=$(date +%s.%N)
( cd "$(dirname "$SH")"; timeout -k 5 ${SIM_TIMEOUT:-90} strace -f -tt -s 400 -e trace=openat,write,writev,sendmsg,exit_group,execve -o "$E/strace.txt" bash "$SH" $ARGS > "$E/sim.log" 2>&1; echo "exit status $?" > "$E/exit_status.txt" ) &
W=$!
if [ -n "${PA_CLIENT:-}" ]; then nohup setsid timeout 150 "$PA_PY" "$(dirname "$0")/pasim_client.py" "$E" "$PA_SIMCFG" > "$E/client.log" 2>&1 < /dev/null &; fi
for s in 2 5 10 20 40; do
  now=$(date +%s.%N); d=$(python3 -c "print(max(0,$s-($now-$t0)))"); sleep "$d"
  case $s in 2|5|10) import -window root "$E/shot_${s}s.png" 2>"$E/shot_${s}s.err" || true ;; esac
  { echo "== ss at ${s}s"; ss -ltnp; echo "== procs"; ps -eo pid,etimes,stat,comm | grep -i -E "blocks|Shipping" | head; } > "$E/ss_${s}s.txt" 2>&1
done
wait $W
cat "$E/exit_status.txt"
# the analysis
S="$E/strace.txt"
ls -la "$S"
tail -n 200 "$S" > "$E/last200_before_exit.txt"
grep -n "exit_group" "$S" | tail -n 20 > "$E/exit_group_lines.txt"
# all text written to fd 1 / 2 (anything the program printed)
grep -E 'write\((1|2),|writev\((1|2),' "$S" | cut -c1-600 > "$E/stdout_stderr_writes.txt"
# vulkan / graphics related files that were opened
grep -E 'openat.*(vulkan|icd\.d|implicit_layer|explicit_layer|lvp|libvulkan|libGL|libEGL|vk_)' "$S" | cut -c1-400 > "$E/opened_vulkan_files.txt"
# anything that looks like an error text anywhere
grep -i -E 'error|fail|cannot|unable|compatible|profile|not found|fatal|message' "$S" | grep -E 'write|writev|sendmsg' | cut -c1-800 > "$E/error_text_writes.txt"
# X11 messages / SDL message box
grep -E 'sendmsg|MessageBox|zenity|kdialog|xmessage' "$S" | cut -c1-400 | head -n 50 > "$E/x11_dialog_traces.txt"
# files opened for writing
grep -E 'openat.*O_(WRONLY|RDWR|CREAT)' "$S" | cut -c1-300 | grep -v -E '/proc/|/dev/(null|shm)' > "$E/files_opened_for_writing.txt"
wc -l "$E"/*.txt | tee "$E/line_counts.txt"
gzip -9 -f "$S"
# P2: every log or crash file anywhere new
{ echo "== ~/.config/Epic"; find ~/.config/Epic -type f 2>/dev/null | head -n 100
  echo "== Saved under the package"; find "$BIN" -path "*Saved*" -type f 2>/dev/null | head -n 100
  echo "== /tmp"; find /tmp -newer "$E/.marker" -type f 2>/dev/null | grep -v "$OUT" | head -n 100
  echo "== new files anywhere in the package"; find "$BIN" -newer "$E/.marker" -type f 2>/dev/null | head -n 100; } > "$E/p2_listing.txt"
mkdir -p "$E/p2_files"
grep -E '^/' "$E/p2_listing.txt" | while read -r f; do
  if [ -f "$f" ] && [ "$(stat -c %s "$f")" -lt 20000000 ]; then cp --parents "$f" "$E/p2_files/" 2>/dev/null; fi
done
rm -f "$E/.marker"
exit 0
