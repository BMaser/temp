#!/usr/bin/env bash
# stamp.sh start <sim>           remember the start time of the install
# stamp.sh end <sim> <0|1>       write install_seconds.txt and install_ok.txt (1 = every install step succeeded)
set -u
d=results/$2
mkdir -p "$d"
if [ "$1" = start ]; then
  date +%s > "$d/t0"
else
  t0=$(cat "$d/t0" 2>/dev/null || date +%s)
  echo $(( $(date +%s) - t0 )) > "$d/install_seconds.txt"
  echo "$3" > "$d/install_ok.txt"
fi
