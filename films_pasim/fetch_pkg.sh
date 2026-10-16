#!/usr/bin/env bash
# Download and unpack one Project AirSim v1.0.1 scene package, and keep its license facts.
# usage: fetch_pkg.sh <Blocks|LandscapeMountains|Neighborhood|CityEnviron> <dest dir> <evidence dir>
set -u
PKG=$1; DEST=$2; EV=$3
BASE="https://github.com/iamaisim/ProjectAirSim/releases/download/v1.0.1"
mkdir -p "$DEST" "$EV"
if [ "$PKG" = "CityEnviron" ]; then
  for p in 001 002; do curl -fL --retry 5 -sS -o "CityEnviron.zip.$p" "$BASE/CityEnviron-Linux-1.0.1.zip.$p" || echo "download part $p failed"; done
  ls -la CityEnviron.zip.* | tee "$EV/zip_size.txt"
  cat CityEnviron.zip.00* > CityEnviron.zip
  rm -f CityEnviron.zip.00*
  unzip -q CityEnviron.zip -d "$DEST" || { sudo apt-get install -y -qq p7zip-full >/dev/null; 7z x -y -o"$DEST" CityEnviron.zip > "$EV/7z.log" 2>&1; }
  rm -f CityEnviron.zip
else
  curl -fL --retry 5 -sS -o pkg.zip "$BASE/$PKG-Linux-1.0.1.zip" || echo "download failed"
  ls -la pkg.zip | tee "$EV/zip_size.txt"
  unzip -q pkg.zip -d "$DEST"; rm -f pkg.zip
fi
du -sh "$DEST" | tee "$EV/unpacked_size.txt"
# license facts: every license-like or readme-like text file inside the package, and the release notes of the release
{
  echo "== files named like license / notice / readme / third party (depth 6)"
  find "$DEST" -maxdepth 6 -type f \( -iname "*license*" -o -iname "*notice*" -o -iname "*readme*" -o -iname "*third*party*" -o -iname "*attribution*" -o -iname "*credits*" \) 2>/dev/null | head -n 60
} > "$EV/license_files.txt"
find "$DEST" -maxdepth 6 -type f \( -iname "*license*" -o -iname "*notice*" -o -iname "*readme*" -o -iname "*third*party*" -o -iname "*credits*" \) -size -400k 2>/dev/null | head -n 12 | while read -r f; do
  echo "=================== $f"; head -n 60 "$f"
done > "$EV/license_heads.txt" 2>/dev/null
find "$DEST" -maxdepth 5 -type f | head -n 80 > "$EV/package_top_files.txt"
gh api repos/iamaisim/ProjectAirSim/releases/tags/v1.0.1 --jq '.body' > "$EV/release_notes_v1.0.1.txt" 2>&1 || echo "release notes not fetched" > "$EV/release_notes_v1.0.1.txt"
gh api repos/iamaisim/ProjectAirSim/license --jq '.license.spdx_id' > "$EV/repo_license.txt" 2>&1 || true
wc -c "$EV"/license_heads.txt "$EV"/release_notes_v1.0.1.txt
