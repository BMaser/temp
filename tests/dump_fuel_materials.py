"""Why are the trees black? Dump the material part of every Collada mesh of some Fuel models in the cache of the runner (text only):
effect kinds, colours, transparency, image files and whether they exist. Usage: python3 tests/dump_fuel_materials.py out.txt"""
import glob
import os
import re
import sys

root = os.path.expanduser("~/.gz/fuel")
lines = []
for dae in sorted(glob.glob(root + "/**/*.dae", recursive=True)):
    low = dae.lower()
    if not any(k in low for k in ("baylands", "oak", "pine", "juniper", "rock")):
        continue
    t = open(dae, errors="replace").read()
    lines.append(f"=== {dae.replace(root, '')} ({len(t)} bytes)")
    lines.append("  effect kinds: " + str(sorted(set(re.findall(r"<(phong|lambert|blinn|constant)>", t)))))
    lines.append("  transparency/transparent elements: " + str(len(re.findall(r"<transparen", t))) + " ; sample: " + " ".join(re.findall(r"<transparent[^>]*>.{0,80}", t, re.S)[:2]).replace("\n", " ")[:200])
    lines.append("  diffuse samples: " + " | ".join(re.findall(r"<diffuse>(.{0,90}?)</diffuse>", t, re.S)[:4]).replace("\n", " "))
    lines.append("  ambient samples: " + " | ".join(re.findall(r"<ambient>(.{0,90}?)</ambient>", t, re.S)[:3]).replace("\n", " "))
    lines.append("  vertex colour sources (COLOR): " + str(len(re.findall(r'semantic="COLOR"', t))) + "; NORMAL: " + str(len(re.findall(r'semantic="NORMAL"', t))) + "; TEXCOORD: " + str(len(re.findall(r'semantic="TEXCOORD"', t))))
    imgs = re.findall(r"<init_from>([^<]+)</init_from>", t)
    for im in sorted(set(imgs))[:12]:
        cand = glob.glob(os.path.join(os.path.dirname(dae), "..", "**", os.path.basename(im)), recursive=True) + glob.glob(os.path.join(os.path.dirname(dae), "**", os.path.basename(im)), recursive=True)
        lines.append(f"  image {im} -> exists: {bool(cand)}")
for sdf in sorted(glob.glob(root + "/**/model.sdf", recursive=True)):
    if any(k in sdf.lower() for k in ("oak", "pine", "juniper", "falling rock")):
        lines.append(f"=== {sdf.replace(root, '')}")
        lines.append(open(sdf, errors="replace").read()[:1500].replace("\n", " "))
open(sys.argv[1], "w").write("\n".join(lines) + "\n")
print(len(lines), "lines")
