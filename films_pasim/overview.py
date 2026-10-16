"""overview.png: one row per film (F??_row.png from make_film.py), stacked in film order.
usage: python overview.py <folder holding the film folders> <out png>"""
import sys
from pathlib import Path

from PIL import Image

root, out = Path(sys.argv[1]), Path(sys.argv[2])
rows = sorted(root.rglob("F*_row.png"), key=lambda p: p.name)
if not rows:
    print("no rows found")
    sys.exit(1)
imgs = [Image.open(p).convert("RGB") for p in rows]
w = max(i.width for i in imgs)
sheet = Image.new("RGB", (w, sum(i.height for i in imgs)), (0, 0, 0))
y = 0
for im in imgs:
    sheet.paste(im, (0, y))
    y += im.height
sheet.save(out)
print("overview of", [p.name for p in rows], sheet.size)
