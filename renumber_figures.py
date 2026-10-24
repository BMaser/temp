"""Renumber the figures of ONE section page 1..N in page order (each page numbers its own figures).
Captions must start "<figcaption>Figure LABEL." where LABEL is anything (12a, T-net-explain-1, 0, ...).
Every "Figure LABEL" / "Figures LABEL" / "Figures L1-L2" in the page that names one of this page's labels is rewritten in
one pass; references to labels that are not on this page (another page's figures) are left alone and listed.
Run on the cloud machine only:  python renumber_figures.py page.html  (rewrites the file, prints the map as JSON)
Copy of v2\\renumber_figures.py with one fix (draft 2.2, 2026-10-20): the original computed "sorted(list) - set", which raises a
TypeError before the file is written; here the list is made from a set difference instead."""
import json
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
labels = re.findall(r"<figcaption>Figure ([^\s.]+)\.", text)
if len(labels) != len(set(labels)):
    sys.exit(f"duplicate caption labels: {labels}")
new = {old: str(i + 1) for i, old in enumerate(labels)}
alt = "|".join(re.escape(l) for l in sorted(labels, key=len, reverse=True))
rng = re.compile(rf"\b(Figures?) ({alt})\s?([-–])\s?({alt})(?![\w-])")
one = re.compile(rf"\b(Figures?) ({alt})(?![\w-])")
text = rng.sub(lambda m: f"{m.group(1)} {new[m.group(2)]}{m.group(3)}{new[m.group(4)]}", text)
text = one.sub(lambda m: f"{m.group(1)} {new[m.group(2)]}", text)
left = sorted(set(re.findall(r"\bFigures? ([^\s,.;:)<]+)", text)) - set(new.values()))
path.write_text(text, encoding="utf-8")
print(json.dumps({"page": path.name, "map": new, "references_not_on_this_page": left}, ensure_ascii=False))
