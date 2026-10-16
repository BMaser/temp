"""Exports the code cells of a notebook as one Python script (markdown cells become comments).

  python nb_to_script.py NOTEBOOK.ipynb OUT.py
"""

import json
import sys
from pathlib import Path

nb = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
parts = []
for i, c in enumerate(nb["cells"]):
    src = "".join(c["source"])
    if c["cell_type"] == "markdown":
        parts.append("\n".join("# " + l for l in src.splitlines()))
    else:
        parts.append(f"# ---- cell {i} ----\n{src}")
Path(sys.argv[2]).write_text("\n\n".join(parts) + "\n", encoding="utf-8")
print("written", sys.argv[2], "from", len(nb["cells"]), "cells")
