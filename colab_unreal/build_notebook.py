"""Builds the Colab notebook from notebook_source.py and the helper files.

  python build_notebook.py OUT.ipynb

notebook_source.py is written as plain Python with `# %%` and `# %% [markdown]` cell markers. The four helper files
(route.py, unreal_films.py, airsim_flight.py, projectairsim_flight.py) are pasted into the cell that writes them to disk
(the placeholders @@name@@), so the notebook is one self-contained file.
"""

import json
import re
import sys
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
HELPERS = ["route.py", "unreal_films.py", "airsim_flight.py", "projectairsim_flight.py"]


def build(out: Path) -> None:
    src = (HERE / "notebook_source.py").read_text(encoding="utf-8")
    for name in HELPERS:
        body = (HERE / name).read_text(encoding="utf-8")
        if "'''" in body or body.rstrip().endswith("\\"):
            raise SystemExit(f"{name} cannot be pasted into a raw triple-quoted string")
        src = src.replace(f"@@{name}@@", body.rstrip("\n"))
    cells, cur_kind, cur = [], None, []

    def flush():
        if cur_kind is None:
            return
        lines = list(cur)
        while lines and not lines[0].strip():
            lines.pop(0)
        while lines and not lines[-1].strip():
            lines.pop()
        if not lines:
            return
        if cur_kind == "markdown":
            text = [re.sub(r"^# ?", "", l) for l in lines]
            cells.append({"cell_type": "markdown", "metadata": {}, "source": [t + "\n" for t in text[:-1]] + [text[-1]], "id": uuid.uuid4().hex[:8]})
        else:
            cells.append({"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [],
                          "source": [l + "\n" for l in lines[:-1]] + [lines[-1]], "id": uuid.uuid4().hex[:8]})

    for line in src.splitlines():
        m = re.match(r"^# %%( \[markdown\])?\s*$", line)
        if m:
            flush()
            cur_kind = "markdown" if m.group(1) else "code"
            cur = []
        else:
            cur.append(line)
    flush()
    nb = {"cells": cells,
          "metadata": {"accelerator": "GPU",
                       "colab": {"name": "racing_drone_unreal_films.ipynb", "provenance": [], "gpuType": "T4"},
                       "kernelspec": {"display_name": "Python 3", "name": "python3"},
                       "language_info": {"name": "python"}},
          "nbformat": 4, "nbformat_minor": 5}
    out.write_text(json.dumps(nb, indent=1, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    build(Path(sys.argv[1]))
    print("written", sys.argv[1])
