"""Checks a built notebook without a GPU.

  python check_notebook.py NOTEBOOK.ipynb [OUT_SCRIPT.py]

1. the file is valid JSON in notebook format 4;
2. every code cell compiles (each one on its own, because Colab runs them one by one);
3. no @@placeholder@@ is left;
4. the four helper programs inside the notebook are byte for byte the files next to this script;
5. the code cells exported as one script compile with py_compile (and the script is written when OUT_SCRIPT is given).
Exit code 1 on any failure.
"""

import json
import py_compile
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
HELPERS = ["route.py", "unreal_films.py", "airsim_flight.py", "projectairsim_flight.py"]


def main() -> int:
    nb_path = Path(sys.argv[1])
    problems = []
    nb = json.loads(nb_path.read_text(encoding="utf-8"))
    if nb.get("nbformat") != 4:
        problems.append("nbformat is not 4")
    code = [("".join(c["source"]), i) for i, c in enumerate(nb["cells"]) if c["cell_type"] == "code"]
    print(f"{len(nb['cells'])} cells, {len(code)} code cells")
    for src, i in code:
        try:
            compile(src, f"cell {i}", "exec")
        except SyntaxError as e:
            problems.append(f"cell {i} does not compile: {e}")
        if re.search(r"@@[\w.]+@@", src):
            problems.append(f"cell {i} still has a placeholder")
    whole = "\n".join(src for src, _ in code)
    for name in HELPERS:
        body = (HERE / name).read_text(encoding="utf-8").rstrip("\n")
        if body not in whole:
            problems.append(f"{name} in the notebook differs from the file")
    script = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(tempfile.mkdtemp()) / "script.py"
    subprocess.run([sys.executable, str(HERE / "nb_to_script.py"), str(nb_path), str(script)], check=True)
    try:
        py_compile.compile(str(script), doraise=True)
        print("py_compile of the exported script: ok")
    except py_compile.PyCompileError as e:
        problems.append(f"py_compile failed: {e}")
    for name in HELPERS:
        try:
            py_compile.compile(str(HERE / name), doraise=True)
        except py_compile.PyCompileError as e:
            problems.append(f"{name}: {e}")
    print("PROBLEMS:", problems or "none")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
