"""Offline guard: lists names that are read but never defined, imported or built in (what pyflakes calls F821), for the given files.
A function lost in an edit (run 39007672680: night_test and Live) only showed after 8 minutes of build and 1 minute of Gazebo; this shows it in a second.
  python3 tests/check_undefined_names.py tests/px4_gazebo_fr_run.py px4_gazebo/race_film.py ..."""
import ast
import builtins
import sys


def undefined(path):
    t = ast.parse(open(path, encoding="utf-8").read())
    defined = set(dir(builtins)) | {"__file__", "__name__"}
    for n in ast.walk(t):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            defined.add(n.name)
        elif isinstance(n, ast.Import):
            defined.update((a.asname or a.name).split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom):
            defined.update(a.asname or a.name for a in n.names)
        elif isinstance(n, ast.Name) and isinstance(n.ctx, (ast.Store, ast.Del)):
            defined.add(n.id)
        elif isinstance(n, ast.arg):
            defined.add(n.arg)
        elif isinstance(n, ast.ExceptHandler) and n.name:
            defined.add(n.name)
    bad = {}
    for n in ast.walk(t):
        if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load) and n.id not in defined:
            bad.setdefault(n.id, n.lineno)
    return bad


if __name__ == "__main__":
    failed = False
    for p in sys.argv[1:]:
        b = undefined(p)
        print(p, "undefined names:", b if b else "none")
        failed = failed or bool(b)
    sys.exit(1 if failed else 0)
